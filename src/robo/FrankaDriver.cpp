/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "RoboDriver.hpp"
#include "RoboDriverNetworking.hpp"
#include "RoboNetworkInfo.hpp"
#include "json.hpp"

#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_filters.h>
#include <Rcs_timer.h>

#include <zmq.hpp>

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <array>
#include <mutex>
#include <thread>
#include <iostream>
#include <chrono>

#include <csignal>

static std::atomic<bool> runLoop(true);


class FrankaDriver : public RoboDriver
{
public:

  void registerFeedbackCallback(std::function<void(const std::string&)> cb)
  {
    feedbackFcn = std::move(cb);
  }

  void stop()
  {
  }

  int start()
  {
    try
    {
      double time = 0.0;
      franka::Robot robot("172.16.0.2");
      franka::Model model = robot.loadModel();

      robot.setCollisionBehavior(
      {{20, 20, 20, 20, 20, 20, 20}},  // joint lower
      {{20, 20, 20, 20, 20, 20, 20}},  // joint upper
      {{10, 10, 10, 10, 10, 10}},      // cartesian lower
      {{10, 10, 10, 10, 10, 10}}       // cartesian upper
      );

      robot.setCartesianImpedance({600, 600, 600, 50, 50, 50});
      std::cout << "[INFO] Cartesian impedance set.\n";

      std::unique_ptr<Rcs::RampFilterND> filteredJointCommands;


      robot.control([&](const franka::RobotState& rs, franka::Duration period) -> franka::CartesianPose
      {

        // Send feedback back to remote process
        if (this->feedbackFcn)
        {

          this->feedbackFcn("Hello");
        }




        if (time == 0.0)
        {
          // Create and initialize filters
          const double tmc = 0.1;
          const double dt = 0.001;
          filteredJointCommands = std::make_unique<Rcs::RampFilterND>(tmc, 0.0, dt, getDOF());
          filteredJointCommands->init(rs.q.data());
          for (size_t i = 0; i < filteredJointCommands->getDim(); ++i)
          {
            filteredJointCommands->setMaxVel(getMaxVel()[i], i);
          }

        }

        time += period.toSec();

        // Tool & stiffness frames to use for the FK:
        //    - If you haven't changed them: use rs.F_T_EE and rs.EE_T_K
        //    - If you want different values, pass those matrices instead.
        const auto& F_T_EE = rs.F_T_EE;   // 4x4, column-major
        const auto& EE_T_K = rs.EE_T_K;   // 4x4, column-major

        // Compute desired EE pose from q_des
        std::array<double,7> q_des;
        for (size_t i=0; i<q_des.size(); ++i)
        {
          q_des[i] = filteredJointCommands->getPosition(i);
        }

        std::array<double,16> O_T_EE_des = model.pose(franka::Frame::kEndEffector, q_des, F_T_EE, EE_T_K);
        franka::CartesianPose motion(O_T_EE_des);

        // Compute elbow angle from q vector
        bool withElbow = false;
        if (withElbow)
        {
          constexpr double q_elbow_flip = -0.467002423653011; // from FCI spec (rad)
          constexpr double eps = 1e-6;                         // tolerance
          double flip = (q_des[3] > q_elbow_flip + eps) ?  1.0
                        : (q_des[3] < q_elbow_flip - eps) ? -1.0 : 0.0;

          motion.elbow = { q_des[2], flip }; // [0]=joint3 angle, [1]=flip indicator
        }

        return motion;
      });

    }
    catch (const franka::Exception& e)
    {
      std::cerr << "Franka Exception: " << e.what() << std::endl;
      return -1;
    }

    return 0;
  }

  size_t getDOF() const
  {
    return 7;
  }

  double getMinTMC() const
  {
    return 0.05;
  }

  std::vector<double> getMaxVel() const
  {
    const double maxVel_1_4 = RCS_DEG2RAD(150.0);
    const double maxVel_5_7 = RCS_DEG2RAD(301.0);
    std::vector<double> maxVel = { maxVel_1_4, maxVel_1_4, maxVel_1_4, maxVel_1_4,
                                   maxVel_5_7,  maxVel_5_7,  maxVel_5_7
                                 };
    return maxVel;
  }

  // Parses this:
  // {
  //   "joints": {
  //     "joint_1": { "index": 0, "position_command": 0.4, "vmax": 0.2, "tmc": 0.1 },
  //     "joint_2": { "index": 0, "position_command": 0.4, "vmax": 0.2, "tmc": 0.1 }
  //   },
  //   "quit": true
  // }
  // All entries are optional
  bool setCommand(const std::string& message)
  {
    std::vector<double> maxVel = getMaxVel();
    nlohmann::json data;
    bool quitMe = false;

    // Parse with exception safety
    try
    {
      data = nlohmann::json::parse(message);
    }
    catch (const nlohmann::json::parse_error& e)
    {
      RLOG_CPP(1, "JSON parse error: " << e.what());
      return quitMe;
    }

    // Parse into temporary variable to keep concurrent access short. We do all
    // the checking and validation here so that there is no overhead in the
    // control loop.
    std::map<std::string, JointCommand> joint_map_tmp;
    bool success = true;


    // Validate "joints"
    if (data.contains("joints") && data["joints"].is_object())
    {
      // Iterate joints
      for (auto it = data["joints"].begin(); it != data["joints"].end(); ++it)
      {
        const std::string& joint_name = it.key();
        const nlohmann::json& joint_data = it.value();

        if (!joint_data.is_object())
        {
          RLOG_CPP(1, "Joint `" << joint_name << "` is not an object");
          success = false;
          continue;
        }

        JointCommand cmd{};

        // Ensure required field "index"
        if (joint_data.contains("index") &&
            joint_data["index"].is_number() &&
            joint_data["index"] < getDOF())
        {
          cmd.index = joint_data["index"].get<int>();
        }
        else
        {
          RLOG_CPP(1, "Joint `" << joint_name << "` has no or wrong index: " << data.dump(2));
          success = false;
          continue;
        }

        // Optional position_command
        if (joint_data.contains("position_command") &&
            joint_data["position_command"].is_number())
        {
          cmd.position_command = joint_data["position_command"].get<double>();
          cmd.has_position_command = true;
        }

        // Optional vmax
        if (joint_data.contains("vmax") && joint_data["vmax"].is_number())
        {
          cmd.vmax = joint_data["vmax"].get<double>();
          cmd.has_vmax = true;

          if (cmd.vmax > maxVel[cmd.index])
          {
            RLOG_CPP(1, "Joint `" << joint_name << "` exceeds vmax: " << data.dump(2));
            success = false;
          }
        }

        // Optional tmc
        if (joint_data.contains("tmc") && joint_data["tmc"].is_number())
        {
          cmd.tmc = joint_data["tmc"].get<double>();
          cmd.has_tmc = true;

          if (cmd.tmc < getMinTMC())
          {
            RLOG_CPP(1, "Joint `" << joint_name << "` has too low tmc: " << data.dump(2));
            success = false;
          }
        }

        joint_map_tmp.emplace(joint_name, cmd);
      }
    }

    // Parse quitMe
    quitMe = data.value("quit", false);

    // Parsing finished - perform concurrent swap here
    if (success)
    {
      std::lock_guard<std::mutex> lock(cmdMtx);
      this->incomingCommand.jointCommands.swap(joint_map_tmp);
      this->incomingCommand.newCommand = true;
      this->incomingCommand.quitMe = quitMe;
    }
    else
    {
      RLOG_CPP(0, "Error creading commands: " << data.dump(2));
    }

    return quitMe;
  }


protected:

  std::function<void(const std::string&)> feedbackFcn;
};





static int runFranka(const std::string& robo_name)
{
  const aff::RoboNetworkInfo* nwInfo = aff::RoboNetworkInfo::getNetworkInfo(robo_name);

  if (!nwInfo)
  {
    RLOG_CPP(0, "Robo name not known: " << robo_name);
    return -1;
  }

  // Thread sending sensory data to remote process
  FeedbackThread feedback;
  feedback.start(nwInfo->roboSender, runLoop);

  // Robo driver thread. The FeedbackThread's updateMessage function is called
  // in each control cycle once registered.
  FrankaDriver robo;
  auto fbFcn = std::bind(&FeedbackThread::updateMessage, &feedback, std::placeholders::_1);
  robo.registerFeedbackCallback(fbFcn);
  //robo.setDummyMode(dummy_mode);
  robo.start();//runLoop, readOnly);

  // Command receiver. On each arriving command, the PTUDriver's setCommand
  // functionis called.
  bool blocking = true;
  CommandThread commands;
  auto cmdFcn = std::bind(&FrankaDriver::setCommand, &robo, std::placeholders::_1);
  commands.start(nwInfo->roboReceiver, cmdFcn, runLoop, blocking);

  commands.stop();
  robo.stop();
  feedback.stop();

  return 0;
}

/*******************************************************************************
 *
 *******************************************************************************/
void quit(int /*sig*/)
{
  static int kHit = 0;
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit+1);
  kHit++;
  runLoop = false;

  if (kHit > 1)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}

/*******************************************************************************
 *
 *******************************************************************************/
int main(int argc, char** argv)
{
  signal(SIGINT, quit);   // Ctrl-C stops threads

  int mode = 0;
  std::string robo_name;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Mode (default is %d)", mode);
  argP.getArgument("-robo_name", &robo_name, "Robot specifier (default is %s)", robo_name.c_str());

  switch (mode)
  {
    case 0:
      printf("\nHere's what you can do:\n\n");
      printf("\t-m 0   Prints this message (default)\n");
      printf("\t-m 1   Run libfranka server\n");
      printf("\n");
      argP.print();
      break;

    case 1:
      runFranka(robo_name);
      break;

    default:
      RLOG_CPP(0, "No mode " << mode);
  };

  RLOG_CPP(0, "Thanks, that's it for mode " << mode);





  return 0;
}
