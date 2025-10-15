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
#include <Rcs_macros.h>

#include <zmq.hpp>

#if defined (AFFACTION_WITH_LIBFRANKA)
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#endif

#include <array>
#include <mutex>
#include <thread>
#include <iostream>
#include <chrono>
#include <atomic>
#include <csignal>


#define DOF_ARM (7)

static std::atomic<bool> runLoop(true);


class FrankaDriver : public RoboDriver
{
public:

  void registerFeedbackCallback(std::function<void(const std::string&)> cb)
  {
    feedbackFcn = std::move(cb);
  }

  void start(std::string robo_ip, const std::atomic_bool& run_flag, bool inSimulation)
  {
    if (roboThread.joinable())
    {
      RLOG(1, "Robo thread start() called while thread is already running.");
      return;
    }

    if (inSimulation)
    {
      roboThread = std::thread(&FrankaDriver::sim_loop, this, std::cref(run_flag));
    }
    else
    {
#if defined (AFFACTION_WITH_LIBFRANKA)
      roboThread = std::thread(&FrankaDriver::joint_compliance, this, robo_ip, std::cref(run_flag));
#else
      RLOG_CPP(0, "libFranka not compiled in - can't start robot thread.");
#endif


    }

  }

  void stop()
  {
    if (roboThread.joinable())
    {
      RLOG(0, "Waiting for robo thread to join");
      roboThread.join();
      RLOG(0, "Robo thread joined");
    }
    else
    {
      RLOG(0, "Robo thread already stopped");
    }

  }

  int sim_loop(const std::atomic<bool>& run_flag)
  {
    size_t loopCount = 0;

    // Create and initialize filters
    const double tmc = 0.1;
    const double dt = 0.001;
    Rcs::RampFilterND filteredJointCommands(tmc, 0.0, dt, DOF_ARM);

    //filteredJointCommands = std::make_unique<Rcs::RampFilterND>(tmc, 0.0, dt, getDOF());
    std::vector<double> q_init{0.0, 0.0, 0.0, -M_PI_2, 0.0, M_PI_2, 0.0};
    filteredJointCommands.init(q_init.data());
    for (size_t i = 0; i < filteredJointCommands.getDim(); ++i)
    {
      filteredJointCommands.setMaxVel(RoboDriver::getMaxVel(i), i);
    }
    RLOG(0, "Filters initialized");


    while (run_flag.load(std::memory_order_relaxed))
    {
      // Send feedback back to remote process every 25th frame (=40Hz)
      if (this->feedbackFcn && (loopCount % 25 == 0))
      {
        nlohmann::json fbJson;
        fbJson["time"] = Timer_getSystemTime();
        fbJson["cycle_time_usec"] = dt;
        fbJson["position"] = filteredJointCommands.getPosition();
        fbJson["velocity"] = filteredJointCommands.getVelocity();
        fbJson["torque"] = std::vector<double>(7, 0.0);
        this->feedbackFcn(fbJson.dump());
      }

      // Process new incoming commands
      bool receivedNewCommand = false;
      RobotCommand copyOfCmd;
      {
        std::lock_guard<std::mutex> lock(cmdMtx);
        copyOfCmd = this->incomingCommand;
        receivedNewCommand = this->newIncomingCommand;
        this->newIncomingCommand = false;
      }

      if (receivedNewCommand)
      {
        applyCommandToFilters(copyOfCmd, filteredJointCommands);
      }

      // Interpolation at every time step
      filteredJointCommands.iterate();

      // Compute desired EE pose from q_des
      std::array<double, 7> q_des{};
      for (size_t i = 0; i < q_des.size(); ++i)
      {
        q_des[i] = filteredJointCommands.getPosition(i);
      }

      loopCount++;

      Timer_waitDT(dt);
    }

    return 0;
  }

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
      return false;
    }

    RoboDriver::RobotCommand rcmd;
    bool valid_cmd = parse_robot_command(data, rcmd);

    if (!valid_cmd)
    {
      RLOG_CPP(1, "Malformed robot command: " << message);
      return false;
    }

    valid_cmd = check_robot_command(rcmd);

    if (valid_cmd)
    {
      std::lock_guard<std::mutex> lock(cmdMtx);
      this->incomingCommand = rcmd;
      this->newIncomingCommand = true;
      quitMe = this->incomingCommand.quit;
    }
    else
    {
      RLOG_CPP(1, "Invalid robot command: " << message);
      return false;
    }

    return quitMe;
  }

  size_t getDOF() const
  {
    return DOF_ARM;
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

  // FR3 joint ranges:
  //    1: -166/166 deg
  //    2: -105/105 deg
  //    3: -166/166 deg
  //    4: -176/-7 deg
  //    5: -165/165 deg
  //    6:  25/265 deg
  //    7: -175/175 deg
  std::vector<double> getLowerJointLimits() const
  {
    std::vector<double> ll(DOF_ARM, 0.0);
    ll[0] = RCS_DEG2RAD(-166.0);
    ll[1] = RCS_DEG2RAD(-105.0);
    ll[2] = RCS_DEG2RAD(-166.0);
    ll[3] = RCS_DEG2RAD(-176.0);
    ll[4] = RCS_DEG2RAD(-165.0);
    ll[5] = RCS_DEG2RAD(25.0);
    ll[6] = RCS_DEG2RAD(-175.0);

    return ll;
  }

  std::vector<double> getUpperJointLimits() const
  {
    std::vector<double> ul(DOF_ARM, 0.0);
    ul[0] = RCS_DEG2RAD(166.0);
    ul[1] = RCS_DEG2RAD(105.0);
    ul[2] = RCS_DEG2RAD(166.0);
    ul[3] = RCS_DEG2RAD(-7.0);
    ul[4] = RCS_DEG2RAD(165.0);
    ul[5] = RCS_DEG2RAD(265.0);
    ul[6] = RCS_DEG2RAD(175.0);

    return ul;
  }

  bool check_robot_command(RobotCommand& robo_cmd) const
  {
    bool success = true;

    std::vector<double> ll = getLowerJointLimits();
    std::vector<double> ul = getUpperJointLimits();

    for (const auto& cmd : robo_cmd.actuators)
    {
      if ((cmd.type != "joint") || (cmd.index<0) || (cmd.index>=DOF_ARM))
      {
        continue;
      }

      if (cmd.has_position)
      {
        if ((cmd.position<ll[cmd.index]) || (cmd.position>ul[cmd.index]))
        {
          success = false;
        }
      }

      if (cmd.has_vmax && (cmd.vmax > RoboDriver::getMaxVel(cmd.index)))
      {
        success = false;
      }

      if (cmd.has_tmc && (cmd.tmc < getMinTMC()))
      {
        return false;
      }

    }

    return success;
  }

  void applyCommandToFilters(const RobotCommand& robo_cmd, Rcs::RampFilterND& filt) const
  {
    for (const auto& cmd : robo_cmd.actuators)
    {
      if ((cmd.type != "joint") || (cmd.index<0) || (cmd.index>=DOF_ARM))
      {
        continue;
      }

      if (cmd.has_position)
      {
        filt.setTarget(cmd.position, cmd.index);
      }

      if (cmd.has_vmax)
      {
        filt.setMaxVel(cmd.vmax, cmd.index);
      }

      if (cmd.has_tmc)
      {
        filt.setTimeConstant(cmd.tmc, cmd.index);
      }
    }

  }











#if defined (AFFACTION_WITH_LIBFRANKA)

  static void setDefaultBehavior(franka::Robot& robot)
  {
    robot.setCollisionBehavior({ {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0} },  // joint lower
    { {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0} },  // joint upper
    { {100.0, 100.0, 100.0, 100.0, 100.0, 100.0} },         // cartesian lower
    { {100.0, 100.0, 100.0, 100.0, 100.0, 100.0} });        // cartesian upper

    // (Optional) Set correct tool mass/COM if you have a tool; zeros if not:
    // robot.setLoad(0.0, {0,0,0}, {0,0,0, 0,0,0, 0,0,0});
  }

  int joint_hold_compliant_simple(std::string robo_ip)
  {
    try
    {
      franka::Robot robot(robo_ip);
      FrankaDriver::setDefaultBehavior(robot);

      // Joint compliance (Nm/rad). Lower = softer; raise for firmer hold.
      // robot.setJointImpedance({120, 120, 120, 80, 60, 40, 30});
      // robot.setJointImpedance({60, 60, 50, 30, 20, 12, 8});   // Ultra-soft
      // robot.setJointImpedance({90, 90, 80, 45, 35, 20, 15});  // Soft
      // robot.setJointImpedance({140, 140, 120, 70, 55, 35, 25});  // Medium

      robot.setJointImpedance({140, 140, 120, 30, 20, 12, 8});   // Ultra-soft

      std::array<double,7> q_hold{};
      bool initialized = false;

      robot.control(
        [&](const franka::RobotState& rs, franka::Duration) -> franka::JointPositions
      {
        if (!initialized)
        {
          q_hold = rs.q;              // latch initial joints once
          initialized = true;
        }
        return franka::JointPositions(q_hold);  // keep commanding the latched pose
      },
      franka::ControllerMode::kJointImpedance,  // use joint impedance controller
      /*limit_rate=*/true                        // smooth & safe
      );

    }
    catch (const franka::Exception& e)
    {
      std::cerr << "Franka Exception: " << e.what() << std::endl;
      return -1;
    }
    return 0;
  }



  int follow_the_hand(std::string robo_ip)
  {
    try
    {
      franka::Robot robot("192.168.42.11");
      FrankaDriver::setDefaultBehavior(robot);

      // Soft, comfy stiffness (tune to taste). Increase values to make it firmer.
      robot.setCartesianImpedance({200, 200, 200, 15, 15, 15});

      std::cout << "[INFO] Follow-the-hand mode: move the arm gently; Ctrl-C to stop.\n";

      // Control loop: equilibrium pose = measured pose (follows the hand)
      robot.control(
        [](const franka::RobotState& rs, franka::Duration) -> franka::CartesianPose
      {
        return franka::CartesianPose(rs.O_T_EE);
      },
      franka::ControllerMode::kCartesianImpedance,  // select Cartesian impedance controller
      /*limit_rate=*/true                           // avoid extra rate limiting for a "light" feel
      );
    }
    catch (const franka::Exception& e)
    {
      std::cerr << "Franka Exception: " << e.what() << std::endl;
      return -1;
    }
    return 0;
  }

  // Seems like setting cartesian compliance is not working currently:
  // https://github.com/frankarobotics/libfranka/issues/180
  int cartesian_compliance(std::string robo_ip)
  {
    RLOG(0, "START called");

    try
    {
      double time = 0.0;
      size_t loopCount = 0;
      franka::Robot robot(robo_ip);
      franka::Model model = robot.loadModel();
      FrankaDriver::setDefaultBehavior(robot);

      robot.setCartesianImpedance({50, 50, 50, 10, 10, 10});
      //robot.setCartesianImpedance({600, 600, 600, 15, 15, 15});
      RLOG_CPP(0, "[INFO] Cartesian impedance set.");

      std::unique_ptr<Rcs::RampFilterND> filteredJointCommands;

      auto cartesian_pose_cb = [&](const franka::RobotState& rs, franka::Duration period) -> franka::CartesianPose
      {
        // Send feedback back to remote process
        if (this->feedbackFcn && (loopCount%25==0))
        {
          this->feedbackFcn(feedback2JsonString(rs, 0, nullptr));
        }

        // Dration is 0 at the first incocation of the callback
        if (time == 0.0 && !filteredJointCommands)
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
          RLOG(0, "Robot initialized");
        }

        time += period.toSec();

        loopCount++;

        if (loopCount%500==0)
        {
          RLOG(0, "tic");
        }

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
      };




      robot.control(cartesian_pose_cb, franka::ControllerMode::kCartesianImpedance,
                    /*limit_rate=*/false);
    }
    catch (const franka::Exception& e)
    {
      std::cerr << "Franka Exception: " << e.what() << std::endl;
      return -1;
    }

    return 0;
  }



  // Joint-space compliance
  int joint_compliance(std::string robo_ip, const std::atomic<bool>& run_flag)
  {

    try
    {
      double time = 0.0;
      size_t loopCount = 0;
      franka::Robot robot(robo_ip);
      FrankaDriver::setDefaultBehavior(robot);

      // Joint compliance (Nm/rad). Lower = softer; raise for firmer hold.
      // robot.setJointImpedance({120, 120, 120, 80, 60, 40, 30});
      // robot.setJointImpedance({60, 60, 50, 30, 20, 12, 8});   // Ultra-soft
      // robot.setJointImpedance({90, 90, 80, 45, 35, 20, 15});  // Soft
      robot.setJointImpedance({140, 140, 120, 70, 55, 35, 25});  // Medium
      RLOG_CPP(0, "[INFO] Joint impedance set.");

      const double tmc = 0.1;
      const double dt = 0.001;
      Rcs::RampFilterND filteredJointCommands(tmc, 0.0, dt, DOF_ARM);
      for (size_t i = 0; i < filteredJointCommands.getDim(); ++i)
      {
        filteredJointCommands.setMaxVel(RoboDriver::getMaxVel(i), i);
      }

      auto joint_pose_cb = [&](const franka::RobotState& rs, franka::Duration period) -> franka::JointPositions
      {
        // Send feedback back to remote process every 25th frame (=40Hz)
        if (this->feedbackFcn && (loopCount%25==0))
        {
          this->feedbackFcn(feedback2JsonString(rs, 0, nullptr));
        }

        // Initialize filters with current robot sensor values on first cycle
        if (loopCount == 0)
        {
          filteredJointCommands.init(rs.q.data());
          RLOG(0, "Filters initialized");
        }


        // Process new incoming commands
        bool receivedNewCommand = false;
        RobotCommand copyOfCmd;
        {
          std::lock_guard<std::mutex> lock(cmdMtx);
          copyOfCmd = this->incomingCommand;
          receivedNewCommand = this->newIncomingCommand;
          this->newIncomingCommand = false;
        }

        if (receivedNewCommand)
        {
          applyCommandToFilters(copyOfCmd, filteredJointCommands);
        }

        // Interpolation at every time step
        filteredJointCommands.iterate();

        // Compute desired EE pose from q_des
        std::array<double,7> q_des{};
        for (size_t i=0; i<q_des.size(); ++i)
        {
          q_des[i] = filteredJointCommands.getPosition(i);
        }

        time += period.toSec();
        loopCount++;

        if (loopCount%500==0)
        {
          RLOG(0, "tic");
        }

        // Exit request from outside?
        if (!run_flag.load(std::memory_order_relaxed))
        {
          return franka::MotionFinished(franka::JointPositions(q_des));
        }

        return franka::JointPositions(q_des);
      };




      robot.control(joint_pose_cb, franka::ControllerMode::kJointImpedance, /*limit_rate=*/true);
    }
    catch (const franka::Exception& e)
    {
      std::cerr << "Franka Exception: " << e.what() << std::endl;
      return -1;
    }

    return 0;
  }



  std::string feedback2JsonString(const franka::RobotState& rs,
                                  int64_t time_usec, const RobotCommand* cmd)
  {
    nlohmann::json fbJson;
    fbJson["time"] = Timer_getSystemTime();
    fbJson["cycle_time_usec"] = time_usec;
    fbJson["position"] = rs.q;
    fbJson["velocity"] = rs.dq;
    fbJson["torque"] = rs.tau_J;

    if (cmd)
    {
      std::vector<double> joint_err(DOF_ARM, 0.0);
      std::vector<double> joint_cmd(DOF_ARM, 0.0);

      for (size_t i=0; i<cmd->actuators.size(); ++i)
      {
        const ActuatorCommand& a = cmd->actuators[i];

        if ((a.type != "joint") || (a.index<0) || (a.index>=DOF_ARM))
        {
          continue;
        }

        joint_cmd[i] = RCS_RAD2DEG(RCS_DEG2RAD(a.position));
        joint_err[i] = RCS_RAD2DEG(RCS_DEG2RAD(a.position) - rs.q[a.index]);
        fbJson["position_error"] = joint_err;
        fbJson["position_command"] = joint_cmd;
      }
    }

    return fbJson.dump();
  }

#endif   // AFFACTION_WITH_LIBFRANKA

protected:

  std::function<void(const std::string&)> feedbackFcn;
  std::thread roboThread;
};

/*******************************************************************************
 *
 *******************************************************************************/
static void quit(int /*sig*/)
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
  std::string robo_name = "riemann";
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Mode (default is %d)", mode);
  argP.getArgument("-robo_name", &robo_name, "Robot specifier (default is %s)", robo_name.c_str());
  bool sim = argP.hasArgument("-sim", "Test in simulation only");

  const aff::RoboNetworkInfo* nwInfo = aff::RoboNetworkInfo::getNetworkInfo(robo_name);

  if (!nwInfo)
  {
    RLOG_CPP(0, "Robo name not known: " << robo_name);
    return -1;
  }

  switch (mode)
  {
    case 0:
      printf("\nHere's what you can do:\n\n");
      printf("\t-m 0   Prints this message (default)\n");
      printf("\t-m 1   Run joint compliance (networked)\n");
      printf("\t-m 2   Run joint compliance demo (no networking)\n");
      printf("\t-m 3   Run follow-the-hand demo (no networking)\n");
      printf("\n");
      argP.print();
      break;

    case 1:
    {
      // Thread sending sensory data to remote process. This runs a networking thread that is
      // woken up by a condition variable that is set from the robo thread.
      FeedbackThread feedback;
      feedback.start(nwInfo->roboSender, runLoop);

      // Robo driver thread. The FeedbackThread's updateMessage function is called
      // in each control cycle once registered.
      FrankaDriver robo;
      auto fbFcn = std::bind(&FeedbackThread::updateMessage, &feedback, std::placeholders::_1);
      robo.registerFeedbackCallback(fbFcn);
      robo.start(nwInfo->robo_ip, runLoop, sim);

      // Command receiver. On each arriving command, the driver's setCommand function is called.
      bool blocking = true;
      CommandThread commands;
      auto cmdFcn = std::bind(&FrankaDriver::setCommand, &robo, std::placeholders::_1);
      commands.start(nwInfo->roboReceiver, cmdFcn, runLoop, blocking);

      commands.stop();
      robo.stop();
      feedback.stop();
    }
    break;

#if defined (AFFACTION_WITH_LIBFRANKA)
    case 2:
    {
      FrankaDriver robo;
      robo.joint_hold_compliant_simple(nwInfo->robo_ip);
      RPAUSE();
      robo.stop();
    }
    break;

    case 3:
    {
      FrankaDriver robo;
      robo.follow_the_hand(nwInfo->robo_ip);
      RPAUSE();
      robo.stop();
    }
    break;
#endif   // AFFACTION_WITH_LIBFRANKA

    default:
      RLOG_CPP(0, "No mode " << mode);
  };

  RLOG_CPP(0, "Thanks, that's it for mode " << mode);





  return 0;
}
