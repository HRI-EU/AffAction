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

#include "RoboDriverNetworking.hpp"
#include "RoboNetworkInfo.hpp"
#include "PW70CANInterface.h"
#include "json.hpp"

#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include <Rcs_filters.h>
#include <Rcs_timer.h>

#include <zmq.hpp>
#include <csignal>


static std::atomic<bool> runLoop(true);

#define DOF_PTU (2)

// Angular limits (conservative)
#define PAN_MIN_RAD           (-80.0*(M_PI/180.0))
#define PAN_MAX_RAD           (80.0*(M_PI/180.0))
#define TILT_MIN_RAD          (-40.0*(M_PI/180.0))
#define TILT_MAX_RAD          (40.0*(M_PI/180.0))

// Hardware limits
#define PAN_MIN_RAD_INTERNAL  (-180.0*(M_PI/180.0))
#define PAN_MAX_RAD_INTERNAL  (180.0*(M_PI/180.0))
#define TILT_MIN_RAD_INTERNAL (-50.0*(M_PI/180.0))
#define TILT_MAX_RAD_INTERNAL (50.0*(M_PI/180.0))

// Velocity limits
#define PAN_VELOCITY_MIN_RAD  (1.0*(M_PI/180.0))
#define PAN_VELOCITY_MAX_RAD  (70.0*(M_PI/180.0))
#define TILT_VELOCITY_MIN_RAD (1.0*(M_PI/180.0))
#define TILT_VELOCITY_MAX_RAD (40.0*(M_PI/180.0))

#define PANTILT_FILT_MIN_TMC  (0.05)

static double pan_tilt_max_vel[2] = { PAN_VELOCITY_MAX_RAD, TILT_VELOCITY_MAX_RAD };

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
 * Command json:
 *
 * {
 *   "joints": {
 *     "joint_1": { "position_command": 0.4, "vmax": 0.2, "tmc": 0.1 },
 *     "joint_2": { "position_command": 0.4, "vmax": 0.2, "tmc": 0.1 }
 *   },
 *   "quit": true
 * }
 *
 *******************************************************************************/
class PTUDriver
{
public:

  PTUDriver(int update_frequency=50) :
    current_pan_position(0.0),
    current_tilt_position(0.0),
    current_pan_velocity(0.0),
    current_tilt_velocity(0.0),
    current_time_stamp(0.0),
    filterInitialized(false),
    dummy_mode(false),
    panTiltFilt(0.1, 0.0, 0.02, 2)
  {
    const double dt = 1.0/ update_frequency;   // 50Hz loop
    RCHECK(dt >= 0.01);
    const double tmc = 0.1;
    panTiltFilt.setDt(dt);
    panTiltFilt.setMaxVel(PAN_VELOCITY_MAX_RAD, 0);
    panTiltFilt.setMaxVel(TILT_VELOCITY_MAX_RAD, 1);
    panTiltFilt.setTimeConstant(tmc, 0);
    panTiltFilt.setTimeConstant(tmc, 1);
  }

  static void limit_check(double pan, double tilt, void* param)
  {
  }

  // Angles come in in radians, timestamp is seconds since epoch (system clock)
  static void position_update(double pan_angle, double tilt_angle, double timestamp, void* param)
  {
    PTUDriver* self = static_cast<PTUDriver*>(param);

    // Determine time delta for velocity calculation
    const double dt = timestamp - self->current_time_stamp;

    if (dt <= 0.0)
    {
      RLOG_CPP(0, "dt <= 0: " << dt << " - skipping step");
      return;
    }

    self->current_time_stamp = timestamp;
    self->current_pan_velocity = (pan_angle - self->current_pan_position) / dt;
    self->current_tilt_velocity = (tilt_angle - self->current_tilt_position) / dt;
    self->current_pan_position = pan_angle;
    self->current_tilt_position = tilt_angle;

    if (!self->filterInitialized)
    {
      double q_init[2] = { pan_angle, tilt_angle };
      self->panTiltFilt.init(q_init);
      self->filterInitialized = true;
    }

    // Here comes the command
    RoboCommand copyOfCmd;
    {
      std::lock_guard<std::mutex> lock(self->cmdMtx);
      if (self->cmd.newCommand)
      {
        copyOfCmd.jointCommands.swap(self->cmd.jointCommands);
        copyOfCmd.newCommand = true;
        copyOfCmd.quitMe = self->cmd.quitMe;
        self->cmd.newCommand = false;
      }
    }

    if (copyOfCmd.newCommand)
    {
      for (const auto& pair : copyOfCmd.jointCommands)
      {
        const std::string& joint_name = pair.first;
        const JointCommand& cmd = pair.second;

        if (cmd.has_position_command)
        {
          self->panTiltFilt.setTarget(cmd.position_command, cmd.index);
        }

        if (cmd.has_vmax)
        {
          self->panTiltFilt.setMaxVel(cmd.vmax, cmd.index);
        }

        if (cmd.has_tmc)
        {
          self->panTiltFilt.setTimeConstant(cmd.tmc, cmd.index);
        }
      }

    }

    double filtPos[2], filtVel[2];
    self->panTiltFilt.iterate();
    self->panTiltFilt.getPosition(filtPos);
    self->panTiltFilt.getVelocity(filtVel);

    double desired_pan_position = filtPos[0];
    double desired_tilt_position = filtPos[1];
    double desired_pan_velocity = filtVel[0];
    double desired_tilt_velocity = filtVel[1];

    if (self->feedbackFcn)
    {
      nlohmann::json fbJson;
      fbJson["time"] = Timer_getSystemTime();
      fbJson["position"] = std::vector<double> { self->current_pan_position, self->current_tilt_position };
      fbJson["velocity"] = std::vector<double> { self->current_pan_velocity, self->current_tilt_velocity };
      fbJson["pan_des"] = desired_pan_position;
      fbJson["tilt_des"] = desired_tilt_position;
      fbJson["pan_vel_des"] = desired_pan_velocity;
      fbJson["tilt_vel_des"] = desired_tilt_velocity;

      self->feedbackFcn(fbJson.dump());
    }

    // Here is the velocity control loop
    if (!self->pw70)
    {
      RLOG(0, "pw70 not initialized - quitting control thread");
      return;
    }

    // Proportional controller gain (adjust as necessary)
    const double Kp = 2.0; // Proportional gain for position error correction

    // Apply proportional control to compute the velocity correction
    double corrected_pan_velocity, corrected_tilt_velocity;
    {
      //std::lock_guard<std::mutex> lock(panTiltUpdateMtx);
      double pan_velocity_correction = Kp * (desired_pan_position - self->current_pan_position);
      corrected_pan_velocity = desired_pan_velocity + pan_velocity_correction;
      corrected_pan_velocity = Math_clip(corrected_pan_velocity, -PAN_VELOCITY_MAX_RAD, PAN_VELOCITY_MAX_RAD);

      double tilt_velocity_correction = Kp * (desired_tilt_position - self->current_tilt_position);
      corrected_tilt_velocity = desired_tilt_velocity + tilt_velocity_correction;
      corrected_tilt_velocity = Math_clip(corrected_tilt_velocity, -TILT_VELOCITY_MAX_RAD, TILT_VELOCITY_MAX_RAD);
    }

    bool success = self->pw70->move_velocity(corrected_pan_velocity, corrected_tilt_velocity);

    RLOG(5, "%s sending velocities[deg]: %.3f %.3f   errors: %.3f %.3f",
         success ? "SUCCESS" : "FAILURE",
         RCS_RAD2DEG(corrected_pan_velocity),
         RCS_RAD2DEG(corrected_tilt_velocity),
         RCS_RAD2DEG(desired_pan_position - self->current_pan_position),
         RCS_RAD2DEG(desired_tilt_position - self->current_tilt_position));
  }

  void start(const std::atomic_bool& run_flag, bool readOnly)
  {
    if (this->pw70)
    {
      RLOG(1, "PW70 already running - skipping start");
      return;
    }

    // Create an instance of PW70CANInterface with the callbacks
    const int frequency = 50;   // 1, 10, 25, 50 or 100
    this->pw70 = aff::PW70CANInterface::create(limit_check, position_update, this, frequency, this->dummy_mode);
    this->pw70->reset_stop();

    // Wait a moment to allow the interface to initialize
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  void stop()
  {
    if (!this->pw70)
    {
      RLOG(1, "PW70 already stopped - skipping stop");
      return;
    }

    pw70.reset();
    this->filterInitialized = false;
    this->feedbackFcn = nullptr;
    RLOG(0, "pw70.reset()");
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
            joint_data["index"] < DOF_PTU)
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

          if (cmd.vmax > pan_tilt_max_vel[cmd.index])
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

          if (cmd.tmc < PANTILT_FILT_MIN_TMC)
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
      cmd.jointCommands.swap(joint_map_tmp);
      cmd.newCommand = true;
      cmd.quitMe = quitMe;
    }
    else
    {
      RLOG_CPP(0, "Error creading commands: " << data.dump(2));
    }

    return quitMe;
  }

  void registerFeedbackCallback(std::function<void(const std::string&)> cb)
  {
    feedbackFcn = std::move(cb);
  }

  void setDummyMode(bool enable)
  {
    this->dummy_mode = enable;
  }

private:


  std::unique_ptr<aff::PW70CANInterface> pw70;
  std::function<void(const std::string&)> feedbackFcn;

  double current_pan_position, current_tilt_position;
  double current_pan_velocity, current_tilt_velocity;
  double current_time_stamp;
  bool filterInitialized;
  bool dummy_mode;
  Rcs::RampFilterND panTiltFilt;

  // Command data struct
  struct JointCommand
  {
    int index;
    double position_command;
    double vmax;
    double tmc;
    bool has_position_command;
    bool has_vmax;
    bool has_tmc;
  };

  struct RoboCommand
  {
    std::map<std::string, JointCommand> jointCommands;
    bool quitMe;
    bool newCommand;
  };

  std::mutex cmdMtx;
  RoboCommand cmd;
};


/*******************************************************************************
 *
 *******************************************************************************/
static void runPTU(int argc, char** argv)
{
  std::string robo_name = "ptu_test";
  Rcs::CmdLineParser argP(argc, argv);
  bool readOnly = argP.hasArgument("-ro", "Read-only, no motor commands");
  argP.getArgument("-robo_name", &robo_name, "Name of PTU to use (default is '%s')", robo_name.c_str());

  if (argP.hasArgument("-h"))
  {
    argP.print();
    return;
  }

  aff::RoboNetworkInfo nwInfo = aff::RoboNetworkInfo::getNetworkInfo(robo_name);
  bool dummy_mode = (nwInfo.roboMode == "TestWithoutRobot") ? true : false;

  // Thread sending sensory data to remote process
  FeedbackThread feedback;
  feedback.start(nwInfo.roboSender, runLoop);

  // Robo driver thread. The FeedbackThread's updateMessage function is called
  // in each control cycle once registered.
  PTUDriver robo;
  auto fbFcn = std::bind(&FeedbackThread::updateMessage, &feedback, std::placeholders::_1);
  robo.registerFeedbackCallback(fbFcn);
  robo.setDummyMode(dummy_mode);
  robo.start(runLoop, readOnly);

  // Command receiver. On each arriving command, the PTUDriver's setCommand
  // functionis called.
  bool blocking = true;
  CommandThread commands;
  auto cmdFcn = std::bind(&PTUDriver::setCommand, &robo, std::placeholders::_1);
  commands.start(nwInfo.roboReceiver, cmdFcn, runLoop, blocking);

  commands.stop();
  robo.stop();
  feedback.stop();
}

/*******************************************************************************
 *
 *******************************************************************************/
static void initializePan()
{
  auto pw70 = aff::PW70CANInterface::create();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  pw70->reference_pan();
  std::this_thread::sleep_for(std::chrono::seconds(5));
}

/*******************************************************************************
 *
 *******************************************************************************/
static void initializeTilt()
{
  auto pw70 = aff::PW70CANInterface::create();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  pw70->reference_tilt();
  std::this_thread::sleep_for(std::chrono::seconds(5));
}

/*******************************************************************************
 *
 *******************************************************************************/
static void movePanTilt(int argc, char** argv)
{
  Rcs::CmdLineParser argP(argc, argv);

  double pan_in_deg = 0.0, tilt_in_deg = 0.0, pan_vel_in_deg = 10.0, tilt_vel_in_deg = 10.0;
  argP.getArgument("-pan_in_deg", &pan_in_deg, "Pan angle in degrees (default is %f)", pan_in_deg);
  argP.getArgument("-tilt_in_deg", &tilt_in_deg, "Tilt angle in degrees (default is %f)", tilt_in_deg);
  argP.getArgument("-pan_vel_in_deg", &pan_vel_in_deg, "Pan angle in degrees (default is %f)", pan_vel_in_deg);
  argP.getArgument("-tilt_vel_in_deg", &tilt_vel_in_deg, "Tilt angle in degrees (default is %f)", tilt_vel_in_deg);

  auto pw70 = aff::PW70CANInterface::create();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  pw70->move_position(RCS_DEG2RAD(pan_in_deg),
                      RCS_DEG2RAD(tilt_in_deg),
                      RCS_DEG2RAD(pan_vel_in_deg),
                      RCS_DEG2RAD(tilt_vel_in_deg));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  pw70->stop();
  pw70->cleanup();
}

/*******************************************************************************
 *
 *******************************************************************************/
int main(int argc, char** argv)
{
  signal(SIGINT, quit);   // Ctrl-C stops threads

  int mode = 0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Mode (default is %d)", mode);

  switch (mode)
  {
    case 0:
      printf("\nHere's what you can do:\n\n");
      printf("\t-m 0   Prints this message (default)\n");
      printf("\t-m 1   Run PTU server (velocity loop)\n");
      printf("\t-m 2   Initialize pan motor\n");
      printf("\t-m 3   Initialize tilt motor\n");
      printf("\t-m 4   Move to pan and tilt position (in degrees)\n");
      printf("\n");
      argP.print();
      break;

    case 1:
      runPTU(argc, argv);
      break;

    case 2:
      initializePan();
      break;

    case 3:
      initializeTilt();
      break;

    case 4:
      movePanTilt(argc, argv);
      break;


    default:
      RLOG_CPP(0, "No mode " << mode);
  };

  RLOG_CPP(0, "Thanks, that's it for mode " << mode);





  return 0;
}
