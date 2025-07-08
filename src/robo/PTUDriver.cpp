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

#if defined (_MSC_VER) && defined (AFFACTION_WITH_PCAN_BASIC)
#include "PW70CANInterfaceWin.hpp"
#elif defined(__linux__) && !defined(__APPLE__)
#include "PW70CANInterfaceLinux.hpp"
#else
#include "PW70CANInterfaceDummy.hpp"
#endif

#include "json.hpp"

#include <Rcs_cmdLine.h>
#include <Rcs_math.h>
#include "Rcs_filters.h"
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
class PTUDriver
{
public:

  PTUDriver() :
    current_pan_position(0.0),
    current_tilt_position(0.0),
    current_pan_velocity(0.0),
    current_tilt_velocity(0.0),
    current_time_stamp(0.0),
    filterInitialized(false),
    panTiltFilt(0.1, 0.0, 0.02, 2)
  {
    const double dt = 0.02;   // 50Hz loop
    const double tmc = 0.1;
    panTiltFilt.setMaxVel(PAN_VELOCITY_MAX_RAD, 0);
    panTiltFilt.setMaxVel(TILT_VELOCITY_MAX_RAD, 1);
    panTiltFilt.setDt(dt);
  }

  static void limit_check(double pan, double tilt, void* param)
  {
    RLOG_CPP(2, "Limit check");
    // Your limit checking logic here
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

    RLOG_CPP(0, std::fixed << std::setprecision(2)
             << "Timestamp: " << timestamp << " sec, "
             << "dt: " << dt << " , "
             << "Pan angle[deg]: " << RCS_RAD2DEG(pan_angle)
             << ", Tilt angle[deg]: " << RCS_RAD2DEG(tilt_angle)
             << ", Pan velocity[deg]: " << RCS_RAD2DEG(self->current_pan_velocity)
             << ", Tilt velocity[deg]: " << RCS_RAD2DEG(self->current_tilt_velocity));

    if (!self->filterInitialized)
    {
      self->filterInitialized = true;
      std::vector<double> q_init = { pan_angle, tilt_angle };
      self->panTiltFilt.init(q_init.data());
    }

    double filtPos[2], filtVel[2];
    self->panTiltFilt.iterate();
    self->panTiltFilt.getPosition(filtPos);
    self->panTiltFilt.getVelocity(filtVel);


    RLOG(0, "Filtered: pos[deg]: %.2f %.2f   vel[deg]: %.2f %.2f",
         RCS_RAD2DEG(filtPos[0]), RCS_RAD2DEG(filtPos[1]),
         RCS_RAD2DEG(filtVel[0]), RCS_RAD2DEG(filtVel[1]));

    double desired_pan_position = filtPos[0];
    double desired_tilt_position = filtPos[1];
    double desired_pan_velocity = filtVel[0];
    double desired_tilt_velocity = filtVel[1];

    if (self->feedbackFcn)
    {
      nlohmann::json fbJson;
      fbJson["time"] = Timer_getSystemTime();
      fbJson["position"] = std::vector<double> {self->current_pan_position, self->current_tilt_position};
      fbJson["pan_curr"] = self->current_pan_position;
      fbJson["tilt_curr"] = self->current_tilt_position;
      fbJson["pan_vel_curr"] = self->current_pan_velocity;
      fbJson["tilt_vel_curr"] = self->current_tilt_velocity;
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

    RLOG(0, "%s sending velocities[deg]: %.3f %.3f   errors: %.3f %.3f",
         success ? "SUCCESS" : "FAILURE",
         RCS_RAD2DEG(corrected_pan_velocity),
         RCS_RAD2DEG(corrected_tilt_velocity),
         RCS_RAD2DEG(desired_pan_position - self->current_pan_position),
         RCS_RAD2DEG(desired_tilt_position - self->current_tilt_position));
  }

  void start(std::function<void(const std::string&)> feedbackFcn_,
             const std::atomic_bool& run_flag,
             bool readOnly)
  {
    if (this->pw70)
    {
      RLOG(1, "PW70 already running - skipping start");
      return;
    }

    this->feedbackFcn = feedbackFcn_;

    // Create an instance of PW70CANInterface with the callbacks
    this->pw70 = std::make_unique<aff::PW70CANInterface>(limit_check, position_update, this, 50);
    this->pw70->reset_stop();

    // Wait a moment to allow the interface to initialize
    std::this_thread::sleep_for(std::chrono::seconds(2));
    this->pw70->move_position(-45.0, -30.0, 10.0, 10.0);
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

  bool setCommand(const std::string& message)
  {
    auto j = nlohmann::json::parse(message);

    // Parse message into motor commands here

    if (j.contains("q_des") && filterInitialized)
    {
      // We receive radians. Degrees are only used internally.
      auto q_des = j["q_des"].get<std::array<double, 2>>();
      this->panTiltFilt.setTarget(q_des.data());
      RLOG(0, "Setting filter target to %f %f", q_des[0], q_des[1]);
    }

    bool quitMe = j.value("quit", false);

    return quitMe;
  }

  int test()
  {
    aff::PW70CANInterface ptu(limit_check, position_update, nullptr, 50);

    // Wait a moment to allow the interface to initialize
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Example commands
    ptu.move_position(-45.0, -30.0, 10.0, 10.0);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    ptu.stop();
    ptu.cleanup();
    return 0;
  }

  std::unique_ptr<aff::PW70CANInterface> pw70;
  std::function<void(const std::string&)> feedbackFcn;

  double current_pan_position, current_tilt_position;
  double current_pan_velocity, current_tilt_velocity;
  double current_time_stamp;
  bool filterInitialized;
  Rcs::RampFilterND panTiltFilt;
};


/*******************************************************************************
 *
 *******************************************************************************/
int main(int argc, char** argv)
{
  signal(SIGINT, quit);   // Ctrl-C stops threads

  std::string sendEndpoint = "tcp://*:5559";
  std::string recvEndpoint = "tcp://*:5560";
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  bool readOnly = argP.hasArgument("-ro", "Read-only, no motor commands");

  FeedbackThread feedback;
  feedback.start(sendEndpoint, runLoop);

  PTUDriver robo;

  // auto fbFcn = std::bind(&FeedbackThread::updateMessage, &feedback, std::placeholders::_1);
  auto fbFcn = [&feedback](const std::string& message)
  {
    RLOG_CPP(1, "Feedback: " << message);
    feedback.updateMessage(message);
  };

  // auto cmdFcn = std::bind(&KortexDriver::setCommand, &robo, std::placeholders::_1);
  auto cmdFcn = [&robo](const std::string& message) -> bool
  {
    RLOG_CPP(1, "Received: " << message);
    return robo.setCommand(message);
  };

  std::vector<double> q_default(DOF_PTU, 0.0);
  VecNd_setRandom(q_default.data(), 1360.0, 1560.0, DOF_PTU);
  robo.start(fbFcn, runLoop, readOnly);


  // Start non-threaded
  bool blocking = true;
  CommandThread commands;
  commands.start(recvEndpoint, cmdFcn, runLoop, blocking);

  commands.stop();
  robo.stop();
  feedback.stop();

  return 0;
}
