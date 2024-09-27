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

#if defined (_MSC_VER)
#include "PW70CANInterfaceWin.hpp"
#else
#include "PW70CANInterfaceLinux.hpp"
#endif

namespace aff
{

void PW70CANInterface::limit_check(double pan, double tilt, void* param)
{
  std::cout << "Limit check" << std::endl;
  // Your limit checking logic here
}

void PW70CANInterface::position_update(double pan, double tilt, double timestamp, void* param)
{
  // Your position update logic here
  std::cout << "Position update" << std::endl;
}

int PW70CANInterface::test()
{
  PW70CANInterface ptu(limit_check, position_update, nullptr, 50);

  // Wait a moment to allow the interface to initialize
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Example commands
  ptu.move_position(-45.0, -30.0, 10.0, 10.0);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  ptu.stop();
  ptu.cleanup();
  return 0;
}

}   // namespace









#include "PW70Component.h"
#include <Rcs_macros.h>
#include <Rcs_basicMath.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>

#include <iostream>
#include <iomanip>

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


namespace aff
{


PW70Component::PW70Component(EntityBase* parent, int panDofIdx, int tiltDofIdx) :
  ComponentBase(parent),
  current_pan_position(0.0),
  current_tilt_position(0.0),
  current_pan_velocity(0.0),
  current_tilt_velocity(0.0),
  current_time_stamp(0.0),
  panJointIdx(panDofIdx),
  tiltJointIdx(tiltDofIdx),
  enableCommands(false)
{
  RLOG(0, "Creating PW70Component with panDofIdx=%d tiltDofIdx=%d", panDofIdx, tiltDofIdx);
  subscribe("PW70_InitPan", &PW70Component::onInitPan);
  subscribe("PW70_InitTilt", &PW70Component::onInitTilt);
  subscribe("PW70_ResetStop", &PW70Component::onResetStop);
  subscribe("PW70_MovePositionInDegrees", &PW70Component::onMovePosition);
  subscribe("EnableCommands", &PW70Component::onEnableCommands);
  subscribe("Stop", &PW70Component::onStop);
  subscribe("UpdateGraph", &PW70Component::onUpdateGraph);
}

bool PW70Component::init(int controlFrequency)
{
  if ((controlFrequency != 1) &&
      (controlFrequency != 10) &&
      (controlFrequency != 25) &&
      (controlFrequency != 50) &&
      (controlFrequency != 100))
  {
    RLOG(1, "Control frequency is %d, but must be one out of: 1, 10, 25, 50, 100", controlFrequency);
    return false;
  }

  // Create an instance of PW70CANInterface with the callbacks
  this->pw70 = std::make_unique<PW70CANInterface>(limitCheck, positionUpdate, this, controlFrequency);

  // Wait a moment to allow the interface to initialize
  std::this_thread::sleep_for(std::chrono::seconds(2));

  return true;
}

PW70Component::~PW70Component()
{
  //onStop();
}

void PW70Component::onUpdateGraph(RcsGraph* graph)
{
  if (panJointIdx==-1 || tiltJointIdx==-1)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(panTiltUpdateMtx);
  MatNd_set(graph->q, panJointIdx, 0, current_pan_position);
  MatNd_set(graph->q, tiltJointIdx, 0, current_tilt_position);
}

void PW70Component::limitCheck(double pan, double tilt, void* params)
{

  if (pan < PAN_MIN_RAD || pan > PAN_MAX_RAD)
  {
    RLOG_CPP(1, "Pan angle out of limits: " << RCS_RAD2DEG(pan) << " degrees");
  }

  if (tilt < TILT_MIN_RAD || tilt > TILT_MAX_RAD)
  {
    RLOG_CPP(1, "Tilt angle out of limits: " << RCS_RAD2DEG(tilt) << " degrees");
  }

}

void PW70Component::positionUpdate(double pan_angle, double tilt_angle, double timestamp, void* params)
{
  PW70VelocityComponent* self = static_cast<PW70VelocityComponent*>(params);

  std::lock_guard<std::mutex> lock(self->panTiltUpdateMtx);

  // Determine time delta for velocity calculation
  const double dt = timestamp - self->current_time_stamp;
  self->current_time_stamp = timestamp;
  self->current_pan_velocity = (pan_angle - self->current_pan_position) / dt;
  self->current_tilt_velocity = (tilt_angle - self->current_tilt_position) / dt;
  self->current_pan_position = pan_angle;
  self->current_tilt_position = tilt_angle;

  RLOG_CPP(2, std::fixed << std::setprecision(2)
           << "Timestamp: " << timestamp << " sec, "
           << "dt: " << dt<< " , "
           << "Pan angle[deg]: " << RCS_RAD2DEG(pan_angle)
           << ", Tilt angle[deg]: " << RCS_RAD2DEG(tilt_angle)
           << ", Pan velocity[deg]: " << RCS_RAD2DEG(self->current_pan_velocity)
           << ", Tilt velocity[deg]: " << RCS_RAD2DEG(self->current_tilt_velocity));
}

void PW70Component::onEnableCommands()
{
  enableCommands = true;
}

void PW70Component::onStop()
{
  if (pw70)
  {
    RLOG(0, "pw70->cleanup()");
    //pw70->cleanup();
    pw70.reset();
    RLOG(0, "pw70.reset()");
  }

}

void PW70Component::onInitPan()
{
  if (!pw70)
  {
    RLOG(1, "pw70 not initialized - skipping command");
    return;
  }
  pw70->reference_pan();
}

void PW70Component::onInitTilt()
{
  if (!pw70)
  {
    RLOG(1, "pw70 not initialized - skipping command");
    return;
  }
  pw70->reference_tilt();
}

void PW70Component::onResetStop()
{
  if (!pw70)
  {
    RLOG(1, "pw70 not initialized - skipping command");
    return;
  }
  pw70->reset_stop();
}

void PW70Component::onMovePosition(double pan_in_degrees, double tilt_in_degrees)
{
  if (!pw70)
  {
    RLOG(1, "pw70 not initialized - skipping command");
    return;
  }

  double maxVel = RCS_DEG2RAD(10.0);
  bool success = pw70->move_position(RCS_DEG2RAD(pan_in_degrees), RCS_DEG2RAD(tilt_in_degrees), maxVel, maxVel);
  RLOG(1, "%s sending target positions[deg]: %.3f %.3f",
       success ? "SUCCESS" : "FAILURE", pan_in_degrees, tilt_in_degrees);
}

















PW70VelocityComponent::PW70VelocityComponent(EntityBase* parent, int panDofIdx, int tiltDofIdx) :
  PW70Component(parent, panDofIdx, tiltDofIdx), filterInitialized(false), panTiltFilt(0.1, 0.0, 0.02, 2)
{
  subscribe("SetJointCommand", &PW70VelocityComponent::onSetJointPosition);
  subscribe("PW70_SetPanTiltInDegrees", &PW70VelocityComponent::onSetJointPositionInDegrees);

  const double dt = 0.02;   // 50Hz loop
  const double tmc = 0.1;
  panTiltFilt.setMaxVel(PAN_VELOCITY_MAX_RAD, 0);
  panTiltFilt.setMaxVel(TILT_VELOCITY_MAX_RAD, 1);
}

PW70VelocityComponent::~PW70VelocityComponent()
{
}

bool PW70VelocityComponent::init(int controlFrequency)
{
  panTiltFilt.setDt(1.0/controlFrequency);

  if ((controlFrequency != 1) &&
      (controlFrequency != 10) &&
      (controlFrequency != 25) &&
      (controlFrequency != 50) &&
      (controlFrequency != 100))
  {
    RLOG(1, "Control frequency is %d, but must be one out of: 1, 10, 25, 50, 100", controlFrequency);
    return false;
  }

  // Create an instance of PW70CANInterface with the callbacks
  this->pw70 = std::make_unique<PW70CANInterface>(limitCheck, positionUpdateVel, this, controlFrequency);

  // Wait a moment to allow the interface to initialize
  std::this_thread::sleep_for(std::chrono::seconds(2));

  return true;
}

void PW70VelocityComponent::onSetJointPosition(const MatNd* q_des)
{
  if (enableCommands)
  {
    if ((panJointIdx!=-1) && (tiltJointIdx != -1)  && filterInitialized)
    {
      // We clip the filtr goal to the permissable limits
      double panTiltTarget[2];
      panTiltTarget[0] = Math_clip(MatNd_get(q_des, panJointIdx, 0), PAN_MIN_RAD, PAN_MAX_RAD);
      panTiltTarget[1] = Math_clip(MatNd_get(q_des, tiltJointIdx, 0), TILT_MIN_RAD, TILT_MAX_RAD);
      std::lock_guard<std::mutex> lock(filtMtx);
      panTiltFilt.setTarget(panTiltTarget);
      RLOG(1, "Setting target to %.2f %.2f degrees",
           RCS_RAD2DEG(panTiltTarget[0]), RCS_RAD2DEG(panTiltTarget[1]));
    }
  }

}

void PW70VelocityComponent::onSetJointPositionInDegrees(double pan_in_deg, double tilt_in_deg)
{
  if (enableCommands && filterInitialized)
  {
    // We clip the filtr goal to the permissable limits
    double panTiltTarget[2];
    panTiltTarget[0] = Math_clip(RCS_DEG2RAD(pan_in_deg), PAN_MIN_RAD, PAN_MAX_RAD);
    panTiltTarget[1] = Math_clip(RCS_DEG2RAD(tilt_in_deg), TILT_MIN_RAD, TILT_MAX_RAD);
    std::lock_guard<std::mutex> lock(filtMtx);
    panTiltFilt.setTarget(panTiltTarget);
  }
}

void PW70VelocityComponent::positionUpdateVel(double pan_angle, double tilt_angle, double timestamp, void* params)
{
  PW70VelocityComponent* self = static_cast<PW70VelocityComponent*>(params);
  std::lock_guard<std::mutex> lock(self->filtMtx);

  if (!self->filterInitialized)
  {
    self->filterInitialized = true;
    double q_init[2];
    q_init[0] = pan_angle;
    q_init[1] = tilt_angle;
    self->panTiltFilt.init(q_init);
  }


  PW70Component::positionUpdate(pan_angle, tilt_angle, timestamp, params);

  self->panTiltFilt.iterate();

  double filtPos[2], filtVel[2];
  self->panTiltFilt.getPosition(filtPos);
  self->panTiltFilt.getVelocity(filtVel);
  RLOG(2, "Filtered: pos[deg]: %.2f %.2f   vel[deg]: %.2f %.2f",
       RCS_RAD2DEG(filtPos[0]), RCS_RAD2DEG(filtPos[1]),
       RCS_RAD2DEG(filtVel[0]), RCS_RAD2DEG(filtVel[1]));

  self->velocityControlStep(filtPos[0], filtPos[1], filtVel[0], filtVel[1]);
  //sinusoidalvelocityStep();
}

void PW70VelocityComponent::velocityControlStep(double desired_pan_position, double desired_tilt_position,
                                                double desired_pan_velocity, double desired_tilt_velocity)
{
  if (!pw70)
  {
    RLOG(0, "pw70 not initialized - quitting control thread");
    return;
  }

  // Proportional controller gain (adjust as necessary)
  const double Kp = 2.0; // Proportional gain for position error correction

  // Apply proportional control to compute the velocity correction
  double corrected_pan_velocity, corrected_tilt_velocity;
  {
    std::lock_guard<std::mutex> lock(panTiltUpdateMtx);
    double pan_velocity_correction = Kp * (desired_pan_position - current_pan_position);
    corrected_pan_velocity = desired_pan_velocity + pan_velocity_correction;
    corrected_pan_velocity = Math_clip(corrected_pan_velocity, -PAN_VELOCITY_MAX_RAD, PAN_VELOCITY_MAX_RAD);

    double tilt_velocity_correction = Kp * (desired_tilt_position - current_tilt_position);
    corrected_tilt_velocity = desired_tilt_velocity + tilt_velocity_correction;
    corrected_tilt_velocity = Math_clip(corrected_tilt_velocity, -TILT_VELOCITY_MAX_RAD, TILT_VELOCITY_MAX_RAD);
  }

  bool success = pw70->move_velocity(corrected_pan_velocity, corrected_tilt_velocity);

  RLOG(1, "%s sending velocities[deg]: %.3f %.3f   errors: %.3f %.3f",
       success ? "SUCCESS" : "FAILURE",
       RCS_RAD2DEG(corrected_pan_velocity),
       RCS_RAD2DEG(corrected_tilt_velocity),
       RCS_RAD2DEG(desired_pan_position - current_pan_position),
       RCS_RAD2DEG(desired_tilt_position - current_tilt_position));
}

void PW70VelocityComponent::sinusoidalvelocityStep()
{
  // Parameters for the sinusoidal trajectory
  const double amplitude = RCS_DEG2RAD(30.0);  // Degrees (peak amplitude of the sine wave)
  const double frequency = 0.1 * 4.0;   // Hz (frequency of the sine wave)
  const double angular_frequency = 2.0 * M_PI * frequency; // radians per second

  // Control loop parameters
  const double control_rate_hz = 50.0;    // Control loop frequency (50 Hz)
  const double control_interval = 1.0 / control_rate_hz; // Control loop interval (seconds)

  // Proportional controller gain (adjust as necessary)
  const double Kp = 0*2.0; // Proportional gain for position error correction

  // Timing variables
  static auto start_time = std::chrono::steady_clock::now();
  static auto current_time = start_time;
  static double elapsed_time = 0.0;

  // Calculate the desired pan position using the sinusoidal trajectory
  double desired_pan_position = amplitude * std::sin(angular_frequency * elapsed_time);

  // Calculate the desired pan velocity (derivative of position)
  double desired_pan_velocity = amplitude * angular_frequency * std::cos(angular_frequency * elapsed_time);

  // Get the current pan position from the position callback
  panTiltUpdateMtx.lock();
  double actual_pan_position = current_pan_position;
  panTiltUpdateMtx.unlock();

  // Compute the position error
  double position_error = desired_pan_position - actual_pan_position;

  // Apply proportional control to compute the velocity correction
  double velocity_correction = Kp * position_error;

  // Compute the corrected pan velocity command
  // Limit the corrected velocity to prevent abrupt movements
  double corrected_pan_velocity = Math_clip(desired_pan_velocity + velocity_correction, -PAN_VELOCITY_MAX_RAD, PAN_VELOCITY_MAX_RAD);

  // Command the pan velocity; keep tilt stationary
  pw70->move_velocity(corrected_pan_velocity, 0.0);

  // Sleep until the next control interval
  std::this_thread::sleep_for(std::chrono::duration<double>(control_interval));

  // Update elapsed time
  current_time = std::chrono::steady_clock::now();
  elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
}


}   // namespace
