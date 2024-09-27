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

#ifndef PW70COMPONENT_H
#define PW70COMPONENT_H


#include "ComponentBase.h"

#include "Rcs_graph.h"
#include "Rcs_filters.h"


namespace aff
{
class PW70CANInterface;

class PW70Component : public ComponentBase
{
public:
  PW70Component(EntityBase* parent, int panDofIdx=-1, int tiltDofIdx=-1);
  virtual ~PW70Component();
  virtual bool init(int controlFrequency);

  static void limitCheck(double pan, double tilt, void* param);
  static void positionUpdate(double pan, double tilt, double timestamp, void* param);
  void onEnableCommands();
  void onInitPan();
  void onInitTilt();
  void onResetStop();
  void onStop();
  void onMovePosition(double pan_in_radians, double tilt_in_radians);
  void onUpdateGraph(RcsGraph* graph);

  // Disallow copying and assigning
  PW70Component(const PW70Component&) = delete;
  PW70Component& operator=(const PW70Component&) = delete;

protected:
  std::unique_ptr<PW70CANInterface> pw70;
  std::mutex panTiltUpdateMtx;

  double current_pan_position, current_tilt_position;
  double current_pan_velocity, current_tilt_velocity;
  double current_time_stamp;
  int panJointIdx, tiltJointIdx;
  bool enableCommands;
};


class PW70VelocityComponent : public PW70Component
{
public:
  PW70VelocityComponent(EntityBase* parent, int panDofIdx=-1, int tiltDofIdx=-1);
  virtual ~PW70VelocityComponent();
  virtual bool init(int controlFrequency);

  void onStart();
  void onStop();
  void onSetJointPosition(const MatNd* q_des);
  void onSetJointPositionInDegrees(double pan_in_deg, double tilt_in_deg);

  // Disallow copying and assigning
  PW70VelocityComponent(const PW70VelocityComponent&) = delete;
  PW70VelocityComponent& operator=(const PW70VelocityComponent&) = delete;

protected:
  static void positionUpdateVel(double pan, double tilt, double timestamp, void* param);
  void sinusoidalvelocityStep();
  void velocityControlStep(double desired_pan_position, double desired_tilt_position,
                           double desired_pan_velocity, double desired_tilt_velocity);

  std::mutex filtMtx;
  bool filterInitialized;
  Rcs::RampFilterND panTiltFilt;
};

}   // namespace

#endif // PW70COMPONENT_H
