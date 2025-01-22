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

#ifndef AFF_EYEMODELIKCOMPONENT_H
#define AFF_EYEMODELIKCOMPONENT_H

#include "ComponentBase.h"
#include "HeadGesture.h"

#include <IkSolverRMR.h>
#include <Rcs_filters.h>
#include <memory>



namespace aff
{

class EyeModelIKComponent : public ComponentBase
{
public:

  EyeModelIKComponent(EntityBase* parent, const RcsGraph* graph);
  virtual ~EyeModelIKComponent();

  static bool hasEyeModel(const RcsGraph* graph);
  void setPanJointName(const std::string& name);
  void setTiltJointName(const std::string& name);
  bool setPupilSpeedWeight(RcsGraph* graph, double weight);

private:

  void onEmergencyStop();
  void onEmergencyRecover();
  void onInitFromState(const RcsGraph* target);
  void onComputeIK(RcsGraph* desired, RcsGraph* current);
  void onRender();
  void onSetGazeTarget(std::string bdyName);
  void onStartGesture(std::string gestureName, double amplitude, int numTurns);
  void onSetPupilWeight(double weight);

  void setPanJointActivation(bool enable);
  void setTiltJointActivation(bool enable);

  std::vector<std::string> createTasksXML() const;
  std::vector<int> jointIds;

  Rcs::ControllerBase* controller;
  Rcs::IkSolverRMR* ikSolver;
  MatNd* a_des;
  MatNd* x_des;
  MatNd* dx_des;
  MatNd* dH;
  MatNd* dq_des;
  std::string gazeTargetBody;
  std::string panJointName;   // default: "ptu_pan_joint"
  std::string tiltJointName;  // default: "ptu_tilt_joint"
  Rcs::RampFilterND goalFilt;

  bool eStop;
  double alpha;
  double lambda;
  double t_gesture;

  std::vector<std::unique_ptr<HeadGesture>> headGestures;

  /*! \brief We disallow copying and assigning this class.
   */
  EyeModelIKComponent(const EyeModelIKComponent&) = delete;
  EyeModelIKComponent& operator=(const EyeModelIKComponent&) = delete;
};

}

#endif   // AFF_EYEMODELCOMPONENT_H
