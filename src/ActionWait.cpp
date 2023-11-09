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

#include "ActionWait.h"
#include "ActionFactory.h"
#include "ActivationSet.h"
#include "VectorConstraint.h"
#include "JointWeightConstraint.h"

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>



namespace aff
{
REGISTER_ACTION(ActionWait, "wait");

ActionWait::ActionWait(const ActionScene& scene,
                       const RcsGraph* graph,
                       std::vector<std::string> params) : waitingTime(0.0)
{
  if (params.empty())
  {
    throw ActionException("ERROR REASON: Didn't receive time to wait. SUGGESTION: Specify time to wait", ActionException::ParamInvalid);
  }

  waitingTime = std::stod(params[0]);

  // Retrieve camera body \todo(MG): no explicit naming here
  cameraFrame = "head_kinect_rgb_link";
  const RcsBody* camBdy = RcsGraph_getBodyByNameNoCase(graph, cameraFrame.c_str());
  if (!camBdy)
  {
    throw ActionException("FATAL_ERROR REASON: Can't find a task to wait for. DEVELOPER: Body '" + cameraFrame + "' not found.", ActionException::ParamNotFound);
  }

  // Task naming
  this->taskGaze = "Gaze-" + cameraFrame;

  explanation = "I'm waiting for " + std::to_string(waitingTime) + " seconds";

  // \todo(MG): That's a shortcut, remove this and enable the waiting with a task as below.
  //throw ActionException("SUCCESS DEVELOPER: Currently shortcutted to not wait.", ActionException::NoError);
}

ActionWait::~ActionWait()
{
}

std::vector<std::string> ActionWait::createTasksXML() const
{
  std::vector<std::string> tasks;

  std::string xmlTask;
  xmlTask = "<Task name=\"" + taskGaze + "\" " + "controlVariable=\"POLAR\" " +
            "effector=\"" + cameraFrame + "\" axisDirection=\"Y\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionWait::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskGaze);
  a1->addActivation(t_end + afterTime, false, 0.5, taskGaze);

  return a1;
}

double ActionWait::getDurationHint() const
{
  return waitingTime;
}

std::string ActionWait::explain() const
{
  return explanation;
}

std::vector<std::string> ActionWait::getManipulators() const
{
  return std::vector<std::string>();
}

std::unique_ptr<ActionBase> ActionWait::clone() const {
      return std::make_unique<ActionWait>(*this);
}

}   // namespace aff
