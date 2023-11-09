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

#include "ActionGaze.h"
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
REGISTER_ACTION(ActionGaze, "gaze");

ActionGaze::ActionGaze(const ActionScene& scene,
                       const RcsGraph* graph,
                       std::vector<std::string> params) :
  isGazeTargetInHand(false),
  keepTasksActiveAfterEnd(true)
{
  if (params.empty())
  {
    throw ActionException("ERROR REASON: The object to gaze at is not specified. SUGGESTION: Use an object name that is defined in the environment", ActionException::ParamNotFound);
  }

  if (!params.size()==1)
  {
    throw ActionException("ERROR REASON: Received " + std::to_string(params.size()) +
                          " objects to gaze at the same time. SUGGESTION: Specify only one object to gaze at. DEVELOPER: Number of passed strings is not 1", ActionException::ParamInvalid);
  }

  std::vector<const AffordanceEntity*> ntts = scene.getAffordanceEntities(params[0]);

  if (ntts.empty())
  {
    throw ActionException("ERROR REASON:: The " + params[0] + " is unknown. SUGGESTION: Use an object name that is defined in the environment", ActionException::ParamNotFound);
  }
  // \todo(MG): Improve this for cases where they are far apart
  // else if (ntts.size()!=1)
  // {
  //   throw ActionException("ERROR REASON: Found several object with the name " +
  //                         params[0] + " to gaze at, but can only gaze at one of them. SUGGESTION: Specify the object more uniquely or gaze at something that is close to it.",
  //                         ActionException::ParamInvalid);
  // }

  gazeTargetInstance = ntts[0]->bdyName;
  gazeTarget = ntts[0]->name;

  // // Retrieve camera body \todo(MG): no explicit naming here
  // cameraFrame = "head_kinect_rgb_link";
  // const RcsBody* camBdy = RcsGraph_getBodyByNameNoCase(graph, cameraFrame.c_str());

  // \todo(CP): handle multiple agents
  std::vector<const Manipulator*> headsInScene = scene.getManipulatorsOfType("head");
  if (headsInScene.empty())
  {
    throw ActionException("FATAL_ERROR REASON: Can't find a head(camera) in the scene to gaze with.",
                          ActionException::ParamNotFound);
  }

  for (const auto h : headsInScene)
  {
    cameraFrame = h->getGazingFrame(graph);

    // BAD: using the first available `head` with `GazeCapability` in the scene atm. OK with only 1 robot in the scene.
    if (!cameraFrame.empty())
    {
      break;
    }
  }

  if (cameraFrame.empty())
  {
    throw ActionException("FATAL_ERROR REASON: Can't find a head with gazing capability in the scene.",
                          ActionException::ParamNotFound);
  }

  const RcsBody* camBdy = RcsGraph_getBodyByNameNoCase(graph, cameraFrame.c_str());

  if (!camBdy)
  {
    throw ActionException("FATAL_ERROR REASON: Can't find a head with gazing capability. DEVELOPER: Body '" + cameraFrame + "' not found.",
                          ActionException::ParamNotFound);
  }

  // Determine if object to be looked at has been grasped
  isGazeTargetInHand = scene.getGraspingHand(graph, ntts[0]) ? true : false;


  // Task naming
  this->taskGaze = "Gaze-" + cameraFrame + "-" + gazeTargetInstance;

  explanation = "I'm gazing at the " + gazeTarget;
}

ActionGaze::~ActionGaze()
{
}

std::vector<std::string> ActionGaze::createTasksXML() const
{
  std::vector<std::string> tasks;

  // taskGaze: XYZ-task with effector=gazeTarget and refBdy=cameraFrame
  // was YZ
  std::string xmlTask;
  xmlTask = "<Task name=\"" + taskGaze + "\" " + "controlVariable=\"XY\" " +
            "effector=\"" + gazeTargetInstance + "\" " + "refBdy=\"" + cameraFrame + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionGaze::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskGaze);

  if (!keepTasksActiveAfterEnd)
  {
    a1->addActivation(t_end + afterTime, false, 0.5, taskGaze);
  }

  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, std::vector<double> {0.0, 0.0}, taskGaze));

  // This freezes the pan/tilt dof if an object is held in hand. We can use the gaze ray to move the object
  if (isGazeTargetInHand)
  {
    a1->add(std::make_shared<tropic::JointWeightConstraint>(t_start, "ptu_pan_joint", 0.0, 0.0));
    a1->add(std::make_shared<tropic::JointWeightConstraint>(t_end, "ptu_pan_joint", 1.0, 1.0));
    a1->add(std::make_shared<tropic::JointWeightConstraint>(t_start, "ptu_tilt_joint", 0.0, 0.0));
    a1->add(std::make_shared<tropic::JointWeightConstraint>(t_end, "ptu_tilt_joint", 1.0, 1.0));
  }

  return a1;
}

double ActionGaze::getDurationHint() const
{
  return 3.0;
}

std::string ActionGaze::explain() const
{
  return explanation;
}

std::vector<std::string> ActionGaze::getManipulators() const
{
  return usedManipulators;
}

std::string ActionGaze::getGazeTarget() const
{
  return gazeTarget;
}

std::unique_ptr<ActionBase> ActionGaze::clone() const
{
  return std::make_unique<ActionGaze>(*this);
}

}   // namespace aff
