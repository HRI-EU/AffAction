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

#include "ActionPanTilt.h"
#include "ActionFactory.h"
#include "ActivationSet.h"
#include "VectorConstraint.h"
#include "JointWeightConstraint.h"
#include "Agent.h"

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#include <algorithm>

namespace aff
{
REGISTER_ACTION(ActionPanTilt, "pantilt");

ActionPanTilt::ActionPanTilt(const ActionScene& scene,
                             const RcsGraph* graph,
                             std::vector<std::string> params) :
  isGazeTargetInHand(false),
  keepTasksActiveAfterEnd(true)
{
  defaultDuration = 3.0;
  parseParams(params);

  if (params.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The object to gaze at is not specified.",
                          "Use an object name that is defined in the environment");
  }

  if (!params.size()==1)
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Received " + std::to_string(params.size()) + " objects to gaze at the same time.",
                          "Specify only one object to gaze at.DEVELOPER : Number of passed strings is not 1");
  }

  std::vector<const AffordanceEntity*> ntts = scene.getAffordanceEntities(params[0]);

  if (!ntts.empty())
  {
    if (ntts.size()!=1)
    {
      RLOG_CPP(1, "Found several object with the name " << params[0]);
    }

    gazeTargetInstance = ntts[0]->bdyName;
    gazeTarget = ntts[0]->name;
  }

  if (gazeTarget.empty())
  {
    const Agent* agent = scene.getAgent(params[0]);

    if (!agent)
    {
      throw ActionException(ActionException::ParamNotFound,
                            "The agent or object " + params[0] + " is unknown. ",
                            "Gaze at an object or agent name that is defined in the environment",
                            "This name was checked: " + params[0]);
    }

    // From here on, we have a valid agent. We look at its (first) head
    auto m = agent->getManipulatorsOfType(&scene, "head");
    if (!m.empty())
    {
      gazeTargetInstance = m[0]->bdyName;
      gazeTarget = m[0]->name;
    }

  }

  // Check if gaze target is a native RcsBody name
  if (gazeTarget.empty())
  {
    const RcsBody* gazeBdy = RcsGraph_getBodyByName(graph, params[0].c_str());
    if (gazeBdy)
    {
      gazeTargetInstance = params[0];
      gazeTarget = params[0];
    }
  }

  // Give up: gazeTarget is no entity, no agent, no RcsBody.
  if (gazeTarget.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't gaze at " + params[0] + ". It is neither an entity nor an agent nor a body",
                          "Check if the given target is correct, or use another one");
  }





  // // Retrieve camera body \todo(MG): no explicit naming here
  // cameraFrame = "head_kinect_rgb_link";
  // const RcsBody* camBdy = RcsGraph_getBodyByNameNoCase(graph, cameraFrame.c_str());

  // \todo(CP): handle multiple agents
  std::vector<const Manipulator*> headsInScene = scene.getManipulatorsOfType("head");
  if (headsInScene.empty())
  {
    throw ActionException(ActionException::ParamNotFound, "Can't find a head(camera) in the scene to gaze with.");
  }

  for (const auto h : headsInScene)
  {
    cameraFrame = h->getGazingFrame();

    // BAD: using the first available `head` with `GazeCapability` in the scene atm. OK with only 1 robot in the scene.
    if (!cameraFrame.empty())
    {
      break;
    }
  }

  if (cameraFrame.empty())
  {
    throw ActionException(ActionException::ParamNotFound, "Can't find a head with gazing capability in the scene.");
  }

  const RcsBody* camBdy = RcsGraph_getBodyByNameNoCase(graph, cameraFrame.c_str());

  if (!camBdy)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't find a head with gazing capability. DEVELOPER: Body '" + cameraFrame + "' not found.");
  }

  // Determine if object to be looked at has been grasped
  if (!ntts.empty())
  {
    isGazeTargetInHand = scene.getGraspingHand(graph, ntts[0]) ? true : false;
  }


  // Task naming
  this->taskGaze = "Gaze-" + cameraFrame + "-" + gazeTargetInstance;
}

ActionPanTilt::~ActionPanTilt()
{
}

std::vector<std::string> ActionPanTilt::createTasksXML() const
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

std::vector<std::string> ActionPanTilt::createTasksPanTiltXML() const
{
  std::vector<std::string> tasks;

  // taskGaze: XYZ-task with effector=gazeTarget and refBdy=cameraFrame
  // was YZ
  std::string xmlTask;
  xmlTask = "<Task name=\"pan\" controlVariable=\"Joint\" jnt=\"ptu_pan_joint\" />";
  tasks.push_back(xmlTask);
  xmlTask = "<Task name=\"tilt\" controlVariable=\"Joint\" jnt=\"ptu_tilt_joint\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionPanTilt::createTrajectory(double t_start, double t_end) const
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

tropic::TCS_sptr ActionPanTilt::createTrajectoryPanTilt(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskGaze);
  a1->addActivation(t_end + afterTime, false, 0.5, taskGaze);

  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, std::vector<double> {0.0, 0.0}, taskGaze));

  return a1;
}

std::vector<std::string> ActionPanTilt::getManipulators() const
{
  return usedManipulators;
}

std::string ActionPanTilt::getGazeTarget() const
{
  return gazeTarget;
}

std::unique_ptr<ActionBase> ActionPanTilt::clone() const
{
  return std::make_unique<ActionPanTilt>(*this);
}

}   // namespace aff
