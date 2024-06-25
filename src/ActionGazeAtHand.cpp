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

#include "ActionGazeAtHand.h"
#include "ActionFactory.h"
#include "Agent.h"
#include "ActivationSet.h"
#include "PositionConstraint.h"

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>

#include <algorithm>
#include <sstream>


namespace aff
{
REGISTER_ACTION(ActionGazeAtHand, "inspect");

ActionGazeAtHand::ActionGazeAtHand(const ActionScene& scene,
                                   const RcsGraph* graph,
                                   std::vector<std::string> params)
{
  parseParams(params);

  if (params.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The object to inspect is not specified.",
                          "Use an object name that is defined in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  std::vector<const AffordanceEntity*> nttsToShake;
  for (const auto& e : scene.getAffordanceEntities(params[0]))
  {
    nttsToShake.push_back(e);
  }

  if (nttsToShake.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Could not find entity to inspect for name '" + params[0] + "'.",
                          "Make sure the object exists in the scene.",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }


  // Find robot agent that should point. \todo: handle multiple agents
  auto robots = scene.getAgents<RobotAgent>();

  if (!robots.size() == 1)
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Found " + std::to_string(robots.size()) + " robots to inspect the same time.",
                          "Only one robot is currently supported.",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  this->roboBaseFrame = robots[0]->bdyName;
  HTr A_RI = robots[0]->getBodyTransform(graph);

  auto gPair = scene.getGraspingHand(graph, nttsToShake);
  const Manipulator* graspingHand = std::get<0>(gPair);
  const AffordanceEntity* object = std::get<1>(gPair);

  if (!graspingHand)
  {
    throw ActionException(ActionException::KinematicallyImpossible,
                          "The " + nttsToShake[0]->name + " is not held in the hand.",
                          "First get the " + nttsToShake[0]->name + " in the hand before performing this command",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  this->graspingHandName = graspingHand->bdyName;
  this->shakeEntityName = object->bdyName;
  this->usedManipulators.push_back(graspingHand->name);

  // Determine camera base link
  auto heads = robots[0]->getManipulatorsOfType(&scene, "head");

  if (!heads.size() == 1)
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Found " + std::to_string(heads.size()) + " heads to inspect.",
                          "Only one head is currently supported.",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }
  const RcsBody* head = heads[0]->getBaseJointBody(graph);
  this->cameraBaseFrame = head->name;

  if (!head)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Couldn't determine head body for manipulator " + heads[0]->name,
                          "Check configuration file",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  // Determine kinematics: direction vector for orientation constraint
  const RcsBody* shldrBdy = graspingHand->getBaseJointBody(graph);

  if (!shldrBdy)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Couldn't determine shoulder body for manipulator " + graspingHand->name,
                          "Check configuration file",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  this->shoulderBaseFrame = shldrBdy->name;

  // Tasks
  this->taskObjectOri = "Ori-" + shakeEntityName;
  this->taskCenterXYZ = "XYZ-" + cameraBaseFrame + "-" + graspingHandName;
  this->taskRelShoulderY = "Y-" + graspingHandName + "-" + shoulderBaseFrame;
}

ActionGazeAtHand::~ActionGazeAtHand()
{
}

void ActionGazeAtHand::print() const
{
  ActionBase::print();

}

/*
<Task name="Polar-glass_green" controlVariable="POLAR" effector="glass_green" guiMin="-180 -180" guiMax="180 180" active="true" />
<Task name="t1" controlVariable="XYZ" effector="ptu_pan_link"  refBdy="hand_right_robot" active="true" />
<Task name="t4" controlVariable="Y" effector="hand_right_robot"  refBdy="shoulder_right"  refFrame="Johnnie" active="true" />
*/

std::vector<std::string> ActionGazeAtHand::createTasksXML() const
{
  std::vector<std::string> tasks;

  std::string xmlTask;
  xmlTask = "<Task name=\"" + taskObjectOri + "\" " +
            "controlVariable=\"POLAR\" effector=\"" + shakeEntityName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskCenterXYZ + "\" " +
            "controlVariable=\"XYZ\" effector=\"" + cameraBaseFrame +
            "\" refBdy=\"" + graspingHandName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskRelShoulderY + "\" " +
            "controlVariable=\"Y\" effector=\"" + graspingHandName +
            "\" refBdy=\"" + shoulderBaseFrame +
            "\" refFrame=\"" + roboBaseFrame + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionGazeAtHand::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  const double duration = (t_end - t_start);
  const double t_mid = t_start + 0.5 * duration;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskObjectOri);
  a1->addActivation(t_start, true, 0.5, taskCenterXYZ);
  a1->addActivation(t_start, true, 0.5, taskRelShoulderY);

  a1->addActivation(t_end + afterTime, false, 0.5, taskObjectOri);
  a1->addActivation(t_end + afterTime, false, 0.5, taskCenterXYZ);
  a1->addActivation(t_end + afterTime, false, 0.5, taskRelShoulderY);

  //a1->add(t_mid, 0.0, 0.0, 0.0, 7, taskCenterXYZ + " 0");   // Height
  a1->add(t_mid, -0.4, 0.0, 0.0, 7, taskCenterXYZ + " 2");  // Distance
  a1->add(std::make_shared<tropic::PositionConstraint>(t_end, 0.0, 0.0, -0.4, taskCenterXYZ));
  a1->add(t_end, 0.0, 0.0, 0.0, 7, taskRelShoulderY + " 0");

  return a1;
}

std::vector<std::string> ActionGazeAtHand::getManipulators() const
{
  return usedManipulators;
}

std::unique_ptr<ActionBase> ActionGazeAtHand::clone() const
{
  return std::make_unique<ActionGazeAtHand>(*this);
}

std::string ActionGazeAtHand::getActionCommand() const
{
  std::string str = "inspect " + shakeEntityName;

  if (getDuration()!=getDefaultDuration())
  {
    str += " duration " + std::to_string(getDuration());
  }

  return str;
}

double ActionGazeAtHand::getDefaultDuration() const
{
  return 15.0;
}

}   // namespace aff
