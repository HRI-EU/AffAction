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

#include "ActionShake.h"
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

#define DEFAULT_NUM_SHAKES (3)


namespace aff
{
REGISTER_ACTION(ActionShake, "shake");

ActionShake::ActionShake(const ActionScene& scene,
                         const RcsGraph* graph,
                         std::vector<std::string> params) : numUpAndDowns(DEFAULT_NUM_SHAKES)
{
  parseParams(params);

  auto it = std::find(params.begin(), params.end(), "number_of_shakes");
  if (it != params.end())
  {
    std::stringstream stream(*(it + 1));
    stream >> numUpAndDowns;
    params.erase(it + 1);
    params.erase(it);
  }

  if (params.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The object to shake is not specified.",
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
                          "Could not find entity to shake for name '" + params[0] + "'.",
                          "Make sure the object exists in the scene.",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }


  // Find robot agent that should point. \todo: handle multiple agents
  auto robots = scene.getAgents<RobotAgent>();

  if (!robots.size() == 1)
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Found " + std::to_string(robots.size()) + " robots to shake the same time.",
                          "Only one robot is currently supported.",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  this->roboName = robots[0]->bdyName;

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

  // Determine kinematics: direction vector for orientation constraint
  const RcsJoint* shldrJnt = graspingHand->getBaseJoint(graph);

  // Find the body associated with the shoulder joint
  RCSGRAPH_FOREACH_BODY(graph)
  {
    RCSBODY_FOREACH_JOINT(graph, BODY)
    {
      if (JNT->id == shldrJnt->id)
      {
        shoulderFrame = BODY->name;
      }
    }
  }

  if (shoulderFrame.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Couldn't determine shoulder body for joint " + std::string(shldrJnt->name),
                          "Check configuration file",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  this->taskPosY = "PosY-" + shakeEntityName;
  this->taskPosZ = "PosZ-" + shakeEntityName;
  this->taskOri = "Polar-" + shakeEntityName;
}

ActionShake::~ActionShake()
{
}

void ActionShake::print() const
{
  ActionBase::print();

}

std::vector<std::string> ActionShake::createTasksXML() const
{
  std::vector<std::string> tasks;

  std::string xmlTask;
  xmlTask = "<Task name=\"" + taskPosY + "\" " +
            "controlVariable=\"Y\" effector=\"" + shakeEntityName +
            "\" refBdy=\"" + shoulderFrame +
            "\" refFrame=\"" + roboName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskPosZ + "\" " +
            "controlVariable=\"Z\" effector=\"" + shakeEntityName +
            "\" refBdy=\"" + shoulderFrame +
            "\" refFrame=\"" + roboName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskOri + "\" " +
            "controlVariable=\"POLAR\" effector=\"" + shakeEntityName + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionShake::createTrajectorySimple(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  const double t_mid = t_start + 0.3 * (t_end - t_start);
  const double t_1 = t_start + 0.4 * (t_end - t_start);
  const double t_2 = t_start + 0.5 * (t_end - t_start);
  const double t_3 = t_start + 0.6 * (t_end - t_start);
  const double t_4 = t_start + 0.7 * (t_end - t_start);
  const double t_5 = t_start + 0.8 * (t_end - t_start);
  const double t_6 = t_start + 0.9 * (t_end - t_start);
  const double height = -0.2;
  const double amplitude = 0.05;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskPosY);
  a1->addActivation(t_start, true, 0.5, taskPosZ);
  a1->addActivation(t_start, true, 0.5, taskOri);

  a1->addActivation(t_end + afterTime, false, 0.5, taskPosY);
  a1->addActivation(t_end + afterTime, false, 0.5, taskPosZ);
  a1->addActivation(t_end + afterTime, false, 0.5, taskOri);

  a1->add(t_mid, 0.0, 0.0, 0.0, 7, taskPosY + " 0");
  a1->add(t_mid, height, 0.0, 0.0, 1, taskPosZ + " 0");// flag 1 for seamless

  a1->add(t_1, height + amplitude, 0.0, 0.0, 7, taskPosZ + " 0");
  a1->add(t_2, height - amplitude, 0.0, 0.0, 7, taskPosZ + " 0");
  a1->add(t_3, height + amplitude, 0.0, 0.0, 7, taskPosZ + " 0");
  a1->add(t_4, height - amplitude, 0.0, 0.0, 7, taskPosZ + " 0");
  a1->add(t_5, height + amplitude, 0.0, 0.0, 7, taskPosZ + " 0");
  a1->add(t_6, height - amplitude, 0.0, 0.0, 7, taskPosZ + " 0");

  a1->add(t_end, height, 0.0, 0.0, 7, taskPosZ + " 0");

  return a1;
}

tropic::TCS_sptr ActionShake::createTrajectory(double t_start, double t_end) const
{
  const size_t numOsc = numUpAndDowns;
  const double afterTime = 0.5;
  const double duration = (t_end - t_start);
  const double t_dt_swing = 0.5*(0.9 - 0.3) * duration / (double)numOsc;
  const double t_mid = t_start + 0.3 * duration;
  std::vector<double> t_shake;

  for (size_t i = 0; i < numOsc; ++i)
  {
    t_shake.push_back(t_mid + (i*1)+1 * t_dt_swing);
    t_shake.push_back(t_mid + (i*1)+2 * t_dt_swing);
  }

  RLOG_CPP(0, "duration = " << duration);
  RLOG_CPP(0, "t_dt_swing = " << t_dt_swing);
  RLOG_CPP(0, "t_mid = " << t_mid);
  for (size_t i = 0; i < t_shake.size(); ++i)
  {
    RLOG_CPP(0, "t_shake[" << i << "] = " << t_shake[i]);
  }
  RLOG_CPP(0, "t_end = " << t_end);

  const double height = -0.2;
  const double amplitude = 0.05;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskPosY);
  a1->addActivation(t_start, true, 0.5, taskPosZ);
  a1->addActivation(t_start, true, 0.5, taskOri);

  a1->addActivation(t_end + afterTime, false, 0.5, taskPosY);
  a1->addActivation(t_end + afterTime, false, 0.5, taskPosZ);
  a1->addActivation(t_end + afterTime, false, 0.5, taskOri);

  a1->add(t_mid, 0.0, 0.0, 0.0, 7, taskPosY + " 0");
  a1->add(t_mid, height, 0.0, 0.0, 1, taskPosZ + " 0");

  for (size_t i = 0; i < numOsc; ++i)
  {
    a1->add(t_shake[i*2], height + amplitude, 0.0, 0.0, 7, taskPosZ + " 0");
    a1->add(t_shake[(i*2)+1], height - amplitude, 0.0, 0.0, 7, taskPosZ + " 0");
  }

  a1->add(t_end, height, 0.0, 0.0, 7, taskPosZ + " 0");

  return a1;
}

std::vector<std::string> ActionShake::getManipulators() const
{
  return usedManipulators;
}

std::unique_ptr<ActionBase> ActionShake::clone() const
{
  return std::make_unique<ActionShake>(*this);
}

std::string ActionShake::getActionCommand() const
{
  std::string str = "shake " + shakeEntityName;

  if (numUpAndDowns != DEFAULT_NUM_SHAKES)
  {
    str += " number_of_shakes " + std::to_string(numUpAndDowns);
  }

  if (getDuration()!=getDefaultDuration())
  {
    str += " duration " + std::to_string(getDuration());
  }

  return str;
}


}   // namespace aff
