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
#include "StringParserTools.hpp"

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_joint.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_utilsCPP.h>

#include <algorithm>
#include <sstream>

#define DEFAULT_NUM_SHAKES (3)
#define DEFAULT_UP_DOWN_AMPLITUDE  (0.03)
#define DEFAULT_TILT_AMPLITUDE  (8.0*M_PI/180.0)


namespace aff
{
REGISTER_ACTION(ActionShake, "shake");

ActionShake::ActionShake(const ActionScene& scene,
                         const RcsGraph* graph,
                         std::vector<std::string> params) :
  numUpAndDowns(DEFAULT_NUM_SHAKES),
  up_down_amplitude(DEFAULT_UP_DOWN_AMPLITUDE),
  tilt_amplitude(DEFAULT_TILT_AMPLITUDE)
{
  parseParams(params);

  int res = getAndEraseKeyValuePair(params, "number_of_shakes", numUpAndDowns);
  RCHECK_MSG(res >= -1, "%s", Rcs::String_concatenate(params, " ").c_str());

  res = getAndEraseKeyValuePair(params, "up_down_amplitude", up_down_amplitude);
  RCHECK_MSG(res >= -1, "%s", Rcs::String_concatenate(params, " ").c_str());
  if (res==0)
  {
    up_down_amplitude *= M_PI / 180.0;  // input in deg
  }

  res = getAndEraseKeyValuePair(params, "tilt_amplitude", tilt_amplitude);
  RCHECK_MSG(res >= -1, "%s", Rcs::String_concatenate(params, " ").c_str());
  if (res == 0)
  {
    tilt_amplitude *= M_PI / 180.0;  // input in deg
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

  this->roboBaseFrameName = robots[0]->bdyName;
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

  // Find the body proximal (closer to the root) of the shoulder joint
  const RcsBody* shldrBdy = graspingHand->getBaseJointBody(graph);

  if (!shldrBdy)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Couldn't determine shoulder body for manipulator " + graspingHand->name,
                          "Check configuration file",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  // shakeTransform: from robo-frame to shoulder frame, centered and a little below
  const HTr* A_SI = &shldrBdy->A_BI;
  HTr_invTransform(&shakeTransform, &A_RI, A_SI);
  shakeTransform.org[0] += 0.5*graspingHand->reach;
  shakeTransform.org[1] = 0.0;
  shakeTransform.org[2] -= 0.2;

  this->taskPosX = "PosX-" + shakeEntityName;
  this->taskPosY = "PosY-" + shakeEntityName;
  this->taskPosZ = "PosZ-" + shakeEntityName;
  this->taskOri = "Polar-" + graspingHandName;
  this->taskInclY = "InclinationY-" + graspingHandName;
  this->taskInclZ = "InclinationZ-" + graspingHandName;
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
  xmlTask = "<Task name=\"" + taskPosX + "\" " +
            "controlVariable=\"X\" effector=\"" + shakeEntityName +
            "\" refBdy=\"" + roboBaseFrameName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskPosY + "\" " +
            "controlVariable=\"Y\" effector=\"" + shakeEntityName +
            "\" refBdy=\"" + roboBaseFrameName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskPosZ + "\" " +
            "controlVariable=\"Z\" effector=\"" + shakeEntityName +
            "\" refBdy=\"" + roboBaseFrameName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskOri + "\" " +
            "controlVariable=\"POLAR\" effector=\"" + shakeEntityName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskInclY + "\" " +
            "controlVariable=\"Inclination\" effector=\"" + graspingHandName + "\" axisDirection=\"Y\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskInclZ + "\" " +
            "controlVariable=\"Inclination\" effector=\"" + graspingHandName + "\" axisDirection=\"Z\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionShake::createTrajectory(double t_start, double t_end) const
{
  const size_t numOsc = numUpAndDowns;
  const double afterTime = 0.5;
  const double duration = (t_end - t_start);
  const double t_reach = std::max(0.3 * duration, 4.0);
  const double t_mid = t_start + t_reach;
  //const double t_dt_swing = 0.5 * (0.9 - 0.3) * duration / (double)numOsc;
  const double t_dt_swing = (0.5 * (0.9 * duration - t_reach)) / (double)numOsc;
  std::vector<double> t_shake;

  for (size_t i = 0; i < numOsc; ++i)
  {
    t_shake.push_back(t_mid + (i*2+1)*t_dt_swing);
    t_shake.push_back(t_mid + (i*2+2)*t_dt_swing);
  }

  REXEC(0)
  {
    RLOG_CPP(0, "up_down_amplitude = " << up_down_amplitude);
    RLOG_CPP(0, "duration = " << duration);
    RLOG_CPP(0, "t_dt_swing = " << t_dt_swing);
    RLOG_CPP(0, "t_mid = " << t_mid);
    for (size_t i = 0; i < t_shake.size(); ++i)
    {
      RLOG_CPP(0, "t_shake[" << i << "] = " << t_shake[i]);
    }
    RLOG_CPP(0, "t_end = " << t_end);
  }

  const double height = shakeTransform.org[2];
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskPosX);
  a1->addActivation(t_start, true, 0.5, taskPosY);
  a1->addActivation(t_start, true, 0.5, taskPosZ);
  //a1->addActivation(t_start, true, 0.5, taskOri);
  a1->addActivation(t_start, true, 0.5, taskInclY);
  a1->addActivation(t_start, true, 0.5, taskInclZ);

  a1->addActivation(t_end + afterTime, false, 0.5, taskPosX);
  a1->addActivation(t_end + afterTime, false, 0.5, taskPosY);
  a1->addActivation(t_end + afterTime, false, 0.5, taskPosZ);
  //a1->addActivation(t_end + afterTime, false, 0.5, taskOri);
  a1->addActivation(t_end + afterTime, false, 0.5, taskInclY);
  a1->addActivation(t_end + afterTime, false, 0.5, taskInclZ);

  a1->add(t_mid, shakeTransform.org[0], 0.0, 0.0, 7, taskPosX + " 0");
  a1->add(t_mid, shakeTransform.org[1], 0.0, 0.0, 7, taskPosY + " 0");
  a1->add(t_mid, height, 0.0, 0.0, 7, taskPosZ + " 0");
  a1->add(t_mid, M_PI_2, 0.0, 0.0, 7, taskInclY + " 0");

  for (size_t i = 0; i < numOsc; ++i)
  {
    // Up and down
    a1->add(t_shake[i*2], height + up_down_amplitude, 0.0, 0.0, 7, taskPosZ + " 0");
    a1->add(t_shake[(i*2)+1], height - up_down_amplitude, 0.0, 0.0, 7, taskPosZ + " 0");

    // Rotate
    const double t_top = t_shake[i * 2];
    const double t_bottom = t_shake[(i * 2) + 1];
    const double t_half = 0.5 * fabs(t_bottom - t_top);
    a1->add(t_top, M_PI_2 + tilt_amplitude, 0.0, 0.0, 7, taskInclY + " 0");
    a1->add(t_top + t_half, M_PI_2 - tilt_amplitude, 0.0, 0.0, 7, taskInclY + " 0");
    a1->add(t_bottom, M_PI_2 + tilt_amplitude, 0.0, 0.0, 7, taskInclY + " 0");
    a1->add(t_bottom + t_half, M_PI_2 - tilt_amplitude, 0.0, 0.0, 7, taskInclY + " 0");
  }

  a1->add(t_end, height, 0.0, 0.0, 7, taskPosZ + " 0");
  a1->add(t_end, M_PI_2, 0.0, 0.0, 7, taskInclY + " 0");

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

  if (up_down_amplitude != DEFAULT_UP_DOWN_AMPLITUDE)
  {
    str += " up_down_amplitude " + std::to_string(RCS_RAD2DEG(up_down_amplitude));
  }

  if (tilt_amplitude != DEFAULT_TILT_AMPLITUDE)
  {
    str += " tilt_amplitude " + std::to_string(RCS_RAD2DEG(tilt_amplitude));
  }

  if (getDuration()!=getDefaultDuration())
  {
    str += " duration " + std::to_string(getDuration());
  }

  return str;
}

double ActionShake::getDefaultDuration() const
{
  const double amplitudeScaler = std::max(up_down_amplitude/DEFAULT_UP_DOWN_AMPLITUDE,
                                          tilt_amplitude/DEFAULT_TILT_AMPLITUDE);
  return 4.0 + amplitudeScaler * (numUpAndDowns * 3.0);
}












/*******************************************************************************
 *
 ******************************************************************************/
class ActionWeigh : public ActionShake
{
public:

  ActionWeigh(const ActionScene& scene,
              const RcsGraph* graph,
              std::vector<std::string> params) : ActionShake(scene, graph, params)
  {
    numUpAndDowns = 0;
  }


  std::unique_ptr<ActionBase> clone() const
  {
    return std::make_unique<ActionWeigh>(*this);
  }

  std::string getActionCommand() const
  {
    std::string str = "weigh " + shakeEntityName;

    if (getDuration() != getDefaultDuration())
    {
      str += " duration " + std::to_string(getDuration());
    }

    return str;
  }

  double getDefaultDuration() const
  {
    return 10.0;
  }

  tropic::TCS_sptr createTrajectory(double t_start, double t_end) const
  {
    const double afterTime = 0.5;
    auto a1 = std::make_shared<tropic::ActivationSet>();

    a1->addActivation(t_start, true, 0.5, taskPosX);
    a1->addActivation(t_start, true, 0.5, taskPosY);
    a1->addActivation(t_start, true, 0.5, taskPosZ);
    a1->addActivation(t_start, true, 0.5, taskOri);

    a1->addActivation(t_end + afterTime, false, 0.5, taskPosX);
    a1->addActivation(t_end + afterTime, false, 0.5, taskPosY);
    a1->addActivation(t_end + afterTime, false, 0.5, taskPosZ);
    a1->addActivation(t_end + afterTime, false, 0.5, taskOri);

    a1->add(t_end, shakeTransform.org[0], 0.0, 0.0, 7, taskPosX + " 0");
    a1->add(t_end, shakeTransform.org[1], 0.0, 0.0, 7, taskPosY + " 0");
    a1->add(t_end, shakeTransform.org[2], 0.0, 0.0, 7, taskPosZ + " 0");

    return a1;
  }

};

REGISTER_ACTION(ActionWeigh, "weigh");


}   // namespace aff
