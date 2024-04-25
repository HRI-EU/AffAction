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

#include "ActionPoint.h"
#include "ActionFactory.h"
#include "Agent.h"
#include "ActivationSet.h"
#include "VectorConstraint.h"
#include "PolarConstraint.h"

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#include <algorithm>

#define ORI_POLAR_INCL

namespace aff
{
REGISTER_ACTION(ActionPoint, "point");

ActionPoint::ActionPoint(const ActionScene& scene,
                         const RcsGraph* graph,
                         std::vector<std::string> params) :
  reach(0.0), keepTasksActiveAfterEnd(true)
{
  Vec3d_setZero(pointDirection);
  parseParams(params);

  if (params.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The object to point at is not specified.",
                          "Use an object name that is defined in the environment");
  }

  if (!params.size()==1)
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Received " + std::to_string(params.size()) + " objects to point at the same time.",
                          "Specify exactly one object to point at.",
                          "Number of passed strings is not 1");
  }

  std::vector<const AffordanceEntity*> ntts = scene.getAffordanceEntities(params[0]);
  const SceneEntity* ntt = nullptr;

  if (!ntts.empty())
  {
    if (ntts.size()!=1)
    {
      RLOG_CPP(1, "Found several object with the name " << params[0]);
    }

    ntt = ntts[0];
    pointBdyName = ntt->bdyName;
  }

  if (pointBdyName.empty())
  {
    const Agent* agent = scene.getAgent(params[0]);

    if (!agent)
    {
      throw ActionException(ActionException::ParamNotFound,
                            "The agent or object " + params[0] + " is unknown. ",
                            "Point at an object or agent name that is defined in the environment",
                            "This name was checked: " + params[0]);
    }

    // From here on, we have a valid agent. We point at its (first) head
    auto m = agent->getManipulatorsOfType(&scene, "head");
    if (!m.empty())
    {
      ntt = m[0];
      pointBdyName = ntt->bdyName;
    }

  }

  // Give up: pointTarget is no entity, no agent, no RcsBody.
  if (pointBdyName.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't point at " + params[0] + ". It is neither an entity nor an agent nor a body",
                          "Check if the given target is correct, or use another one");
  }

  // Find robot agent that should point. \todo: handle multiple agents
  auto robots = scene.getAgents<RobotAgent>();

  if (!robots.size() == 1)
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Found " + std::to_string(robots.size()) + " robots to point at the same time.",
                          "Only one robot is currently supported.",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  std::vector<const Manipulator*> hands = robots[0]->getManipulatorsOfType(&scene, "hand");
  if (hands.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't find a hand of robot " + robots[0]->name + " to point with");
  }

  this->pointerFrame = hands[0]->bdyName;


  // Assemble finger task name
  fingerJoints.clear();
  for (auto& f : hands[0]->fingerJoints)
  {
    fingerJoints += f;
    fingerJoints += " ";
  }


  auto heads = robots[0]->getManipulatorsOfType(&scene, "head");
  if (heads.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't find a head of robot " + robots[0]->name);
  }

  // Determine direction vector for Polar constraint
  HTr shldr = hands[0]->getBaseJointTransform(graph);
  HTr objBdy = ntt->getBodyTransform(graph);
  Vec3d_sub(this->pointDirection, shldr.org, objBdy.org);
  double len = Vec3d_normalizeSelf(pointDirection);
  if (len==0.0)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Couldn't determine Polar direction vector", "",
                          "Normalization failed - eye and object coincide");
  }

  this->reach = std::min(0.7*hands[0]->reach, Vec3d_distance(shldr.org, objBdy.org)-0.3);

  this->taskPoint = "Point-" + pointerFrame + "-" + pointBdyName;
  this->taskDir = "Align-" + pointerFrame + "-" + pointBdyName;
  this->taskDist = "Distance" + pointerFrame + "-" + pointBdyName;
  this->taskHandIncline = "Incline-" + pointerFrame;
  this->taskFingers = pointerFrame + "_fingers";
}

ActionPoint::~ActionPoint()
{
}

std::vector<std::string> ActionPoint::createTasksXML() const
{
  std::vector<std::string> tasks;

  // taskGaze: XYZ-task with effector=gazeTarget and refBdy=cameraFrame
  // was YZ
  std::string xmlTask;
  xmlTask = "<Task name=\"" + taskPoint + "\" " + "controlVariable=\"XY\" " +
            "effector=\"" + pointBdyName + "\" " + "refBdy=\"" + pointerFrame + "\" />";
  tasks.push_back(xmlTask);

#if defined (ORI_POLAR_INCL)
  xmlTask = "<Task name=\"" + taskDir + "\" " + "controlVariable=\"POLAR\" " +
            "effector=\"" + pointerFrame + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskHandIncline + "\" " + "controlVariable=\"Inclination\" axisDirection=\"X\" " +
            "effector=\"" + pointerFrame + "\" />";
  tasks.push_back(xmlTask);
#else
  xmlTask = "<Task name=\"" + taskDir + "\" " + "controlVariable=\"ABC\" " +
            "effector=\"" + pointerFrame + "\" />";
  tasks.push_back(xmlTask);
#endif

  xmlTask = "<Task name=\"" + taskDist + "\" " + "controlVariable=\"SphR\" " +
            "effector=\"" + pointerFrame + "\" " + "refBdy=\"" + "upperarm_left" + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerJoints + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionPoint::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskPoint);
  a1->addActivation(t_start+0.5*(t_end-t_start), true, 0.5, taskDir);
  a1->addActivation(t_start, true, 0.5, taskDist);
  a1->addActivation(t_start + 0.25 * (t_end - t_start), true, 0.5, taskHandIncline);
  a1->addActivation(t_start, true, 0.5, taskFingers);

  if (!keepTasksActiveAfterEnd)
  {
    a1->addActivation(t_end + afterTime, false, 0.5, taskPoint);
    a1->addActivation(t_end + afterTime, false, 0.5, taskDir);
    a1->addActivation(t_end + afterTime, false, 0.5, taskDist);
    a1->addActivation(t_end + afterTime, false, 0.5, taskHandIncline);
    a1->addActivation(t_end + afterTime, false, 0.5, taskFingers);
  }

#if defined (ORI_POLAR_INCL)
  double polarAngles[2];
  Vec3d_getPolarAngles(polarAngles, pointDirection);
  a1->add(std::make_shared<tropic::PolarConstraint>(t_end, polarAngles[0], polarAngles[1], taskDir));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, std::vector<double> {M_PI_2}, taskHandIncline));
#else
  double abc[3][3];
  Vec3d_copy(abc[0], pointDirection);
#endif

  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, std::vector<double> {0.0, 0.0}, taskPoint));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, std::vector<double> {reach}, taskDist));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, std::vector<double> {RCS_DEG2RAD(80.0), RCS_DEG2RAD(10.0), RCS_DEG2RAD(10.0)}, taskFingers));

  return a1;
}

std::vector<std::string> ActionPoint::getManipulators() const
{
  return usedManipulators;
}

std::unique_ptr<ActionBase> ActionPoint::clone() const
{
  return std::make_unique<ActionPoint>(*this);
}

}   // namespace aff
