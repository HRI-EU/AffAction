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
#include "EulerConstraint.h"

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>

#include <algorithm>



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

  std::vector<const SceneEntity*> nttsToPointAt;
  for (const auto& e : scene.getSceneEntities(params[0]))
  {
    const Agent* agent = dynamic_cast<const Agent*>(e);
    if (agent)   // Select the agent's head as pointing target
    {
      auto m = agent->getManipulatorsOfType(&scene, "head");
      if (!m.empty())
      {
        nttsToPointAt.push_back(m[0]);
      }
    }
    else   // not an agent
    {
      nttsToPointAt.push_back(e);
    }

  }

  if (nttsToPointAt.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Could not find entity to point at for name '" + params[0] + "'.",
                          "Make sure the object exists in the scene.",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
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

  std::vector<const Manipulator*> hands;

  if (params.size()>1)
  {
    const Manipulator* hand = scene.getManipulator(params[1]);
    if (!hand)
    {
      throw ActionException(ActionException::ParamInvalid,
                            "Specified manipulator '" + params[1] + " ' not found in the scene.",
                            "Check manipulator name.",
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__));
    }

    hands.push_back(hand);
  }
  else
  {
    hands = robots[0]->getManipulatorsOfType(&scene, "hand");
  }

  if (hands.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't find a hand of robot " + robots[0]->name + " to point with");
  }

  // Assemble all combinations
  for (const auto& h : hands)
  {
    for (const auto& e : nttsToPointAt)
    {
      RLOG(0, "Adding '%s' - '%s' to map", h->bdyName.c_str(), e->bdyName.c_str());
      manipulatorEntityMap.push_back(std::make_tuple(h->bdyName, e->bdyName,
                                                     pointDistance(scene, graph, h->bdyName, e->bdyName)));
    }
  }

  // Sort the solutions somehow.
  std::sort(manipulatorEntityMap.begin(), manipulatorEntityMap.end(),
            [&](std::tuple<std::string,std::string,double>& t1, std::tuple<std::string,std::string,double>& t2)
  {
    double d1 = pointDistance(scene, graph, std::get<0>(t1), std::get<1>(t1));
    double d2 = pointDistance(scene, graph, std::get<0>(t2), std::get<1>(t2));

    return std::get<2>(t1) < std::get<2>(t2);
  });

  initialize(scene, graph, 0);
}

ActionPoint::~ActionPoint()
{
}

double ActionPoint::pointDistance(const ActionScene& scene, const RcsGraph* graph,
                                  const std::string& fingerName, const std::string& objectName) const
{
  auto fingers = scene.getSceneEntities(fingerName);
  auto objects = scene.getSceneEntities(objectName);
  RCHECK_MSG(fingers.size()==1, "Name %s has %zu entities", fingerName.c_str(), fingers.size());
  RCHECK_MSG(objects.size()==1, "Name %s has %zu entities", objectName.c_str(), objects.size());
  auto finger = fingers[0];
  auto object = objects[0];
  RCHECK_MSG(finger, "%s", fingerName.c_str());
  RCHECK_MSG(object, "%s", objectName.c_str());
  return Vec3d_distance(finger->getBodyTransform(graph).org, object->getBodyTransform(graph).org);
}

bool ActionPoint::initialize(const ActionScene& scene,
                             const RcsGraph* graph,
                             size_t solutionRank)
{
  if (solutionRank >= getNumSolutions())
  {
    return false;
  }

  pointerFrame = std::get<0>(manipulatorEntityMap[solutionRank]);
  pointBdyName = std::get<1>(manipulatorEntityMap[solutionRank]);

  const Manipulator* hand = scene.getManipulator(pointerFrame);
  RCHECK(hand);
  auto objects = scene.getSceneEntities(pointBdyName);
  RCHECK(objects.size()==1);
  auto object = objects[0];

  usedManipulators.clear();
  usedManipulators.push_back(hand->name);

  // Assemble finger task name
  fingerJoints.clear();
  for (auto& f : hand->fingerJoints)
  {
    fingerJoints += f;
    fingerJoints += " ";
  }

  // Determine kinematics: direction vector for orientation constraint
  const RcsJoint* shldrJnt = hand->getBaseJoint(graph);

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

  HTr objBdy = object->getBodyTransform(graph);
  Vec3d_sub(this->pointDirection, shldrJnt->A_JI.org, objBdy.org);
  double len = Vec3d_normalizeSelf(pointDirection);
  if (len==0.0)
  {
    throw ActionException(ActionException::KinematicallyImpossible,
                          "Couldn't determine Polar direction vector", "",
                          "Normalization failed - eye and object coincide");
  }

  this->reach = std::min(0.7*hand->reach, Vec3d_distance(shldrJnt->A_JI.org, objBdy.org)-0.3);
  this->taskPoint = "Point-" + pointerFrame + "-" + pointBdyName;
  this->taskOri = "Align-" + pointerFrame + "-" + pointBdyName;
  this->taskDist = "Distance" + pointerFrame + "-" + pointBdyName;
  this->taskFingers = pointerFrame + "_fingers";

  print();

  return true;
}

void ActionPoint::print() const
{
  ActionBase::print();

  for (size_t i=0; i<manipulatorEntityMap.size(); ++i)
  {
    std::cout << "Solution " << i << ": Manipulator: " << std::get<0>(manipulatorEntityMap[i])
              << " Object: " << std::get<1>(manipulatorEntityMap[i]) << std::endl;
  }
}

size_t ActionPoint::getNumSolutions() const
{
  return manipulatorEntityMap.size();
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

  xmlTask = "<Task name=\"" + taskOri + "\" " + "controlVariable=\"ABC\" " +
            "effector=\"" + pointerFrame + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskDist + "\" " + "controlVariable=\"SphR\" " +
            "effector=\"" + pointerFrame + "\" " + "refBdy=\"" + shoulderFrame + "\" />";
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
  a1->addActivation(t_start+0.0*(t_end-t_start), true, 0.5, taskOri);
  a1->addActivation(t_start, true, 0.5, taskDist);
  a1->addActivation(t_start, true, 0.5, taskFingers);

  if (!keepTasksActiveAfterEnd)
  {
    a1->addActivation(t_end + afterTime, false, 0.5, taskPoint);
    a1->addActivation(t_end + afterTime, false, 0.5, taskOri);
    a1->addActivation(t_end + afterTime, false, 0.5, taskDist);
    a1->addActivation(t_end + afterTime, false, 0.5, taskFingers);
  }

  double abc[3][3];
  Vec3d_copy(abc[2], pointDirection);   // z-axis from object to shoulder
  Vec3d_crossProduct(abc[0], abc[2], Vec3d_ez());   // force x-axis to horizontal plane
  Vec3d_normalizeSelf(abc[0]);
  Vec3d_crossProduct(abc[1], abc[2], abc[0]);
  RCHECK(Mat3d_isValid(abc));
  double ea[3];
  Mat3d_toEulerAngles(ea, abc);
  RLOG(0, "Euler angles: %f %f %f (%f %f %f)", ea[0], ea[1], ea[2],
       (180.0/M_PI)*ea[0], (180.0/M_PI)*ea[1], (180.0/M_PI)*ea[2]);
  a1->add(std::make_shared<tropic::EulerConstraint>(t_end, ea, taskOri));

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

std::string ActionPoint::getActionCommand() const
{
  return "point " + pointBdyName + " " + pointerFrame + " duration " + std::to_string(getDurationHint());
}

double ActionPoint::actionCost(const ActionScene& scene, const RcsGraph* graph) const
{
  return pointDistance(scene, graph, pointBdyName, pointerFrame);
  //return ActionBase::actionCost(scene, graph);
}


}   // namespace aff
