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

#include <ActivationSet.h>
#include <VectorConstraint.h>
#include <EulerConstraint.h>
#include <PolarConstraint.h>
#include <PositionConstraint.h>
#include <CollisionModelConstraint.h>

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_shape.h>
#include <Rcs_geometry.h>

#include <algorithm>

#define OVERRIDE_3D

namespace aff
{
REGISTER_ACTION(ActionPoint, "point");

#define POINT_DEFAULT_DISTANCE (0.1)

ActionPoint::ActionPoint(const ActionScene& scene,
                         const RcsGraph* graph,
                         std::vector<std::string> params) :
  keepTasksActiveAfterEnd(false), fingerDistance(POINT_DEFAULT_DISTANCE)
{
  Vec3d_setZero(pointDirection);
  Vec3d_setZero(fingerTipPosition);
  parseParams(params);

  auto it = std::find(params.begin(), params.end(), "distance");
  if (it != params.end())
  {
    fingerDistance = std::stod(*(it + 1));
    params.erase(it + 1);
    params.erase(it);
  }

  if (params.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The object to point at is not specified.",
                          "Use an object name that is defined in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
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
                          "Can't find a hand of robot " + robots[0]->name + " to point with",
                          "Check xml file",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  // Assemble all combinations
  for (const auto& h : hands)
  {
    auto pointCapabilities = getCapabilities<FingerpushCapability>(h);
    if (pointCapabilities.empty())
    {
      throw ActionException(ActionException::ParamInvalid,
                            "Specified manipulator '" + params[1] + " ' has no pointing capability.",
                            "Check if FingerpushCapability has been added in xml file.",
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__));
    }



    for (const auto& pointCapability : pointCapabilities)
    {
      for (const auto& e : nttsToPointAt)
      {
        const double dist = pointDistance(scene, graph, pointCapability->frame, e->bdyName);
        manipulatorEntityMap.push_back(std::make_tuple(pointCapability->frame, e->bdyName, dist));
      }
    }
  }

  // Sort the solutions somehow.
  std::sort(manipulatorEntityMap.begin(), manipulatorEntityMap.end(),
            [&](std::tuple<std::string,std::string,double>& t1, std::tuple<std::string,std::string,double>& t2)
  {
    return std::get<2>(t1) < std::get<2>(t2);
  });

  initialize(scene, graph, 0);
}

ActionPoint::~ActionPoint()
{
}

bool ActionPoint::initialize(const ActionScene& scene,
                             const RcsGraph* graph,
                             size_t solutionRank)
{
  if (solutionRank >= getNumSolutions())
  {
    return false;
  }

  this->pointerFrame = std::get<0>(manipulatorEntityMap[solutionRank]);
  this->pointBdyName = std::get<1>(manipulatorEntityMap[solutionRank]);

  const Manipulator* hand = scene.getManipulatorOwningFrame(pointerFrame);
  RCHECK(hand);
  this->handName = hand->bdyName;
  this->pointingFingerAngles.clear();
  auto pointMdlState = Rcs::RcsGraph_getModelState(graph, "point_fingers");

  for (const auto& finger : hand->fingerJoints)
  {
    for (const auto& js : pointMdlState)
    {
      if (STREQ(graph->joints[js.first].name, finger.c_str()))
      {
        this->pointingFingerAngles.push_back(js.second);
      }
    }
  }

  if (hand->getNumFingers()!=pointingFingerAngles.size())
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Manipulator '" + hand->name + " ' number of fingers mismatch.",
                          "Check the 'point_fingers' model_state in your config file.",
                          "Hand " + hand->name + " has " + std::to_string(hand->getNumFingers()) +
                          " finger joints, but only " + std::to_string(pointingFingerAngles.size()) +
                          " found in xml file" + std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }


  auto objects = scene.getSceneEntities(pointBdyName);
  RCHECK(objects.size()==1);
  auto object = objects[0];

  this->usedManipulators.clear();
  this->usedManipulators.push_back(hand->name);

  // Assemble finger task name
  this->fingerJoints = Rcs::String_concatenate(hand->fingerJoints, " ");

  // Determine kinematics: direction vector for orientation constraint
  const RcsJoint* shldrJnt = hand->getBaseJoint(graph);
  this->shoulderFrame = shldrJnt->name;

  double fingerTipPosition[3];
  computeFingerTipPosition(scene, graph, object->body(graph), shldrJnt, fingerTipPosition);

  Vec3d_sub(this->pointDirection, fingerTipPosition, shldrJnt->A_JI.org);  // Point direction is from shoulder to object
  double len = Vec3d_normalizeSelf(pointDirection);
  if (len==0.0)
  {
    throw ActionException(ActionException::KinematicallyImpossible,
                          "Couldn't determine pointing direction vector", "",
                          "Normalization failed - shoulder and object centroid coincide");
  }



  const double reach = std::min(0.9*hand->reach, Vec3d_distance(shldrJnt->A_JI.org, fingerTipPosition) - fingerDistance);
  Vec3d_constMulAndAdd(this->fingerTipPosition, shldrJnt->A_JI.org, pointDirection, reach);
  this->taskOri = "Align-" + pointerFrame + "-" + pointBdyName;
  this->taskFingers = pointerFrame + "_fingers";
  this->taskFingerTip = pointerFrame + "_XYZ";

  return true;
}

void ActionPoint::computeFingerTipPosition(const ActionScene& scene,
                                           const RcsGraph* graph,
                                           const RcsBody* objBdy,
                                           const RcsJoint* shldrJnt,
                                           double fingerTipPos[3]) const
{
  // Compute object's AABB in world coordinates
  double xyzMin[3], xyzMax[3];
  Vec3d_copy(xyzMin, objBdy->A_BI.org);
  Vec3d_copy(xyzMax, objBdy->A_BI.org);
  RcsGraph_computeBodyAABB(graph, objBdy->id, RCSSHAPE_COMPUTE_DISTANCE, xyzMin, xyzMax, NULL);

  // Get centroid of frontal plane facing towards the robot. We do use the
  // frontal face so that we avoid pointing on the top of the object.
  HTr A_RI = scene.getAgents<RobotAgent>()[0]->getBodyTransform(graph);
  double vertices[8][3];
  Math_computeVerticesAABB(vertices, xyzMin, xyzMax);

  // Transform the 8 AABB vertices into robot's base frame
  std::vector<std::vector<double>> v(8, std::vector<double>(3, 0.0));
  for (int i = 0; i < 8; ++i)
  {
    Vec3d_invTransform(v[i].data(), &A_RI, vertices[i]);
  }

  // Sort the vertices according to their proximity to the base frame
  std::sort(v.begin(), v.end(),
            [&](std::vector<double>& v1, std::vector<double>& v2)
  {
    return v1[0] < v2[0];
  });

  // Compute the mean of the closest face which we assume to be
  // the first 4 vertices. There might be degenerate cases of
  // the first 4 vertices not belonging to the same face. We
  // ignore this here.
  double zMin = DBL_MAX, zMax = -DBL_MAX;
  Vec3d_setZero(fingerTipPos);
  for (int i = 0; i < 4; ++i)
  {
    fingerTipPos[0] += v[i][0];
    fingerTipPos[1] += v[i][1];
    fingerTipPos[2] += v[i][2];
    zMin = std::min(zMin, v[i][2]);
    zMax = std::max(zMax, v[i][2]);
  }
  Vec3d_constMulSelf(fingerTipPos, 0.25);
  fingerTipPos[2] += 0.25 * (zMax-zMin);

  // Transform back to into world coordinates (that's the task description)
  Vec3d_transformSelf(fingerTipPos, &A_RI);
}

void ActionPoint::print() const
{
  ActionBase::print();

  for (size_t i=0; i<pointingFingerAngles.size(); ++i)
  {
    RLOG_CPP(0, "Finger point angle[" << i << "] is " << pointingFingerAngles[i]);
  }

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
#if defined (OVERRIDE_3D)
  const bool withOri3d = true;
#else
  const bool withOri3d = pointingFingerAngles.size() > 1 ? true : false;
#endif

  std::string xmlTask;

  if (withOri3d)
  {
    xmlTask = "<Task name=\"" + taskOri + "\" " + "controlVariable=\"ABC\" " +
              "effector=\"" + pointerFrame + "\" />";
  }
  else
  {
    xmlTask = "<Task name=\"" + taskOri + "\" " + "controlVariable=\"POLAR\" " +
              "effector=\"" + pointerFrame + "\" axisDirection=\"X\" />";
  }

  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerJoints + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskFingerTip + "\" controlVariable=\"XYZ\" " +
            "effector=\"" + pointerFrame + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionPoint::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  auto a1 = std::make_shared<tropic::ActivationSet>();
#if defined (OVERRIDE_3D)
  const bool withOri3d = true;
#else
  const bool withOri3d = pointingFingerAngles.size() > 1 ? true : false;
#endif

  a1->addActivation(t_start + 0.0 * (t_end - t_start), true, 0.5, taskOri);
  a1->addActivation(t_start, true, 0.5, taskFingers);
  a1->addActivation(t_start, true, 0.5, taskFingerTip);

  if (!keepTasksActiveAfterEnd)
  {
    a1->addActivation(t_end + afterTime, false, 0.5, taskOri);
    a1->addActivation(t_end + afterTime, false, 0.5, taskFingers);
    a1->addActivation(t_end + afterTime, false, 0.5, taskFingerTip);
  }


  if (withOri3d)
  {
    double abc[3][3];
    Vec3d_copy(abc[0], pointDirection);   // x-axis from shoulder to object
    Vec3d_crossProduct(abc[1], abc[0], Vec3d_ey());   // force y-axis to point down vertically
    Vec3d_normalizeSelf(abc[1]);
    if (abc[1][2]>0.0)
    {
      Vec3d_constMulSelf(abc[1], -1.0);
    }
    Vec3d_crossProduct(abc[2], abc[0], abc[1]);

    RCHECK(Mat3d_isValid(abc));
    double ea[3];
    Mat3d_toEulerAngles(ea, abc);
    a1->add(std::make_shared<tropic::EulerConstraint>(t_start + 0.7 * (t_end - t_start), ea, taskOri));
  }
  else
  {
    double pa[2];
    Vec3d_getPolarAngles(pa, pointDirection);
    a1->add(std::make_shared<tropic::PolarConstraint>(t_start + 0.7 * (t_end - t_start), pa[0], pa[1], taskOri));
  }

  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, pointingFingerAngles, taskFingers));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_end, fingerTipPosition, taskFingerTip));

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
  std::string str = "point " + pointBdyName + " " + handName;

  if (getDuration()!=getDefaultDuration())
  {
    str += " duration " + std::to_string(getDuration());
  }

  if (fingerDistance != getDefaultFingerDistance())
  {
    str += " distance " + std::to_string(fingerDistance);
  }

  return str;
}

double ActionPoint::actionCost(const ActionScene& scene, const RcsGraph* graph) const
{
  return pointDistance(scene, graph, pointerFrame, pointBdyName);
}

double ActionPoint::pointDistance(const ActionScene& scene, const RcsGraph* graph,
                                  const std::string& fingerName, const std::string& objectName) const
{
  const RcsBody* pointFrame = RcsGraph_getBodyByName(graph, fingerName.c_str());
  RCHECK_MSG(pointFrame, "Name %s is not the pointing frame RcsBody", fingerName.c_str());
  auto objects = scene.getSceneEntities(objectName);
  RCHECK_MSG(objects.size()==1, "Name %s has %zu entities", objectName.c_str(), objects.size());
  auto object = objects[0];
  RCHECK_MSG(object, "%s", objectName.c_str());
  return Vec3d_distance(pointFrame->A_BI.org, object->getBodyTransform(graph).org);
}

double ActionPoint::getDefaultDuration() const
{
  return 15.0;
}

double ActionPoint::getDefaultFingerDistance() const
{
  return POINT_DEFAULT_DISTANCE;
}









/*******************************************************************************
 *
 ******************************************************************************/
#define POKE_DEFAULT_DISTANCE (-0.05)
#define POKE_RETRACT_DISTANCE (0.1)

class ActionPoke : public ActionPoint
{
public:

  ActionPoke(const ActionScene& scene,
             const RcsGraph* graph,
             std::vector<std::string> params) : ActionPoint(scene, graph, params)
  {
    // We pass params by value, therefore distance is still part of params
    // when we parse it here again. We can therefore safely repeat the parent's
    // class initialization at this point.
    auto it = std::find(params.begin(), params.end(), "distance");
    if (it != params.end())
    {
      fingerDistance = std::stod(*(it + 1));
      params.erase(it + 1);
      params.erase(it);
    }
    else
    {
      fingerDistance = POKE_DEFAULT_DISTANCE;
    }

    keepTasksActiveAfterEnd = true;
    Vec3d_setZero(retractPosition);
  }

  virtual ~ActionPoke()
  {
  }

  bool initialize(const ActionScene& scene,
                  const RcsGraph* graph,
                  size_t solutionRank)
  {
    bool success = ActionPoint::initialize(scene, graph, solutionRank);
    pointDirection[2] *= 0.25;
    Vec3d_normalizeSelf(pointDirection);
    Vec3d_constMulAndAdd(retractPosition, fingerTipPosition, pointDirection, -POKE_RETRACT_DISTANCE);

    return success;
  }

  std::unique_ptr<ActionBase> clone() const
  {
    return std::make_unique<ActionPoke>(*this);
  }

  std::string getActionCommand() const
  {
    std::string pointCmd = ActionPoint::getActionCommand();
    return "poke" + pointCmd.substr(5);
  }

  tropic::TCS_sptr createTrajectory(double t_start, double t_end) const
  {
    const double afterTime = 0.5;
    const double duration = t_end - t_start;
    const double t_push = t_start + 0.6 * duration;
    auto a1 = std::make_shared<tropic::ActivationSet>();

    a1->add(ActionPoint::createTrajectory(t_start, t_push));

    // Deactivate and reactivate object collisions
    a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_start+0.3*duration, pointBdyName, false));
    a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_end, pointBdyName, true));

    // Move hand a little bit back
    a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.4 * duration, retractPosition, taskFingerTip, 1));
    a1->add(std::make_shared<tropic::PositionConstraint>(t_end, retractPosition, taskFingerTip));

    // Deactivate tasks after finishing
    a1->addActivation(t_end, false, 0.5, taskOri);
    a1->addActivation(t_end + afterTime, false, 0.5, taskFingers);
    a1->addActivation(t_end + afterTime, false, 0.5, taskFingerTip);

    return a1;
  }

  virtual double getDefaultFingerDistance() const
  {
    return POKE_DEFAULT_DISTANCE;
  }

  double retractPosition[3];
};

REGISTER_ACTION(ActionPoke, "poke");


}   // namespace aff
