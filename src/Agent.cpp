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

#include "Agent.h"
#include "ActionScene.h"
#include "Manipulator.h"

#include <Rcs_typedef.h>
#include <Rcs_shape.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_utils.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_body.h>
#include <Rcs_resourcePath.h>
#include <TaskFactory.h>
#include <IkSolverRMR.h>

#include <algorithm>
#include <exception>
#include <cfloat>

namespace aff
{

Agent::Agent(const xmlNodePtr node, const std::string& groupSuffix, const ActionScene* scene) : SceneEntity(node, groupSuffix)
{
  xmlNodePtr child = node->children;

  while (child)
  {
    if (isXMLNodeNameNoCase(child, "Component"))
    {
      std::string manipulatorName = Rcs::getXMLNodePropertySTLString(child, "manipulator");
      manipulatorName += groupSuffix;

      if (!manipulatorName.empty())
      {
        const Manipulator* m = scene->getManipulator(manipulatorName);
        RCHECK_MSG(m, "Manipulator '%s' of agent '%s' not found in scene",
                   manipulatorName.c_str(), name.c_str());

        // Manipulator found in scene, adding to Agent.
        manipulators.push_back(manipulatorName);
      }

    }

    child = child->next;
  }
}

Agent::Agent(const Agent& other) : SceneEntity(other), manipulators(other.manipulators)
{
}

Agent& Agent::operator= (const Agent& other)
{
  if (this == &other)
  {
    return *this;
  }

  SceneEntity::operator=(other);
  manipulators = other.manipulators;

  return *this;
}

Agent* Agent::clone() const
{
  return new Agent(*this);
}

Agent::~Agent()
{
}

std::string Agent::isLookingAt() const
{
  return std::string();
}

Agent* Agent::createAgent(const xmlNodePtr node, const std::string& groupSuffix, ActionScene* scene)
{
  Agent* agent = nullptr;

  std::vector<std::string> types = Rcs::getXMLNodePropertyVecSTLString(node, "types");

  if (std::find(types.begin(), types.end(), "robot") != types.end())
  {
    agent =  new RobotAgent(node, groupSuffix, scene);
  }
  else if (std::find(types.begin(), types.end(), "human") != types.end())
  {
    agent = new HumanAgent(node, groupSuffix, scene);
  }
  else
  {
    std::string name = Rcs::getXMLNodePropertySTLString(node, "name");
    RLOG(0, "[WARNING] Invalid config. Agent `%s` has invalid type or not specified! Defaulting to `robot`.",
         name.c_str());
  }

  return agent;
}

void Agent::print() const
{
  std::cout << name << " has " << types.size() << " types: ";

  for (const auto& t : types)
  {
    std::cout << t << " ";
  }

  std::cout << "\n   and " << manipulators.size() << " manipulators: ";

  for (const auto& m : manipulators)
  {
    std::cout << m << " ";
  }

  std::cout << std::endl;
}

bool Agent::canReachTo(const ActionScene* scene,
                       const RcsGraph* graph,
                       const double position[3]) const
{
  return false;
}

bool Agent::isVisible() const
{
  return true;
}

bool Agent::check(const ActionScene* scene,
                  const RcsGraph* graph) const
{
  return true;
}

std::vector<const Manipulator*> Agent::getManipulatorsOfType(const ActionScene* scene,
                                                             const std::string& type) const
{
  std::vector<const Manipulator*> typeManipulators;

  // Add manipulators whose types contains the requested name.
  for (const auto& mName : manipulators)
  {
    const Manipulator* m = scene->getManipulator(mName);
    if (std::find(m->types.begin(), m->types.end(), type) != m->types.end())
    {
      typeManipulators.push_back(m);
    }
  }

  return typeManipulators;
}

std::vector<const AffordanceEntity*> Agent::getObjectsInReach(const ActionScene* scene,
                                                              const RcsGraph* graph) const
{
  std::vector<const AffordanceEntity*> reachableNtts;

  for (auto& e : scene->entities)
  {
    if (getAffordances<Graspable>(&e).empty())
    {
      continue;
    }

    const double* nttPos = e.body(graph)->A_BI.org;
    if (canReachTo(scene, graph, nttPos))
    {
      reachableNtts.push_back(&e);
    }
  }

  return reachableNtts;
}

// \todo(MG): Check if manipulator is owned by several agents

/*static */ Agent* Agent::getAgentOwningManipulator(const ActionScene* scene,
                                                    const std::string& manipulatorName)
{
  for (const auto& a : scene->agents)
  {
    if (std::find(a->manipulators.begin(), a->manipulators.end(), manipulatorName) != a->manipulators.end())
    {
      return a;
    }

  }

  return nullptr;
}

RobotAgent::RobotAgent(const xmlNodePtr node,
                       const std::string& groupSuffix,
                       const ActionScene* scene) :
  Agent(node, groupSuffix, scene)
{
}

RobotAgent::RobotAgent(const RobotAgent& other) : Agent(other)
{
}

RobotAgent& RobotAgent::operator= (const RobotAgent& other)
{
  if (this == &other)
  {
    return *this;
  }

  Agent::operator=(other);

  return *this;
}

RobotAgent::~RobotAgent()
{
}

Agent* RobotAgent::clone() const
{
  return new RobotAgent(*this);
}

bool RobotAgent::check(const ActionScene* scene,
                       const RcsGraph* graph) const
{
  for (const auto& mName : manipulators)
  {
    const Manipulator* m = scene->getManipulator(mName);

    // If a robot agent has a manipulator that can grasp, we enforce that we
    // can compute the reachability. This requires being able to find a "shoulder"
    // or "base" joint.
    auto graspCapabilities = getCapabilities<GraspCapability>(m);
    if (!graspCapabilities.empty())
    {
      const RcsJoint* baseJnt = RcsGraph_getJointByName(graph, m->baseJointName.c_str());
      if (!baseJnt)
      {
        RLOG(1, "Couldn't find base joint '%s' for grasp-capable manipulator '%s' (%s)",
             m->baseJointName.c_str(), m->name.c_str(), m->bdyName.c_str());
        return false;
      }
    }
  }

  return true;
}

int RobotAgent::getPanTilt(const RcsGraph* graph, const std::string& gazeTarget,
                           double panTilt[2], size_t maxIter, double eps,
                           double err[2])
{
  // "ptu_pan_joint" "ptu_tilt_joint"
  RcsGraph* copyOfGraph = RcsGraph_clone(graph);
  Rcs::ControllerBase controller(copyOfGraph);   // Takes ownership of graph

  if (!copyOfGraph)
  {
    RLOG(1, "Graph for gazing IK could not be created");
    return -1;
  }

  const RcsBody* gazeBdy = RcsGraph_getBodyByName(copyOfGraph, gazeTarget.c_str());
  if (!gazeBdy)
  {
    RLOG(1, "gaze body '%s' not found", gazeTarget.c_str());
    return -1;
  }

  const RcsJoint* panJoint = RcsGraph_getJointByName(copyOfGraph, "ptu_pan_joint");
  if (!panJoint)
  {
    RLOG(1, "Pan joint 'ptu_pan_joint' not found");
    return -1;
  }

  const RcsJoint* tiltJoint = RcsGraph_getJointByName(copyOfGraph, "ptu_tilt_joint");
  if (!tiltJoint)
  {
    RLOG(1, "Tilt joint 'ptu_tilt_joint' not found");
    return -1;
  }

  const RcsBody* camera = RcsGraph_getBodyByName(copyOfGraph, "head_kinect_rgb_link");
  if (!camera)
  {
    RLOG(1, "Camera frame 'head_kinect_rgb_link' not found");
    return -1;
  }

  // Set pan and tilt joints to [0 0]
  double targetGazeLine[3];
  Vec3d_sub(targetGazeLine, gazeBdy->A_BI.org, camera->A_BI.org);
  if (Vec3d_diffAngle(camera->A_BI.rot[2], targetGazeLine) > RCS_DEG2RAD(20.0))
  {
    RLOG(1, "Initializing pan-tilt to 0 0");
    MatNd_set(copyOfGraph->q, panJoint->jointIndex, 0, 0.0);
    MatNd_set(copyOfGraph->q, tiltJoint->jointIndex, 0, 0.0);
  }
  else
  {
    RLOG(1, "pan-tilt diff angle is %f deg", RCS_RAD2DEG(Vec3d_diffAngle(camera->A_BI.rot[2], targetGazeLine)));
  }

  std::string cameraFrame = "head_kinect_rgb_link";
  std::string xmlTask = "<Task name=\"Gaze\" controlVariable=\"XY\" effector=\"" +
                        gazeTarget + "\" " + "refBdy=\"" + cameraFrame + "\" />";

  Rcs::Task* task = Rcs::TaskFactory::createTask(xmlTask, controller.getGraph());
  if (!task)
  {
    RLOG(1, "Gazing task could not be created");
    return -1;
  }
  controller.add(task);

  Rcs::IkSolverRMR ikSolver(&controller);

  MatNd* dq_des = MatNd_createLike(copyOfGraph->q);
  MatNd* x_des = MatNd_create(2,1);
  MatNd* dx_des = MatNd_create(2,1);
  MatNd* dH = NULL;
  MatNd* a = NULL;
  double lambda = 1.0e-8;
  size_t iter = 0;
  double clipLimit = 0.05;
  MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);

  for (iter=0; iter<maxIter; ++iter)
  {
    controller.computeDX(dx_des, x_des);
    MatNd_saturateSelf(dx_des, &clipArr);

    RLOG(1, "dx_des[%zu] = %f %f", iter, dx_des->ele[0], dx_des->ele[1]);

    if ((fabs(dx_des->ele[0])<eps) && (fabs(dx_des->ele[1])<eps))
    {
      break;
    }

    ikSolver.solveRightInverse(dq_des, dx_des, dH, a, lambda);
    MatNd_addSelf(copyOfGraph->q, dq_des);
    RcsGraph_setState(copyOfGraph, NULL, NULL);
  }

  panTilt[0] = MatNd_get(copyOfGraph->q, panJoint->jointIndex, 0);
  panTilt[1] = MatNd_get(copyOfGraph->q, tiltJoint->jointIndex, 0);

  if (err)
  {
    err[0] = fabs(dx_des->ele[0]);
    err[1] = fabs(dx_des->ele[1]);
    RLOG(1, "err[%zu] = %f %f", iter, err[0], err[1]);
  }

  MatNd_destroyN(3, dq_des, x_des, dx_des);

  return iter;
}

bool RobotAgent::canReachTo(const ActionScene* scene,
                            const RcsGraph* graph,
                            const double position[3]) const
{
  for (const auto& mName : manipulators)
  {
    const Manipulator* m = scene->getManipulator(mName);
    if (m->canReachTo(scene, graph, position))
    {
      return true;
    }
  }

  return false;
}

HumanAgent::HumanAgent(const xmlNodePtr node,
                       const std::string& groupSuffix,
                       const ActionScene* scene) :
  Agent(node, groupSuffix, scene), lastTimeSeen(0.0), visible(false)
{
  Vec3d_setZero(defaultPos);
  defaultRadius = DBL_MAX;

  tracker = Rcs::getXMLNodePropertySTLString(node, "tracker");
  getXMLNodePropertyVec3(node, "defaultPosition", defaultPos);
  getXMLNodePropertyDouble(node, "defaultRadius", &defaultRadius);
}

HumanAgent::HumanAgent(const HumanAgent& other) : Agent(other),
  lastTimeSeen(other.lastTimeSeen),
  visible(other.visible),
  tracker(other.tracker),
  markers(other.markers),
  gazeTarget(other.gazeTarget),
  gazeTargetPrev(other.gazeTargetPrev),
  defaultRadius(other.defaultRadius)
{
  Vec3d_copy(defaultPos, other.defaultPos);
  Vec3d_copy(gazeDirection, other.gazeDirection);
  Vec3d_copy(headPosition, other.headPosition);
}

HumanAgent& HumanAgent::operator= (const HumanAgent& other)
{
  if (this == &other)
  {
    return *this;
  }

  Agent::operator=(other);

  lastTimeSeen = other.lastTimeSeen;
  visible = other.visible;
  tracker = other.tracker;
  markers = other.markers;
  gazeTarget = other.gazeTarget;
  gazeTargetPrev = other.gazeTargetPrev;
  defaultRadius = other.defaultRadius;

  Vec3d_copy(defaultPos, other.defaultPos);
  Vec3d_copy(gazeDirection, other.gazeDirection);
  Vec3d_copy(headPosition, other.headPosition);

  return *this;
}

HumanAgent::~HumanAgent()
{
}

Agent* HumanAgent::clone() const
{
  return new HumanAgent(*this);
}

void HumanAgent::setVisibility(const bool newVisibilty)
{

  visible = newVisibilty;
}

bool HumanAgent::hasHead() const
{
  HTr tmp;
  return getHeadTransform(&tmp);
}

// Human head from Azure Kinect: y: fwd, x: up, z: left
bool HumanAgent::getHeadTransform(HTr* A_HI) const
{
  if (tracker!="azure_kinect")
  {
    RLOG(1, "Unsupported tracker: %s", tracker.c_str());
    return false;
  }

  if (markers.size()<27)
  {
    return false;
  }

  HTr_copy(A_HI, &markers[26]);
  return true;
}

bool HumanAgent::getHeadPosition(double pos[3]) const
{
  HTr A_head;
  if (!getHeadTransform(&A_head))
  {
    return false;
  }

  Vec3d_copy(pos, A_head.org);

  return true;
}

bool HumanAgent::getGazeDirection(double dir[3]) const
{
  HTr A_head;
  if (!getHeadTransform(&A_head))
  {
    return false;
  }

  Vec3d_copy(dir, A_head.rot[1]);

  return true;
}

bool HumanAgent::getHeadUpAxis(double dir[3]) const
{
  HTr A_head;
  if (!getHeadTransform(&A_head))
  {
    return false;
  }

  Vec3d_copy(dir, A_head.rot[0]);

  return true;
}

std::string HumanAgent::isLookingAt() const
{
  return gazeTarget;
}

bool HumanAgent::isVisible() const
{
  return visible;
}

// This is really bad. We test if the object's position is in a sphere with
// origin being the agent's head, with a very roughly estimated reach value.
bool HumanAgent::canReachTo(const ActionScene* scene,
                            const RcsGraph* graph,
                            const double position[3]) const
{
  for (const auto& mName : manipulators)
  {
    const Manipulator* m = scene->getManipulator(mName);
    if (m->isOfType("head"))
    {
      const RcsBody* head = RcsGraph_getBodyByName(graph, m->bdyName.c_str());
      RCHECK(head);
      const double avgArmLength = 0.77;   // rough estimate
      const double* headPos = head->A_BI.org;

      if (Vec3d_distance(position, headPos) < avgArmLength)
      {
        return true;
      }

    }
  }

  return false;
}

bool HumanAgent::computeAABB(double xyzMin[3], double xyzMax[3], MatNd* vertices) const
{
  if (markers.empty())
  {
    return false;
  }

  Vec3d_copy(xyzMin, markers[0].org);
  Vec3d_copy(xyzMax, markers[0].org);

  for (size_t i = 1; i < markers.size(); ++i)
  {
    const double* markerXYZ = markers[i].org;

    for (size_t j = 0; j < 3; ++j)
    {
      xyzMin[j] = std::min(markerXYZ[j], xyzMin[j]);
      xyzMax[j] = std::max(markerXYZ[j], xyzMax[j]);
    }
  }

  // Here we compute all 8 vertices of the boundig box.
  if (vertices)
  {
    double(*bb)[3] = (double(*)[3])vertices->ele;
    Vec3d_set(bb[0], xyzMin[0], xyzMin[1], xyzMin[2]);
    Vec3d_set(bb[1], xyzMin[0], -xyzMin[1], xyzMin[2]);
    Vec3d_set(bb[2], -xyzMin[0], xyzMin[1], xyzMin[2]);
    Vec3d_set(bb[3], -xyzMin[0], -xyzMin[1], xyzMin[2]);
    Vec3d_set(bb[4], xyzMax[0], xyzMax[1], xyzMax[2]);
    Vec3d_set(bb[5], xyzMax[0], -xyzMax[1], xyzMax[2]);
    Vec3d_set(bb[6], -xyzMax[0], xyzMax[1], xyzMax[2]);
    Vec3d_set(bb[7], -xyzMax[0], -xyzMax[1], xyzMax[2]);
  }

  return true;
}



} // namespace aff
