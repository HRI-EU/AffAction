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

namespace aff
{

Agent::Agent(const xmlNodePtr node, const ActionScene* scene)
{
  name = Rcs::getXMLNodePropertySTLString(node, "name");
  //id = Rcs::getXMLNodePropertySTLString(node, "id");

  RLOG(5, "Adding Agent with name=%s", name.c_str());

  xmlNodePtr child = node->children;

  while (child)
  {
    if (isXMLNodeNameNoCase(child, "Component"))
    {
      std::string manipulatorName = Rcs::getXMLNodePropertySTLString(child, "manipulator");

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

Agent::~Agent()
{
}

std::string Agent::isLookingAt() const
{
  return std::string();
}

Agent* Agent::createAgent(const xmlNodePtr node, ActionScene* scene)
{
  Agent* agent = NULL;

  std::string type = Rcs::getXMLNodePropertySTLString(node, "type");

  if (type == "robot")
  {
    agent =  new RobotAgent(node, scene);
  }
  else if (type == "human")
  {
    agent = new HumanAgent(node, scene);
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
  std::cout << name << " has " << manipulators.size() << " manipulators:\n";

  for (const auto& m : manipulators)
  {
    std::cout << m << "\n   ";
  }

  std::cout << std::endl;
}

RobotAgent::RobotAgent(const xmlNodePtr node, const ActionScene* scene) : Agent(node, scene)
{
  type = "robot";
}

RobotAgent::~RobotAgent()
{
}

int RobotAgent::getPanTilt(const RcsGraph* graph, const std::string& gazeTarget,
                           double panTilt[2], size_t maxIter, double eps,
                           double err[2]) const
{
  // "ptu_pan_joint" "ptu_tilt_joint"
  RcsGraph* copyOfGraph = RcsGraph_clone(graph);

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

  Rcs::ControllerBase controller(copyOfGraph);   // Takes ownership of graph

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

  for (iter=0; iter<maxIter; ++iter)
  {
    controller.computeDX(dx_des, x_des);

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
  }

  MatNd_destroyN(3, dq_des, x_des, dx_des);

  return iter;
}

HumanAgent::HumanAgent(const xmlNodePtr node, const ActionScene* scene) :
  Agent(node, scene), isVisible(false), lastTimeSeen(0.0)
{
  type = "human";

  Vec3d_setZero(defaultPos);
  defaultRadius = DBL_MAX;

  tracker = Rcs::getXMLNodePropertySTLString(node, "tracker");
  getXMLNodePropertyVec3(node, "defaultPosition", defaultPos);
  getXMLNodePropertyDouble(node, "defaultRadius", &defaultRadius);
}

HumanAgent::~HumanAgent()
{
}

void HumanAgent::setVisibility(const bool newVisibilty)
{

  isVisible = newVisibilty;
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




} // namespace aff
