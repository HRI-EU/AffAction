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

#include "EyeModelIKComponent.h"
#include "ActionEyeGaze.h"

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_body.h>

#include <unordered_set>



static std::vector<int> getJointIndexBackwardRecursion(const RcsGraph* graph, const std::string& bdyName)
{
  std::vector<int> jointIds;
  const RcsBody* startBody = RcsGraph_getBodyByName(graph, bdyName.c_str());
  RcsJoint* jnt = RcsBody_lastJointBeforeBody(graph, startBody);

  // Traverse backwards through the joints
  while (jnt)
  {
    if (!jnt->constrained)
    {
      jointIds.push_back(jnt->id);
    }
    jnt = RCSJOINT_BY_ID(graph, jnt->prevId);
  }

  return jointIds;
}

static std::vector<int> getEyeModelJoints(const RcsGraph* graph, std::vector<std::string> bdyNames)
{
  std::vector<int> allIds;

  for (const auto& b : bdyNames)
  {
    auto jntIds = getJointIndexBackwardRecursion(graph, b.c_str());
    allIds.insert(allIds.end(), jntIds.begin(), jntIds.end());
  }

  // Conversion to unordered_set eliminates duplicate entries
  std::unordered_set<int> unique_elements(allIds.begin(), allIds.end());
  std::vector<int> unique_vec(unique_elements.begin(), unique_elements.end());

  return unique_vec;
}


namespace aff
{

EyeModelIKComponent::EyeModelIKComponent(EntityBase* parent, const RcsGraph* graph) :
  ComponentBase(parent), controller(nullptr), ikSolver(nullptr),
  eStop(false), alpha(0.05), lambda(1.0e-8),
  goalFilt(0.1, 1.0, parent->getDt(), 3)
{
  this->controller = new Rcs::ControllerBase(RcsGraph_clone(graph));
  controller->setGraphOwnership(true);
  this->ikSolver = new Rcs::IkSolverRMR(controller);

  // Add constraints for eye model
  std::vector<std::string> taskVec = createTasksXML();
  std::vector<Rcs::Task*> tasks = Rcs::TaskFactory::createTasks(taskVec, controller->getGraph());

  for (auto t : tasks)
  {
    controller->add(t);
  }

  // These have constant dimensions
  this->a_des = MatNd_create(controller->getNumberOfTasks(), 1);
  this->x_des = MatNd_create(controller->getTaskDim(), 1);
  this->dx_des = MatNd_create(controller->getTaskDim(), 1);
  this->dH = MatNd_create(1, controller->getGraph()->nJ);
  this->dq_des = MatNd_create(controller->getGraph()->dof, 1);

  MatNd_setElementsTo(a_des, 1.0);

  // Collect joints that are part of the eye model
  std::vector<std::string> bdyNames;// = { screenSurface, rightPupil, leftPupil, leftGazePoint, rightGazePoint };
  bdyNames.push_back(ActionEyeGaze::getRightGazePointName());
  bdyNames.push_back(ActionEyeGaze::getLeftGazePointName());
  bdyNames.push_back(ActionEyeGaze::getScreenName());
  bdyNames.push_back(ActionEyeGaze::getRightPupilName());
  bdyNames.push_back(ActionEyeGaze::getLeftPupilName());
  this->jointIds = getEyeModelJoints(graph, bdyNames);

  for (const auto& j : jointIds)
  {
    RLOG(0, "jntid: %d (%s)", j, RCSJOINT_BY_ID(graph,j)->name);
  }

  // Initialize gaze point comliant with the gaze
  const RcsBody* screen = RcsGraph_getBodyByName(graph, ActionEyeGaze::getScreenName().c_str());
  RCHECK(screen);
  double gazePt[3];
  Vec3d_add(gazePt, screen->A_BI.org, screen->A_BI.rot[2]);   // 1 m in front of screen
  goalFilt.init(gazePt);

  subscribe("PostUpdateGraph", &EyeModelIKComponent::onComputeIK);
  subscribe("InitFromState", &EyeModelIKComponent::onInitFromState);
  subscribe("EmergencyStop", &EyeModelIKComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &EyeModelIKComponent::onEmergencyRecover);
  subscribe("Render", &EyeModelIKComponent::onRender);
  subscribe("SetGazeTarget", &EyeModelIKComponent::onSetGazeTarget);
}

EyeModelIKComponent::~EyeModelIKComponent()
{
  delete this->ikSolver;
  delete this->controller;

  MatNd_destroy(a_des);
  MatNd_destroy(x_des);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(dq_des);
}

void EyeModelIKComponent::onComputeIK(RcsGraph* desired, RcsGraph* current)
{
  if (this->eStop == true)
  {
    return;
  }

  // Update gaze target
  const RcsBody* gazePtDes = RcsGraph_getBodyByName(desired, gazeTargetBody.c_str());
  if (gazePtDes)
  {
    goalFilt.setTarget(gazePtDes->A_BI.org);
  }
  goalFilt.iterate();
  goalFilt.getPosition(x_des->ele);


  // Inverse kinematics. The vector x_des is all zero.
  controller->computeDX(dx_des, x_des);
  controller->computeJointlimitGradient(dH);
  MatNd_constMulSelf(dH, this->alpha);
  ikSolver->solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
  RcsGraph_limitJointSpeeds(controller->getGraph(), dq_des,
                            getEntity()->getDt(), RcsStateFull);
  MatNd_addSelf(controller->getGraph()->q, dq_des);

  // Forward kinematics including velocities
  MatNd_constMulSelf(dq_des, 1.0 / getEntity()->getDt());
  RcsGraph_setState(controller->getGraph(), NULL, dq_des);

  // Constrain gaze dof in passed graphs
  for (const auto& j : jointIds)
  {
    current->joints[j].constrained = true;
    desired->joints[j].constrained = true;

    const unsigned int jidx = controller->getGraph()->joints[j].jointIndex;
    desired->q->ele[jidx] = controller->getGraph()->q->ele[jidx];
  }

}

void EyeModelIKComponent::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  this->eStop = true;
}

void EyeModelIKComponent::onEmergencyRecover()
{
  RLOG(0, "EmergencyRecover");
  this->eStop = false;
}

void EyeModelIKComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(1, "EyeModelIKComponent::onInitFromState()");
  RcsGraph_copy(controller->getGraph(), target);
}

void EyeModelIKComponent::onRender()
{
  //getEntity()->publish<std::string, const RcsGraph*>("RenderGraph", "Eye", controller->getGraph());
}

void EyeModelIKComponent::onSetGazeTarget(std::string bdyName)
{
  gazeTargetBody = bdyName;
}

std::vector<std::string> EyeModelIKComponent::createTasksXML() const
{
  std::vector<std::string> tasks;
  tasks.push_back("<Task name=\"GazePoint\" effector=\"" + ActionEyeGaze::getGazePointName() + "\" controlVariable=\"XYZ\" />");

  auto eyeTasks = ActionEyeGaze::createEyeTasksXML();
  tasks.insert(tasks.end(), eyeTasks.begin(), eyeTasks.end());

  return tasks;
}

}   // namespace aff
