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
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>

#include <unordered_set>



static const std::string taskNamePan             = "Pan";
static const std::string taskNameTilt            = "Tilt";
static const std::string taskNameLeftEyeBallDir  = "LeftEyeBallDir";
static const std::string taskNameRightEyeBallDir = "RightEyeBallDir";
static const std::string taskNameGazePointLeft   = "GazeL";
static const std::string taskNameGazePointRight  = "GazeR";

static const std::string panJointName            = "ptu_pan_joint";
static const std::string tiltJointName           = "ptu_tilt_joint";

static const std::string rightEyeBallName        = "RightEyeBall";
static const std::string leftEyeBallName         = "LeftEyeBall";




static std::vector<int> getJointIndexBackwardRecursion(const RcsGraph* graph, const std::string& bdyName)
{
  std::vector<int> jointIds;
  const RcsBody* startBody = RcsGraph_getBodyByName(graph, bdyName.c_str());
  RCHECK_MSG(startBody, "Couldn't find body '%s' in graph", bdyName.c_str());
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
  a_des(nullptr), x_des(nullptr), dx_des(nullptr), dH(nullptr), dq_des(nullptr),
  goalFilt(0.1, 1.0, parent->getDt(), 3),
  eStop(false), alpha(0.05), lambda(1.0e-8), t_gesture(-1.0),
  gazeMode(GazeMode::HeadEyeApproximate)
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
  MatNd_set(a_des, 1, 0, 0.0);    // Deactivate right eye ball direction
  MatNd_set(a_des, 2, 0, 0.0);    // Deactivate left eye ball direction
  setPanJointActivation(false);   // Deactivate pan dof
  setTiltJointActivation(false);  // Deactivate tilt dof

  REXEC(1)
  {
    controller->toXML("cGaze.xml", a_des);
  }

  // Collect joints that are part of the eye model
  std::vector<std::string> bdyNames;
  bdyNames.push_back(ActionEyeGaze::getRightGazePointName());
  bdyNames.push_back(ActionEyeGaze::getLeftGazePointName());
  bdyNames.push_back(ActionEyeGaze::getScreenName());
  bdyNames.push_back(ActionEyeGaze::getRightPupilName());
  bdyNames.push_back(ActionEyeGaze::getLeftPupilName());
  this->jointIds = getEyeModelJoints(graph, bdyNames);

  // Initialize gaze point consistent with the gaze
  const RcsBody* screen = RcsGraph_getBodyByName(graph, ActionEyeGaze::getScreenName().c_str());
  RCHECK(screen);
  double gazePt[3];
  Vec3d_add(gazePt, screen->A_BI.org, screen->A_BI.rot[2]);   // 1 m in front of screen
  goalFilt.init(gazePt);

  // Inititlize desired eye ball direction (for PupilDirection mode). The x-axis
  // points forward.
  const RcsBody* rightEyeBall = RcsGraph_getBodyByName(graph, rightEyeBallName.c_str());
  RCHECK(rightEyeBall);
  const RcsBody* leftEyeBall = RcsGraph_getBodyByName(graph, leftEyeBallName.c_str());
  RCHECK(leftEyeBall);
  Vec3d_copy(rightEyeDirCommand, rightEyeBall->A_BI.rot[0]);
  Vec3d_copy(leftEyeDirCommand, leftEyeBall->A_BI.rot[0]);

  // Initialize head gestures
  HeadNod* nod = new HeadNod("yes", 3.0, jointIds);
  nod->setAmplitude(RCS_DEG2RAD(6.0));
  headGestures.push_back(std::unique_ptr<HeadNod>(nod));

  HeadShake* shake = new HeadShake("no", 3.0, jointIds);
  shake->setAmplitude(RCS_DEG2RAD(18.0));
  headGestures.push_back(std::unique_ptr<HeadShake>(shake));

  // Event subscriptions
  subscribe("PostUpdateGraph", &EyeModelIKComponent::onComputeIK);
  subscribe("InitFromState", &EyeModelIKComponent::onInitFromState);
  subscribe("EmergencyStop", &EyeModelIKComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &EyeModelIKComponent::onEmergencyRecover);
  subscribe("SetGazeTarget", &EyeModelIKComponent::onSetGazeTarget);
  subscribe("SetPupilWeight", &EyeModelIKComponent::onSetPupilWeight);
  subscribe("StartGesture", &EyeModelIKComponent::onStartGesture);
  subscribe("GestureThreeRepetitions", &EyeModelIKComponent::onGestureThreeRepetitions);
  subscribe("SetEyeBallDirection", &EyeModelIKComponent::onEyeDirCommand);
  //subscribe("Render", &EyeModelIKComponent::onRender);

  // Generic checks
  RCHECK(controller->getTaskArrayIndex(taskNamePan.c_str())!=-1);
  RCHECK(controller->getTaskArrayIndex(taskNameTilt.c_str())!=-1);
  RCHECK(controller->getTaskArrayIndex(taskNameLeftEyeBallDir.c_str())!=-1);
  RCHECK(controller->getTaskArrayIndex(taskNameRightEyeBallDir.c_str())!=-1);
  RCHECK(controller->getTaskIndex(taskNameGazePointLeft.c_str())!=-1);
  RCHECK(controller->getTaskIndex(taskNameGazePointRight.c_str())!=-1);
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

  switch (this->gazeMode)
  {
    case GazeMode::HeadEyeApproximate:
    case GazeMode::HeadEyePrecise:
      computeIK_headEye(desired, current);
      break;

    case GazeMode::PupilDirection:
      computeIK_gazeDir(desired, current);
      break;

    default:
      RFATAL("Unknown gaze mode");
  }

}

void EyeModelIKComponent::computeIK_gazeDir(RcsGraph* desired, RcsGraph* current)
{
  // Keep consistent with gaze Diretion
  goalFilt.iterate();

  MatNd_setElementsTo(a_des, 1.0);
  MatNd_set(a_des, 0, 0, 0.0);   // Deactivate gaze point constraint
  MatNd_set(a_des, 1, 0, 1.0);   // Activate right eye ball direction
  MatNd_set(a_des, 2, 0, 1.0);   // Activate left eye ball direction
  setPanJointActivation(true);   // Deactivate pan dof
  setTiltJointActivation(true);  // Deactivate tilt dof

  const int gazeLIdx = controller->getTaskIndex(taskNameGazePointLeft.c_str());
  const int gazeRIdx = controller->getTaskIndex(taskNameGazePointRight.c_str());
  MatNd_set(a_des, gazeLIdx, 0, 0.0);
  MatNd_set(a_des, gazeRIdx, 0, 0.0);

  const int leftEyeIdx = controller->getTaskArrayIndex(taskNameLeftEyeBallDir.c_str());
  const int rightEyeIdx = controller->getTaskArrayIndex(taskNameRightEyeBallDir.c_str());

  Vec3d_getPolarAngles(&this->x_des->ele[leftEyeIdx], leftEyeDirCommand);
  Vec3d_getPolarAngles(&this->x_des->ele[rightEyeIdx], rightEyeDirCommand);
  RLOG(0, "Left eye dir: %f %f %f", leftEyeDirCommand[0], leftEyeDirCommand[1], leftEyeDirCommand[2]);
  RLOG(0, "Left polar angles: %f %f",
       RCS_RAD2DEG(this->x_des->ele[leftEyeIdx]),
       RCS_RAD2DEG(this->x_des->ele[leftEyeIdx+1]));

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

  // Apply all eye dof coordinates to constrained gaze dof in target graph
  for (const auto& j : jointIds)
  {
    desired->joints[j].constrained = true;
    desired->joints[j].weightMetric = controller->getGraph()->joints[j].weightMetric;
    const unsigned int jidx = controller->getGraph()->joints[j].jointIndex;
    desired->q->ele[jidx] = controller->getGraph()->q->ele[jidx];
  }

}

void EyeModelIKComponent::computeIK_headEye(RcsGraph* desired, RcsGraph* current)
{
  // Update gaze target
  const RcsBody* gazePtDes = RcsGraph_getBodyByName(desired, gazeTargetBody.c_str());
  if (gazePtDes)
  {
    goalFilt.setTarget(gazePtDes->A_BI.org);
  }
  else
  {
    // We set a gaze point 1m in front of the head so that the agent looks straight forward.
    const RcsBody* screen = RcsGraph_getBodyByName(desired, ActionEyeGaze::getScreenName().c_str());
    RCHECK(screen);
    double gazePt[3];
    Vec3d_add(gazePt, screen->A_BI.org,Vec3d_ex());   // 1 m in front of screen
    goalFilt.setTarget(gazePt);
  }

  goalFilt.iterate();
  goalFilt.getPosition(x_des->ele);

  MatNd_setElementsTo(this->a_des, 1.0);
  MatNd_set(this->a_des, 1, 0, 0.0);   // Deactivate right eye ball direction
  MatNd_set(this->a_des, 2, 0, 0.0);   // Deactivate left eye ball direction
  setPanJointActivation(false);
  setTiltJointActivation(false);

  // Gesture generation - variant 1 (of 2)
  if (gazeMode==GazeMode::HeadEyePrecise)
  {
    for (const auto& g : headGestures)
    {
      std::vector<double> panTilt = g->stepPrecise(controller, a_des, desired, getEntity()->getDt());

      if (!panTilt.empty())
      {
        const int panIdx = controller->getTaskArrayIndex(taskNamePan.c_str());
        const int tiltIdx = controller->getTaskArrayIndex(taskNameTilt.c_str());
        x_des->ele[panIdx] = panTilt[0];
        x_des->ele[tiltIdx] = panTilt[1];
        setPanJointActivation(true);
        setTiltJointActivation(true);
        //RLOG(0, "Gesture = %.2f %.2f", RCS_RAD2DEG(panTilt[0]), RCS_RAD2DEG(panTilt[1]));
      }
    }
  }

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

  // Apply all eye dof coordinates to constrained gaze dof in target graph
  for (const auto& j : jointIds)
  {
    desired->joints[j].constrained = true;
    desired->joints[j].weightMetric = controller->getGraph()->joints[j].weightMetric;
    const unsigned int jidx = controller->getGraph()->joints[j].jointIndex;
    desired->q->ele[jidx] = controller->getGraph()->q->ele[jidx];
  }

  // Gesture generation - variant 2 (of 2)
  if (gazeMode==GazeMode::HeadEyeApproximate)
  {
    for (const auto& g : headGestures)
    {
      g->step(controller->getGraph(), desired, getEntity()->getDt());
    }
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
  getEntity()->publish<std::string, const RcsGraph*>("RenderGraph", "Eye", controller->getGraph());
}

void EyeModelIKComponent::onSetGazeTarget(std::string bdyName)
{
  this->gazeMode = GazeMode::HeadEyeApproximate;
  this->gazeTargetBody = bdyName;
}

void EyeModelIKComponent::onEyeDirCommand(std::string sixValues)
{
  this->gazeMode = GazeMode::PupilDirection;

  std::vector<std::string> values = Rcs::String_split(sixValues, " ");
  RCHECK(values.size()==6);

  for (int i=0; i<3; ++i)
  {
    leftEyeDirCommand[i] = String_toDouble_l(values[i].c_str());
    rightEyeDirCommand[i] = String_toDouble_l(values[i+3].c_str());
  }

  Vec3d_normalizeSelf(leftEyeDirCommand);
  Vec3d_normalizeSelf(rightEyeDirCommand);

  const RcsBody* rightPupil = RcsGraph_getBodyByName(controller->getGraph(), ActionEyeGaze::getRightPupilName().c_str());
  const RcsBody* leftPupil = RcsGraph_getBodyByName(controller->getGraph(), ActionEyeGaze::getLeftPupilName().c_str());
  RCHECK(rightPupil);
  RCHECK(leftPupil);

  double rightPt[3], leftPt[3], midPt[3];
  Vec3d_add(rightPt, rightPupil->A_BI.org, rightEyeDirCommand);
  Vec3d_add(leftPt, leftPupil->A_BI.org, leftEyeDirCommand);
  Vec3d_addAndConstMul(midPt, rightPt, leftPt, 0.5);

  goalFilt.setTarget(midPt);
}

/*

GazePoint         XYZ
RightEyeBallDir   POLAR
LeftEyeBallDir    POLAR
Pan               Joint
Tilt              Joint
LeftPupil         Z
LeftPupil         POLAR
LeftGazePoint     XYZ
RightPupil        Z
RightPupil        POLAR
RightGazePoint    XYZ

 */

std::vector<std::string> EyeModelIKComponent::createTasksXML() const
{
  std::vector<std::string> tasks;
  tasks.push_back("<Task name=\"GazePoint\" effector=\"" + ActionEyeGaze::getGazePointName() + "\" controlVariable=\"XYZ\" />");

  tasks.push_back("<Task name=\"" + taskNameRightEyeBallDir + "\" effector=\"" + rightEyeBallName + "\" controlVariable=\"POLAR\" axisDirection=\"X\" />");
  tasks.push_back("<Task name=\"" + taskNameLeftEyeBallDir + "\" effector=\"" + leftEyeBallName + "\" controlVariable=\"POLAR\" axisDirection=\"X\" />");

  tasks.push_back("<Task name=\"" + taskNamePan + "\" jnt=\"" + panJointName + "\" controlVariable=\"Joint\" />");
  tasks.push_back("<Task name=\"" + taskNameTilt + "\" jnt=\"" + tiltJointName + "\" controlVariable=\"Joint\" />");

  auto eyeTasks = ActionEyeGaze::createEyeTasksXML();
  tasks.insert(tasks.end(), eyeTasks.begin(), eyeTasks.end());


  return tasks;
}

void EyeModelIKComponent::onSetPupilWeight(double weight)
{
  setPupilSpeedWeight(controller->getGraph(), weight);
}

void EyeModelIKComponent::onGestureThreeRepetitions(std::string gestureName, double gestureAmplitude)
{
  onStartGesture(gestureName, gestureAmplitude, 3);
}

void EyeModelIKComponent::onStartGesture(std::string gestureName, double gestureAmplitude, int numTurns)
{
  for (auto& g : headGestures)
  {
    if (g->getName() == gestureName)
    {
      g->setNumTurns(numTurns);
      g->setAmplitude(gestureAmplitude);
      g->setNumTurns(numTurns);
      g->start();
    }
  }
}

void EyeModelIKComponent::setPanJointActivation(bool enable)
{
  MatNd_set(a_des, 3, 0, enable ? 1.0 : 0.0);
}

void EyeModelIKComponent::setTiltJointActivation(bool enable)
{
  MatNd_set(a_des, 4, 0, enable ? 1.0 : 0.0);
}

// 0: Neck only, 1: pupils only
bool EyeModelIKComponent::setPupilSpeedWeight(RcsGraph* graph, double weight)
{
  if ((weight<0.0) || (weight>1.0))
  {
    RLOG(1, "Weight is %f but must be [0...1]", weight);
    return false;
  }

  RcsJoint* pan = RcsGraph_getJointByName(graph, panJointName.c_str());
  RcsJoint* tilt = RcsGraph_getJointByName(graph, tiltJointName.c_str());

  if (!pan)
  {
    RLOG_CPP(1, "Joint with name \"" << panJointName << "\" not found - skipping setting weight");
    return false;
  }

  if (!tilt)
  {
    RLOG_CPP(1, "Joint with name \"" << tiltJointName << "\" not found - skipping setting weight");
    return false;
  }

  pan->weightMetric = 1.0-weight;
  tilt->weightMetric = 1.0-weight;

  return true;
}

bool EyeModelIKComponent::hasEyeModel(const RcsGraph* graph)
{
  return ActionEyeGaze::hasEyeModel(graph);
}

}   // namespace aff
