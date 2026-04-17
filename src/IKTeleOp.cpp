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

#include "IKTeleOp.h"
#include "TrajectoryPredictor.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>


namespace aff
{

IKTeleOp::IKTeleOp(EntityBase* parent, Rcs::ControllerBase* controller) :
  ComponentBase(parent), ikSolver(controller),
  eStop(false), alpha(0.05), lambda(1.0e-4),
  speedLimitCheck(true), jointLimitCheck(true),
  collisionCheck(true), applySpeedAndAccLimits(true),
  jointSpeedScaling(0.0, 0.33, parent->getDt())   // Transition in 3 seconds
{
  subscribe("EmergencyStop", &IKTeleOp::onEmergencyStop);
  subscribe("EmergencyRecover", &IKTeleOp::onEmergencyRecover);
  subscribe("Render", &IKTeleOp::onRender);
  subscribe("InitFromState", &IKTeleOp::onInitFromState);
  subscribe("Print", &IKTeleOp::print);
  subscribe("EnableRetargetting", &IKTeleOp::onEnableRetargetting);
}

const MatNd* IKTeleOp::getJointCommandPtr() const
{
  return ikSolver.getController()->getGraph()->q;
}

const RcsGraph* IKTeleOp::getGraph() const
{
  return ikSolver.getController()->getGraph();
}

RcsGraph* IKTeleOp::getGraph()
{
  return ikSolver.getController()->getGraph();
}

void IKTeleOp::onEnableRetargetting(bool enable)
{
  jointSpeedScaling.setTarget(enable ? 1.0 : 0.0);
}

void IKTeleOp::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  this->eStop = true;
  this->alpha = 0.0;
}

void IKTeleOp::onEmergencyRecover()
{
  RLOG(0, "EmergencyRecover");
  this->eStop = false;
}

void IKTeleOp::onInitFromState(const RcsGraph* target)
{
  RLOG(1, "IKTeleOp::onInitFromState()");
  RcsGraph_copy(getGraph(), target);
  ikSolver.getController()->computeCollisionModel();
}

void IKTeleOp::onRender()
{
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "IK", getGraph());
}

void IKTeleOp::setEnableSpeedAccelerationLimit(bool enable)
{
  this->applySpeedAndAccLimits = enable;
  RLOG(0, "Setting applySpeedAndAccLimits to %s", this->applySpeedAndAccLimits ? "TRUE" : "FALSE");
}

void IKTeleOp::setSpeedLimitCheck(bool enable)
{
  this->speedLimitCheck = enable;
  RLOG(0, "Setting speedLimitCheck to %s", this->speedLimitCheck ? "TRUE" : "FALSE");
}

void IKTeleOp::setJointLimitCheck(bool enable)
{
  this->jointLimitCheck = enable;
  RLOG(0, "Setting jointLimitCheck to %s", this->jointLimitCheck ? "TRUE" : "FALSE");
}

void IKTeleOp::setCollisionCheck(bool enable)
{
  this->collisionCheck = enable;
  RLOG(0, "Setting collisionCheck to %s", this->collisionCheck ? "TRUE" : "FALSE");
}

void IKTeleOp::print() const
{
  RcsCollisionModel_fprintCollisions(stdout, ikSolver.getController()->getNarrowPhase(), 1000.0);
  RcsGraph_fprintModelState(stdout, getGraph(), getGraph()->q, NULL, 0);
}

void IKTeleOp::setAlpha(double value)
{
  this->alpha = value;
  RLOG_CPP(0, "Setting alpha to " << this->alpha);
}

double IKTeleOp::getAlpha() const
{
  return this->alpha;
}

void IKTeleOp::setLambda(double value)
{
  this->lambda = value;
  RLOG_CPP(0, "Setting lambda to " << this->lambda);
}

double IKTeleOp::getLambda() const
{
  return this->lambda;
}

void IKTeleOp::onRetargetCommand(RcsGraph* desired, RcsGraph* current, ActionScene* scene)
{
  if (this->eStop)
  {
    return;
  }

  RCHECK(scene);
  auto agents = scene->getAgents<HumanAgent>();

  if (agents.size()!=1)
  {
    RLOG(0, "There's no or more than one agent");
    return;
  }

  const HumanAgent* agent = agents[0];
  const RcsBody* shl=nullptr, *shr=nullptr, *ell=nullptr, *elr=nullptr, *hl=nullptr, *hr=nullptr, *head=nullptr;


  auto it = agent->trackedFrames.find(HumanAgent::BodyType::ShoulderLeft);
  if (it != agent->trackedFrames.end())
  {
    shl = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::ShoulderRight);
  if (it != agent->trackedFrames.end())
  {
    shr = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::ElbowLeft);
  if (it != agent->trackedFrames.end())
  {
    ell = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::ElbowRight);
  if (it != agent->trackedFrames.end())
  {
    elr = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::HandLeft);
  if (it != agent->trackedFrames.end())
  {
    hl = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::HandRight);
  if (it != agent->trackedFrames.end())
  {
    hr = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::Head);
  if (it != agent->trackedFrames.end())
  {
    head = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  RCHECK(shl && shr && ell && elr && hl && hr && head);

  Rcs::ControllerBase* controller = ikSolver.getController();

  MatNd* x_des = MatNd_create(controller->getTaskDim(), 1);
  controller->computeX(x_des);


  // Polar angles calculation for arms: the z-axis is pointing from the
  // link's origin outwards (to distal direction).
  int array_idx = -1;

  // Left upper and lower arm
  array_idx = controller->getTaskArrayIndex("Right upperarm");
  if (array_idx != -1)
  {
    double a_uar[3];
    Vec3d_sub(a_uar, elr->A_BI.org, shr->A_BI.org);
    Vec3d_normalizeSelf(a_uar);
    Vec3d_getPolarAngles(x_des->ele + array_idx, a_uar);
  }

  array_idx = controller->getTaskArrayIndex("Right forearm");
  if (array_idx != -1)
  {
    double a_far[3];
    Vec3d_sub(a_far, hr->A_BI.org, elr->A_BI.org);
    Vec3d_normalizeSelf(a_far);
    Vec3d_getPolarAngles(x_des->ele + array_idx, a_far);
  }

  // Right upper and lower arm
  array_idx = controller->getTaskArrayIndex("Left upperarm");
  if (array_idx != -1)
  {
    double a_uar[3];
    Vec3d_sub(a_uar, ell->A_BI.org, shl->A_BI.org);
    Vec3d_normalizeSelf(a_uar);
    Vec3d_getPolarAngles(x_des->ele + array_idx, a_uar);
  }

  array_idx = controller->getTaskArrayIndex("Left forearm");
  if (array_idx != -1)
  {
    double a_far[3];
    Vec3d_sub(a_far, hl->A_BI.org, ell->A_BI.org);
    Vec3d_normalizeSelf(a_far);
    Vec3d_getPolarAngles(x_des->ele + array_idx, a_far);
  }

  // Left and right thumb direction
  array_idx = controller->getTaskArrayIndex("Right thumb");
  if (array_idx != -1)
  {
    const unsigned int tdim = controller->getTask("Right thumb")->getDim();

    if (tdim==2)
    {
      Vec3d_getPolarAngles(x_des->ele + array_idx, hr->A_BI.rot[2]);
    }
    else if (tdim==3)
    {
      Mat3d_toEulerAngles(x_des->ele + array_idx, (double (*)[3])hr->A_BI.rot);
    }
  }

  array_idx = controller->getTaskArrayIndex("Left thumb");
  if (array_idx != -1)
  {
    const unsigned int tdim = controller->getTask("Left thumb")->getDim();

    if (tdim==2)
    {
      Vec3d_getPolarAngles(x_des->ele + array_idx, hl->A_BI.rot[2]);
    }
    else if (tdim==3)
    {
      Mat3d_toEulerAngles(x_des->ele + array_idx, (double (*)[3])hl->A_BI.rot);
    }
  }

  // Head direction
  array_idx = controller->getTaskArrayIndex("Head");
  if (array_idx != -1)
  {
    const unsigned int tdim = controller->getTask("Head")->getDim();

    if (tdim==2)
    {
      Vec3d_getPolarAngles(x_des->ele + array_idx, head->A_BI.rot[2]);
    }
    else if (tdim==3)
    {
      Mat3d_toEulerAngles(x_des->ele + array_idx, (double (*)[3])head->A_BI.rot);
    }
  }



  MatNd* dx_des = MatNd_createLike(x_des);
  controller->computeDX(dx_des, x_des);

  MatNd* lambdaArr = MatNd_createLike(dx_des);
  MatNd_setElementsTo(lambdaArr, lambda);

  computeIK(nullptr, dx_des, lambdaArr);


  if (!agent->fingersLeft.empty())
  {
    const RcsJoint* jnt = RcsGraph_getJointByName(desired, "joint_0_0_left");
    if (jnt)
    {
      double* q_ptr = MatNd_getElePtr(desired->q, jnt->jointIndex, 0);
      if (q_ptr)
      {
        VecNd_copy(q_ptr, agent->fingersLeft.data(), agent->fingersLeft.size());
      }
    }
  }

  if (!agent->fingersRight.empty())
  {
    const RcsJoint* jnt = RcsGraph_getJointByName(desired, "joint_0_0_right");
    if (jnt)
    {
      double* q_ptr = MatNd_getElePtr(desired->q, jnt->jointIndex, 0);
      if (q_ptr)
      {
        VecNd_copy(q_ptr, agent->fingersRight.data(), agent->fingersRight.size());
      }
    }
  }



  MatNd_destroyN(3, x_des, dx_des, lambdaArr);
}

void IKTeleOp::onWrenchCommand(std::array<double, 6> wrench, bool inWorldFrame)
{
  if (this->eStop)
  {
    return;
  }

  RCHECK_MSG(inWorldFrame, "Only world frame supported");
  RCHECK(ikSolver.getController()->getTaskDim()>=wrench.size());

  std::array<double, 6> twist;
  MatNd* x_des = MatNd_create(ikSolver.getController()->getTaskDim(), 1);
  MatNd* dx_des = MatNd_createLike(x_des);

  VecNd_copy(x_des->ele, wrench.data(), wrench.size());
  ikSolver.getController()->computeDX(dx_des, x_des);
  VecNd_copy(twist.data(), dx_des->ele, twist.size());
  MatNd_destroyN(2, x_des, dx_des);

  onTwistCommand(twist, inWorldFrame);
}

void IKTeleOp::onTwistCommand(std::array<double, 6> twist, bool inWorldFrame)
{
  if (this->eStop)
  {
    return;
  }

  Rcs::ControllerBase* controller = ikSolver.getController();

  MatNd* dx_des = MatNd_create(controller->getTaskDim(), 1);
  MatNd* lambdaArr = MatNd_createLike(dx_des);

  // Transform angular velocities into body frame
  if (inWorldFrame)
  {
    const RcsBody* ef = controller->getTask(1)->getEffector();
    RCHECK(ef);
    Vec3d_rotateSelf(twist.data()+3, (double (*)[3])ef->A_BI.rot);
  }

  // From here on, twist is in world coordinates. We can use the "normal" Jacobian.
  size_t dimTaskTwist = controller->getTaskDim(0) + controller->getTaskDim(1);
  VecNd_copy(dx_des->ele, twist.data(), dimTaskTwist);

  const double lambdaOri = 0.01;
  MatNd_setElementsTo(lambdaArr, lambdaOri);
  Vec3d_set(lambdaArr->ele, lambda, lambda, lambda);
  computeIK(nullptr, dx_des, lambdaArr);

  MatNd_destroyN(2, dx_des, lambdaArr);
}

void IKTeleOp::computeIK(const MatNd* a_des, const MatNd* dx_des, const MatNd* lambdaArr)
{
  const double dt = getEntity()->getDt();
  ActionResult resMsg;
  RcsGraph* graph = getGraph();
  Rcs::ControllerBase* controller = ikSolver.getController();

  MatNd* dq_des = MatNd_create(graph->dof, 1);
  MatNd* dH = MatNd_create(1, graph->nJ);
  MatNd* qdot = MatNd_createLike(dq_des);

  controller->computeJointlimitGradient(dH);
  MatNd_constMulSelf(dH, alpha);
  double det = ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambdaArr);

  // We treat a singular configuration as an error. Typically, lambda is
  // set to a value larger than zero, therefore this will probably never
  // be a source of error.
  if (det == 0.0)
  {
    resMsg.error = "ERROR";
    resMsg.reason = "Got into a singular posture";
    MatNd_destroyN(3, dq_des, dH, qdot);
    return;
  }

  // Apply speed and acceleration limits only if speed limit check is set
  if (applySpeedAndAccLimits)
  {
    double scale = RcsGraph_getJointSpeedScaling(graph, dq_des, dt, RcsStateFull);
    if (scale < 1.0)
    {
      NLOG(0, "Scaling down joint speeds by factor %f", scale);
      MatNd_constMulSelf(dq_des, 0.99999 * scale);
    }

    // Apply acceleration limits
    const MatNd* qdot_prev = graph->q_dot;
    MatNd_constMul(qdot, dq_des, 1.0/dt);

    int nClippedAcc = RcsGraph_clipJointAccelerations(graph, qdot, qdot_prev,
                                                      dt, 1.0, RcsStateFull);
  }
  else
  {
    MatNd_constMul(qdot, dq_des, 1.0/dt);
  }

  // Update ramp for global joint speed scaling
  const double globalScaling = jointSpeedScaling.iterate();
  MatNd_constMulSelf(dq_des, globalScaling);
  MatNd_constMulSelf(qdot, globalScaling);


  // Scaling down velocities
  MatNd* q_dot_max = MatNd_createLike(graph->q_dot);
  RcsGraph_getSpeedLimits(graph, q_dot_max, RcsStateFull);
  MatNd_destroy(q_dot_max);


  // Integration and FK  including velocities
  MatNd_addSelf(graph->q, dq_des);

  if (applySpeedAndAccLimits)
  {
    RcsGraph_limitJoints(graph, graph->q, RcsStateFull);
  }

  RcsGraph_setState(graph, NULL, qdot);

  // We perform the check after the forward kinematics to consider the
  // pose after the IK step.
  controller->computeCollisionModel();

  // This method is static and doesn't modify the TrajectoryPredictor instance
  bool verbose = true;
  int res = TrajectoryPredictor::checkState(controller, speedLimitCheck, jointLimitCheck,
                                            collisionCheck, verbose, resMsg);

  // We only print this once after the e-stop being triggered, therefore the second comparison
  if ((res<0) && (eStop==false) && applySpeedAndAccLimits)
  {
    RLOG_CPP(0, "res = " << res << " E-Stopping, error = " << resMsg.error
             << " reason = " << resMsg.reason << " developer = " << resMsg.developer);
    getEntity()->publish("EmergencyStop");
  }

  // Clean up
  MatNd_destroyN(3, dq_des, dH, qdot);
}

double IKTeleOp::getJointSpeedScaling() const
{
  return jointSpeedScaling.getPosition();
}


}   // namespace aff
