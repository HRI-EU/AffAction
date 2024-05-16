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

#include "IKComponent.h"
#include "TrajectoryPredictor.h"

#include <TaskFactory.h>
#include <IkSolverConstraintRMR.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>


namespace aff
{

IKComponent::IKComponent(EntityBase* parent,
                         Rcs::ControllerBase* controller_, IkSolverType ik) :
  ComponentBase(parent), controller(controller_), ikSolver(NULL), a_prev(NULL),
  eStop(false), alphaMax(0.05), alpha(0.0), lambda(1.0e-6), blending(1.0), phase(0.0),
  qFilt(0.0), renderSolid(false), speedLimitCheck(true), jointLimitCheck(true),
  collisionCheck(true), applySpeedAndAccLimits(true)
{
  switch (ik)
  {
    case RMR:
      ikSolver = new Rcs::IkSolverRMR(controller);
      break;

    case ConstraintRMR:
      ikSolver = new Rcs::IkSolverConstraintRMR(controller);
      break;

    default:
      RFATAL("Unknown IkSolverType: %d", ik);
  }

  subscribeAll();
}

IKComponent::~IKComponent()
{
  delete this->ikSolver;
  MatNd_destroy(this->a_prev);
}

void IKComponent::subscribeAll()
{
  subscribe("SetTaskCommand", &IKComponent::onTaskCommand);
  subscribe("SetBlending", &IKComponent::onSetBlending);
  subscribe("SetPhase", &IKComponent::onSetPhase);
  subscribe("LinkGenericBody", &IKComponent::onLinkGenericBody);
  subscribe("EmergencyStop", &IKComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &IKComponent::onEmergencyRecover);
  subscribe("Render", &IKComponent::onRender);
  subscribe("InitFromState", &IKComponent::onInitFromState);
  subscribe("TriggerInitFromDesiredState", &IKComponent::onTriggerInitFromDesiredState);
  subscribe("Print", &IKComponent::print);
  subscribe("ChangeTaskVector", &IKComponent::onTaskVectorChange);
}

void IKComponent::onTaskCommand(const MatNd* a, const MatNd* x)
{
  if (this->eStop == true)
  {
    return;
  }

  if (this->a_prev==NULL)
  {
    this->a_prev = MatNd_clone(a);
  }

  bool taskSwitch = !MatNd_isEqual(a_prev, a, 1.0e-3);
  const double qFiltDecay = 0.1;

  if (taskSwitch)
  {
    qFilt = 1.0;
  }

  if (qFiltDecay<=0.0)
  {
    qFilt = 0.0;
  }
  else
  {
    qFilt = Math_clip(qFilt-(1.0/qFiltDecay)*getEntity()->getDt(), 0, 1.0);
  }




  TrajectoryPredictor::FeedbackMessage resMsg;
  int ikOk = TrajectoryPredictor::computeIK(ikSolver, a, x, getEntity()->getDt(), alpha*blending,
                                            lambda, qFilt, phase, speedLimitCheck, jointLimitCheck,
                                            collisionCheck, applySpeedAndAccLimits, true, NULL, resMsg);

  // We only print this once after the e-stop being triggered, therefore the
  // second comparison
  if ((ikOk<0) && (eStop==false))
  {
    RLOG_CPP(0, "ikOK = " << ikOk << " E-Stopping, error = " << resMsg.error << " reason = " << resMsg.reason);
  }

  // Gradually activate null space so that it takes 1 second from 0 to alphaMax.
  // This is only happening after class construction / initialization, but not
  // during run-time.
  this->alpha += getEntity()->getDt()*alphaMax;
  if (this->alpha > this->alphaMax)
  {
    this->alpha = this->alphaMax;
  }

  // Memorize last activation vector to detect task switches
  MatNd_copy(this->a_prev, a);
}

const MatNd* IKComponent::getJointCommandPtr() const
{
  return controller->getGraph()->q;
}

const RcsGraph* IKComponent::getGraph() const
{
  return controller->getGraph();
}

RcsGraph* IKComponent::getGraph()
{
  return controller->getGraph();
}

void IKComponent::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  this->eStop = true;
  this->alpha = 0.0;
}

void IKComponent::onEmergencyRecover()
{
  RLOG(0, "EmergencyRecover");
  this->eStop = false;
}

void IKComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(1, "IKComponent::onInitFromState()");
  RcsGraph_copy(controller->getGraph(), target);
  controller->computeCollisionModel();
}

void IKComponent::onTriggerInitFromDesiredState()
{
  RLOG(0, "IKComponent::onTriggerInitFromDesiredState()");
  getEntity()->publish<const RcsGraph*>("InitFromState", controller->getGraph());
}

void IKComponent::onRender()
{
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "IK",
                                                    controller->getGraph());

  if (controller->getCollisionMdl())
  {
    getEntity()->publish<const MatNd*>("RenderLines",
                                       controller->getCollisionMdl()->cp);
  }

  if (this->renderSolid == false)
  {
    getEntity()->publish<std::string,std::string>("RenderCommand", "IK",
                                                  "setGhostMode");
    this->renderSolid = true;
  }
}

void IKComponent::setEnableSpeedAccelerationLimit(bool enable)
{
  this->applySpeedAndAccLimits = enable;
}

void IKComponent::setSpeedLimitCheck(bool enable)
{
  this->speedLimitCheck = enable;
}

void IKComponent::setJointLimitCheck(bool enable)
{
  this->jointLimitCheck = enable;
}

void IKComponent::setCollisionCheck(bool enable)
{
  this->collisionCheck = enable;
}

void IKComponent::print() const
{
  RcsCollisionModel_fprintCollisions(stdout, controller->getCollisionMdl(), 1000.0);
  RcsGraph_fprintModelState(stdout, controller->getGraph(),
                            controller->getGraph()->q, NULL, 0);
}

void IKComponent::onSetBlending(double value)
{
  this->blending = value;
}

void IKComponent::onSetPhase(double value)
{
  this->phase = value;
}

void IKComponent::onLinkGenericBody(int gBodyId, std::string bodyName)
{
  RLOG(0, "Linking GenericBody %d against %s", gBodyId, bodyName.c_str());
  RcsBody* lb = RcsGraph_linkGenericBody(controller->getGraph(),
                                         gBodyId, bodyName.c_str());
  if (!lb)
  {
    RLOG(0, "Failed to link GenericBody %d against %s", gBodyId, bodyName.c_str());
  }
}

void IKComponent::onTaskVectorChange(std::vector<std::string> taskVec,
                                     std::vector<std::string> channels)
{
  RLOG(1, "Changing task vector");

  // We need to delete this array since it is used for determining task
  // switches. If it is NULL, it will be re-initialized properly with
  // the next incoming activation command.
  MatNd_destroy(this->a_prev);
  this->a_prev = NULL;
}

void IKComponent::setAlpha(double value)
{
  this->alphaMax = value;
}

double IKComponent::getAlpha() const
{
  return this->alphaMax;
}

void IKComponent::setLambda(double value)
{
  this->lambda = value;
}

double IKComponent::getLambda() const
{
  return this->lambda;
}

void IKComponent::renderSolidModel()
{
  this->renderSolid = true;
}


}   // namespace aff
