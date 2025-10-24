/*******************************************************************************

  Copyright (c) by Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "JointGuiComponent.h"

#include <JointWidget.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_graphParser.h>



namespace aff
{

class JointUpdateCallback : public Rcs::JointWidget::JointChangeCallback
{
public:
  JointUpdateCallback(JointGuiComponent* gui_) : gui(gui_)
  {
  }

  virtual void callback()
  {
    gui->guiCallback();
  };

  JointGuiComponent* gui;
};


JointGuiComponent::JointGuiComponent(EntityBase* parent, const RcsGraph* g,
                                     double tmc, double vmax) :
  ComponentBase(parent)
{
  RCHECK(tmc>=0.0);
  RCHECK(vmax>=0.0);
  this->guiGraph = RcsGraph_clone(g);
  this->q_des = MatNd_clone(g->q);
  this->q_des_filt = MatNd_clone(g->q);
  this->q_curr = MatNd_clone(g->q);

  this->filt = new Rcs::RampFilterND(q_curr->ele, tmc, vmax, parent->getDt(), g->dof);
  pthread_mutex_init(&this->mtx, NULL);

  subscribe("Start", &JointGuiComponent::onStart);
  subscribe("Stop", &JointGuiComponent::onStop);
  subscribe<const RcsGraph*>("InitFromState", &JointGuiComponent::onInitialize);
  subscribe("ComputeKinematics", &JointGuiComponent::onFilterAndUpdateGui);
  subscribe("EmergencyStop", &JointGuiComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &JointGuiComponent::onEmergencyRecover);
  subscribe("SetModelStatePose", &JointGuiComponent::onGoalPose);
  subscribe("Render", &JointGuiComponent::onRender);
}

JointGuiComponent::~JointGuiComponent()
{
  unsubscribe();

  delete this->jGui;
  delete this->filt;

  RcsGraph_destroy(this->guiGraph);
  MatNd_destroy(this->q_des);
  MatNd_destroy(this->q_des_filt);
  MatNd_destroy(this->q_curr);

  pthread_mutex_destroy(&this->mtx);
}

void JointGuiComponent::onStart()
{
  jGui = new Rcs::JointGui(this->guiGraph, &this->mtx, this->q_des, this->q_curr);
  RLOG(1, "Start::start()");
  MatNd_copy(this->q_des, guiGraph->q);

  JointUpdateCallback* jcb = new JointUpdateCallback(this);
  Rcs::JointWidget* jw = static_cast<Rcs::JointWidget*>(jGui->getWidget());
  jw->registerCallback(jcb);
}

void JointGuiComponent::guiCallback()
{
  pthread_mutex_lock(&this->mtx);
  filt->setTarget(this->q_des->ele);
  pthread_mutex_unlock(&this->mtx);
}

void JointGuiComponent::onGoalPose(std::string goalPose)
{
  MatNd* q_goal = MatNd_clone(guiGraph->q);
  bool ok = RcsGraph_getModelStateFromXML(q_goal, guiGraph, goalPose.c_str(), -1);

  if (ok)
  {
    setGoalPose(q_goal);
  }

  MatNd_destroy(q_goal);
}

void JointGuiComponent::setGoalPose(const MatNd* q_goal)
{
  if ((q_goal->m!=this->q_des->m) || (q_goal->n!=this->q_des->n))
  {
    RLOG(1, "Mismatch in setGoalPose(): q_goal is [%d x %d], but state vector "
         "should be [%d x %d] - skipping q_goal", q_goal->m, q_goal->n,
         this->q_des->m, this->q_des->n);
    return;
  }

  pthread_mutex_lock(&this->mtx);
  filt->setTarget(q_goal->ele);
  Rcs::JointWidget* jw = static_cast<Rcs::JointWidget*>(jGui->getWidget());
  if (jw)
  {
    jw->reset(q_goal);
  }
  pthread_mutex_unlock(&this->mtx);
}

const RcsGraph* JointGuiComponent::getGraph() const
{
  return this->guiGraph;
}

void JointGuiComponent::onFilterAndUpdateGui(RcsGraph* from)
{
  RLOG(5, "ComputeKinematics::setState()");
  pthread_mutex_lock(&this->mtx);
  MatNd_copy(this->q_curr, from->q);
  filt->iterate();
  filt->getPosition(q_des_filt->ele);
  pthread_mutex_unlock(&this->mtx);
}

void JointGuiComponent::onEmergencyStop()
{
  RLOG(1, "EmergencyStop");
  MatNd_copy(guiGraph->q, this->q_curr);
  onInitialize(this->guiGraph);
}

void JointGuiComponent::onEmergencyRecover()
{
  RLOG(1, "EmergencyRecover");
  MatNd_copy(guiGraph->q, this->q_curr);
  onInitialize(this->guiGraph);
}

void JointGuiComponent::onInitialize(const RcsGraph* target)
{
  RLOG(1, "Initialize::initialize()");
  pthread_mutex_lock(&this->mtx);

  MatNd_copy(this->q_curr, target->q);
  MatNd_copy(this->q_des, target->q);
  MatNd_copy(this->q_des_filt, target->q);
  filt->init(target->q->ele);

  Rcs::JointWidget* jw = static_cast<Rcs::JointWidget*>(jGui->getWidget());
  jw->reset(target->q);

  pthread_mutex_unlock(&this->mtx);
}

void JointGuiComponent::onStop()
{
  delete jGui;
  jGui = NULL;
}

const MatNd* JointGuiComponent::getJointCommandPtr() const
{
  return this->q_des_filt;
}

void JointGuiComponent::onRender()
{
  RcsGraph_setState(this->guiGraph, this->q_des_filt, NULL);
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "Gui", this->guiGraph);
}

}   // namespace
