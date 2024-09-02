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

#include "PhysicsComponent.h"
#include "PhysicsFactory.h"

#include <Rcs_body.h>
#include <Rcs_joint.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_timer.h>
#include <Rcs_math.h>



namespace aff
{

PhysicsComponent::PhysicsComponent(EntityBase* parent,
                                   const RcsGraph* graph_,
                                   std::string engine,
                                   std::string cfgFile) :
  ComponentBase(parent),
  dtSim(0.0),
  tStart(Timer_getSystemTime()),
  enableCommands(false),
  perceptionFrozen(false)
{
  auto physics = Rcs::PhysicsFactory::create(engine.c_str(), graph_, cfgFile.c_str());
  RCHECK(physics);
  this->sim = std::unique_ptr<Rcs::PhysicsBase>(physics);

  subscribe("EnableCommands", &PhysicsComponent::onEnableCommands);
  subscribe<const RcsGraph*>("InitFromState", &PhysicsComponent::onInitFromState);
  subscribe("SetJointCommand", &PhysicsComponent::setPositionCommand);
  subscribe<RcsGraph*>("UpdateGraph", &PhysicsComponent::onUpdateGraph);
  subscribe("PostUpdateGraph", &PhysicsComponent::onPostUpdateGraph);
  subscribe("ResetRigidBodies", &PhysicsComponent::onResetRigidBodies);
  subscribe("FreezePerception", &PhysicsComponent::onFreezePerception);
}

PhysicsComponent::~PhysicsComponent()
{
}

void PhysicsComponent::onUpdateGraph(RcsGraph* graph)
{
  const double dt = getEntity()->getDt();
  if (dt <  0.0)
  {
    RLOG(1, "Simulation time step not >0: %g", dt);
    return;
  }

  double tmp = Timer_getSystemTime();
  sim->simulate(dt, graph, NULL, NULL, this->enableCommands);
  this->dtSim = Timer_getSystemTime() - tmp;
}

void PhysicsComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  if (perceptionFrozen)
  {
    return;
  }

  for (unsigned int i = 0; i < current->nBodies; ++i)
  {
    const RcsBody* currBdy = &current->bodies[i];

    // Ignore all bodies that are not part of the simulation
    if (!currBdy->physicsSim)
    {
      continue;
    }

    // Ignore all bodies that don't have 6dof
    if (!currBdy->rigid_body_joints)
    {
      continue;
    }


    // The body in the desired graph might have a different connectivity, for
    // instance if its parent changed due to grasping. We consider this here.

    // The bodie's absolute transform from physics.
    const HTr* new_A_BI_body = &currBdy->A_BI;

    // The parent's transform from the internal model (desired)
    const RcsBody* desBdy = &desired->bodies[i];
    const HTr* new_A_BI_parent = (desBdy->parentId==-1) ? HTr_identity() : &desired->bodies[desBdy->parentId].A_BI;

    // It is certain that the body has 6 dof, therefore we do a hard check.
    int qidx = RcsBody_getJointIndex(current, currBdy);
    RCHECK(qidx != -1);
    RcsGraph_relativeRigidBodyDoFs(desired,
                                   &desired->bodies[i],
                                   new_A_BI_body,
                                   new_A_BI_parent,
                                   desired->q->ele + qidx);
  }

}

void PhysicsComponent::setPositionCommand(const MatNd* q_des)
{
  if (!this->enableCommands)
  {
    return;
  }

  MatNd* zeroVec = MatNd_create(q_des->m, 1);
  sim->setControlInput(q_des, zeroVec, zeroVec);
  MatNd_destroy(zeroVec);
}

int PhysicsComponent::sprint(char* str, size_t size) const
{
  return snprintf(str, size, "Simulation time: %.3f (%.3f)\nStep took %.1f msec\n",
                  sim->time(), Timer_getSystemTime()-this->tStart, dtSim*1.0e3);
}

Rcs::PhysicsBase* PhysicsComponent::getPhysics()
{
  return this->sim.get();
}

void PhysicsComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(1, "PhysicsComponent::onInitFromState()");
  RcsGraph_setState(sim->getGraph(), target->q, target->q_dot);
  sim->reset();
}

void PhysicsComponent::onEnableCommands()
{
  RLOG(1, "PhysicsComponent::onEnableCommands()");
  this->enableCommands = true;
}

void PhysicsComponent::onResetRigidBodies()
{
  RLOG(1, "PhysicsComponent::onEnableCommands()");
  sim->resetRigidBodies();
}

void PhysicsComponent::onFreezePerception(bool freeze)
{
  this->perceptionFrozen = freeze;
}

}   // namespace aff
