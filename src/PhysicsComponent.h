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

#ifndef RCS_PHYSICSCOMPONENT_H
#define RCS_PHYSICSCOMPONENT_H

#include "ComponentBase.h"

#include <PhysicsBase.h>

#include <memory>



namespace aff
{
/*! \brief Physics simulation class. Given some control input, this class
 *         computes the physics response according to the underlying physics
 *         model (coming from the passed graph). The class creates a physics
 *         simulation instance by name internally, and does not refer to the
 *         passed graph from the constructor. If no physics engine with the
 *         given name is found, the class behaves purely kinematic and uses
 *         the last set joint command vector as current state. Incase there's a
 *         physics engine instantiated, the class can simulate either in a
 *         sequential computation strategy (simulate() will be called upon each
 *         UpdateGraph event), or parallel in it's own thread. See the
 *         constructor for details. This class has been written to provide
 *         interfaces as similar as possible to real robot hardware, so that
 *         it can be used for testing new concepts without experiments.
 *
 *         The class publishes no events.
 *
 *         The class subscribes to the following events:
 *         - SetRandomPose: Test event, sets a randomized pose. This allows to
 *                          test emergency recovery and other things.
 *         - EnableCommands: Enables accepting commands through the
 *                           SetJointCommand event. After construction, the
 *                           class does not accept any commands.
 *         - InitFromState: Resets the simulation with the passed graph.
 *         - SetJointCommand: Sets the passed vector to the next joint position
 *                            command for the next simulation step.
 *         - UpdateGraph: Copies the q, q_dot vectors and all sensory data into
 *                        the passed graph.
 *
 *         The following events are only subscribed if a physics engine is
 *         instantiated:
 *         - Start: Starts the simulation thread (if threading is requested)
 *         - Stop: Stops the simulation thread (if threading is requested)
 *         - EmergencyStop: Sets the desired position to the current ones, the
 *                         desired velovities to zero, and the sTop flag. This
 *                         results in ignoring any new incoming commands through
 *                         the SetJointCommand event.
 *         - EmergencyRecover: Releases the eStop flag so that new incoming
 *                            commands (through the SetJointCommand event) are
 *                            accepted.
 *         - SetObjectActivation: Activates or deactivates a rigid body. A
 *                               deactivation leads to the body not taking part
 *                               in the dynamics responses any more.
 */
class PhysicsComponent : public ComponentBase
{
public:

  PhysicsComponent(EntityBase* parent,
                   const RcsGraph* graph,
                   std::string engine="Bullet",
                   std::string physicsCfgFile=std::string());

  virtual ~PhysicsComponent();

  virtual void setPositionCommand(const MatNd* q_des);
  virtual int sprint(char* str, size_t size) const;
  Rcs::PhysicsBase* getPhysics();


private:

  void onUpdateGraph(RcsGraph* graph);
  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);
  void onInitFromState(const RcsGraph* target);
  void onEnableCommands();
  void onResetRigidBodies();
  void onFreezePerception(bool freeze);

  double dtSim, tStart;
  bool enableCommands;
  bool perceptionFrozen;
  std::unique_ptr<Rcs::PhysicsBase> sim;

  PhysicsComponent(const PhysicsComponent&);
  PhysicsComponent& operator=(const PhysicsComponent&);
};

}

#endif   // RCS_PHYSICSCOMPONENT_H
