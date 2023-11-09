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

#include "ActionDoor.h"
#include "ActionFactory.h"
#include "ActivationSet.h"
#include "PositionConstraint.h"
#include "PolarConstraint.h"
#include "EulerConstraint.h"
#include "JointWeightConstraint.h"

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#include <limits>


namespace aff
{
REGISTER_ACTION(ActionDoor, "open_door");
REGISTER_ACTION(ActionDoor, "id_635a8d817388ce8eb5e5e32f");

REGISTER_ACTION(ActionDoor, "close_door");
REGISTER_ACTION(ActionDoor, "id_635a8e497388ce8eb5e5e330");



ActionDoor::ActionDoor(const ActionScene& domain,
                       const RcsGraph* graph,
                       std::vector<std::string> params)
{

  std::string errorMsg = "ERROR ";


  objectToOpenName = params[0];
  const AffordanceEntity* objectToOpen = domain.getAffordanceEntity(objectToOpenName);

  if (!objectToOpen)
  {
    throw ActionException(errorMsg + "REASON: The " + objectToOpenName +
                          " to open or close is unknown. SUGGESTION: Use an object name that is defined in the environment", ActionException::ParamNotFound);
  }

  auto hingeables = getAffordances<Hingeable>(objectToOpen);

  if (hingeables.empty())
  {
    throw ActionException(errorMsg + "REASON: the " + objectToOpenName +
                          " is not openable or closable. DEVELOPER: no Hingeable affordances found",
                          ActionException::ParamNotFound);
  }

  objectToOpenName = objectToOpen->bdyName;



  // Manipulator business
  std::string manipulator;
  if (params.size()>=2)
  {
    manipulator = params[1];
  }




  std::vector<const Manipulator*> freeManipulators;

  if (manipulator.empty())
  {
    freeManipulators = domain.getFreeManipulators(graph);
  }
  else
  {
    const Manipulator* hand = domain.getManipulator(manipulator);

    // If a manipulator has been passed but cannot be found, we consider this as an error.
    if (!hand)
    {
      std::string reasonMsg = " REASON: Can't find a hand with the name " + manipulator;
      std::string suggestionMsg = " SUGGESTION: Use a hand name that is defined in the environment";
      throw ActionException(errorMsg + reasonMsg + suggestionMsg, ActionException::ParamNotFound);
    }

    if (!hand->isEmpty(graph))
    {
      throw ActionException(errorMsg + "REASON: The hand " + manipulator +
                            " is already holding something. SUGGESTION: Free your hand before performing this command, or use another hand",
                            ActionException::KinematicallyImpossible);
    }

    freeManipulators.push_back(hand);
  }

  if (freeManipulators.empty())
  {
    std::string reasonMsg = " REASON: All hands are already full";
    std::string suggestionMsg = " SUGGESTION: Free one or all your hands before performing this command";
    throw ActionException(errorMsg + reasonMsg + suggestionMsg, ActionException::KinematicallyImpossible);
  }

  // From here on we have one or several manipulators that can be tentatively used
  // to get the object. We now go through each of them and compile a list of pairs
  // matching each manipulator's capability to the object affordances.
  for (const Manipulator* hand : freeManipulators)
  {
    auto graspMap_i = match<Hingeable>(objectToOpen, hand);
    affordanceMap.insert(affordanceMap.end(), graspMap_i.begin(), graspMap_i.end());
  }

  // If this vector is empty, there's no capability that can get the object in any way.
  // How sad that we have to stop here.
  if (affordanceMap.empty())
  {
    throw ActionException(errorMsg + "REASON: Don't have the capability to grasp the handle of the " + objectToOpenName,
                          ActionException::KinematicallyImpossible);
  }

  // We have one or several matches and sort them according to their cost. Lower
  // costs pairs are at the beginning.
  sort(graph, affordanceMap, 1.0, 0.0);

  // Initialize with the best solution.
  bool successInit = initialize(domain, graph, 0);
  RCHECK(successInit);
}

ActionDoor::~ActionDoor()
{
}

bool ActionDoor::initialize(const ActionScene& domain,
                            const RcsGraph* graph,
                            size_t solutionRank)
{
  if (solutionRank >= getNumSolutions())
  {
    return false;
  }

  std::string errorMsg = "ERROR ";

  // Get the frame of the capability from the first entry
  const Capability* winningCap = std::get<1>(affordanceMap[solutionRank]);
  handName = winningCap->frame;

  // Get the frame of the affordance from the second capability
  Affordance* winningAff = std::get<0>(affordanceMap[solutionRank]);
  doorHandle = winningAff->frame;

  if (dynamic_cast<PowerGraspable*>(winningAff))
  {
    graspOrientation = "Polar";
  }
  else
  {
    graspOrientation = "Euler";
  }

  RLOG_CPP(1, "Capability frame: " << handName << " affordance frame: "
           << doorHandle);

  const Manipulator* hand = domain.getManipulator(winningCap);
  usedManipulators.clear();
  usedManipulators.push_back(hand->name);

  // Assemble finger task name
  fingerJoints.clear();
  for (auto& f : hand->fingerJoints)
  {
    fingerJoints += f;
    fingerJoints += " ";
  }

  // Determine the hinge joint: The door handle parent's joint
  // \todo(MG) Put into Affordance check
  const RcsBody* doorHandleBdy = RcsGraph_getBodyByName(graph, doorHandle.c_str());
  if (!doorHandleBdy)
  {
    std::string errMsg = errorMsg + "REASON: The door handle " + doorHandle + " is unknown.";
    RFATAL("%s", errMsg.c_str());
    throw ActionException(errMsg, ActionException::ParamNotFound);
  }

  const RcsBody* handleParent = RCSBODY_BY_ID(graph, doorHandleBdy->parentId);
  if (!handleParent)
  {
    std::string errMsg = errorMsg + "REASON: door handle (" + doorHandle + ") is not part of the " + objectToOpenName;
    RFATAL("%s", errMsg.c_str());
    throw ActionException(errMsg, ActionException::ParamNotFound);
  }

  const RcsJoint* doorHingeJnt = RCSJOINT_BY_ID(graph, handleParent->jntId);
  if (!doorHingeJnt)
  {
    std::string errMsg = errorMsg + "REASON: Something is wrong with the door handle. DEVELOPER: Not found: joint of door handle's parent" + std::string(handleParent->name);
    RFATAL("%s", errMsg.c_str());
    throw ActionException(errMsg, ActionException::ParamNotFound);
  }
  if (doorHingeJnt->constrained)
  {
    std::string errMsg = errorMsg + "REASON: Something is wrong with the door handle. DEVELOPER: Joint of door handle's parent" + std::string(handleParent->name) + " is constrained";
    RFATAL("%s", errMsg.c_str());
    throw ActionException(errMsg, ActionException::ParamInvalid);
  }

  doorHingeJoint = doorHingeJnt->name;

  // Task naming
  this->taskRelPos = handName + "-" + doorHandle + "-XYZ";
  this->taskRelOri = handName + "-" + doorHandle + "-ABC";
  this->taskHinge = doorHingeJoint + "-Joint";

  explanation = "I'm opening the " + std::string(handleParent->name);

  return true;
}

std::vector<std::string> ActionDoor::createTasksXML() const
{
  std::vector<std::string> tasks;

  std::string xmlTask;
  xmlTask = "<Task name=\"" + taskRelPos + "\" " + "controlVariable=\"XYZ\" " +
            "effector=\"" + handName + "\" " + "refBdy=\"" + doorHandle + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskRelOri + "\" " + "controlVariable=\"ABC\" " +
            "effector=\"" + handName + "\" " + "refBdy=\"" + doorHandle + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskHinge + "\" " + "controlVariable=\"Joint\" " +
            "jnt=\"" + doorHingeJoint + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionDoor::createTrajectory(double t_start, double t_end) const
{
  double openAngle = 0.0;
  if (getName() == "open_door" || getName() == "id_635a8d817388ce8eb5e5e32f")
  {
    openAngle = -RCS_DEG2RAD(60.0);
  }
  const double afterTime = 0.5;
  const double duration = t_end - t_start;  // 10sec
  const double t_mid = t_start + 0.5 * duration;   // Grasping the handle
  const double preGraspDist = 0.1;

  auto a1 = std::make_shared<tropic::ActivationSet>();

  // All tasks are active throughout the period
  a1->addActivation(t_start, true, 0.5, taskRelPos);
  a1->addActivation(t_end + afterTime, false, 0.5, taskRelPos);
  a1->addActivation(t_start, true, 0.5, taskRelOri);
  a1->addActivation(t_end + afterTime, false, 0.5, taskRelOri);
  a1->addActivation(t_mid, true, 0.5, taskHinge);
  a1->addActivation(t_end + afterTime, false, 0.5, taskHinge);

  if (getOptimDim()==3)
  {
    const double t_optim = t_start + 0.25 * duration;
    auto oState = getOptimState();
    RLOG(1, "Adding optimization constraint [%.4f %.4f %.4f] at t=%f",
         oState[0], oState[1], oState[2], t_optim);
    a1->add(std::make_shared<tropic::PositionConstraint>(t_optim, oState.data(), taskRelPos, 1));
  }

  // Modulate motion ability of door
  a1->add(std::make_shared<tropic::JointWeightConstraint>(t_start, doorHingeJoint, 0.0));
  a1->add(std::make_shared<tropic::JointWeightConstraint>(t_mid, doorHingeJoint, 1.0));
  a1->add(std::make_shared<tropic::JointWeightConstraint>(t_end, doorHingeJoint, 0.0));

  // Pre-grasp door handle
  a1->add(std::make_shared<tropic::PositionConstraint>(t_start + 0.3*duration, preGraspDist, 0.0, 0.0, taskRelPos, 1));

  // Grasp door handle
  a1->add(std::make_shared<tropic::PositionConstraint>(t_mid, 0.0, 0.0, 0.0, taskRelPos));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_mid, 0.0, 0.0, M_PI, taskRelOri));

  // Pull door handle open
  double hingeAngle = openAngle;
  a1->add(t_end, hingeAngle, 0.0, 0.0, 7, taskHinge + " 0");

  return a1;
}

double ActionDoor::getDurationHint() const
{
  return 20.0;
}

std::string ActionDoor::explain() const
{
  return explanation;
}

std::vector<std::string> ActionDoor::getManipulators() const
{
  return usedManipulators;
}

std::vector<double> ActionDoor::getInitOptimState(tropic::TrajectoryControllerBase* tc,
                                                  double duration) const
{
  const double t_optim = 0.25 *duration;
  double pos[3];
  tc->getTrajectory(taskRelPos)->getPosition(t_optim, pos);
  std::vector<double> o(pos, pos+3);
  return o;
}

std::unique_ptr<ActionBase> ActionDoor::clone() const {
  return std::make_unique<ActionDoor>(*this);
}

}   // namespace aff
