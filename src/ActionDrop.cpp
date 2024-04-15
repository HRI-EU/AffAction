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

#include "ActionDrop.h"
#include "ActionFactory.h"
#include "ActivationSet.h"
#include "VectorConstraint.h"
#include <ConnectBodyConstraint.h>

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#define fingersOpen   (0.01)


namespace aff
{
REGISTER_ACTION(ActionDrop, "drop");

ActionDrop::ActionDrop(const ActionScene& domain,
                       const RcsGraph* graph,
                       std::vector<std::string> params)
{
  HTr_setIdentity(&dropTransform);

  if (params.empty())
  {
    throw ActionException(ActionException::ParamNotFound, "No object to drop given");
  }

  this->objectToDrop = params[0];


  // Detect hand that holds the object
  std::vector<const AffordanceEntity*> dropEntities = domain.getAffordanceEntities(objectToDrop);

  if (dropEntities.empty())
  {
    throw ActionException(ActionException::ParamNotFound, "Entity for " + objectToDrop  + " not found in scene");
  }

  auto gPair = domain.getGraspingHand(graph, dropEntities);
  const Manipulator* graspingHand = std::get<0>(gPair);
  const AffordanceEntity* dropEntity = std::get<1>(gPair);

  if (!graspingHand)
  {
    throw ActionException(ActionException::KinematicallyImpossible, "No hand is grasping the object " + objectToDrop);
  }

  objectToDrop = dropEntity->bdyName;

  for (auto& f : graspingHand->fingerJoints)
  {
    this->fingerJoints += f;
    this->fingerJoints += " ";
  }

  // Find the frame with which the object is being grasped.
  auto graspCapability = graspingHand->getGraspingCapability(graph, dropEntity);
  this->graspFrameName = graspCapability ? graspCapability->frame : std::string();

  this->taskInclination = graspingHand->name + "-InclinationY";
  this->taskPosition = graspingHand->name + "-XYZ";
  this->taskFingers = graspingHand->name + "_fingers";

  // We only allow dropping objects on "Supportables".
  const AffordanceEntity* surface = NULL;

  if (params.size()>=2)   // The second parameter is the surface to drop the object on
  {
    surface = domain.getAffordanceEntity(params[1]);
  }
  else   // Only one parameter: Raycast down to force-find a surface to drop on.
  {
    std::string errMsg;
    surface = raycastSurface(domain, dropEntity, graph, &dropTransform, errMsg);

    if (!surface)
    {
      throw ActionException(errMsg, ActionException::KinematicallyImpossible);
      //throw ActionException("ERROR: Can't drop the " + objectToDrop +
      //    " REASON: There is nothing under the " +
      //    objectToDrop + " where I can put it on",
      //    ActionException::KinematicallyImpossible);
    }

    this->surfaceName = surface->bdyName;
  }

  if (!surface)
  {
    throw ActionException(ActionException::KinematicallyImpossible, "No surface entity to drop on found");
  }

  // Capability is "graspCapability", all support surfaces are stored in
  // supportSurfaces. We create an affordance-capability map that will be
  // sorted in order to find out the best support surface to drop the object
  // on. It's going to be the closest one.
  auto supportSurfaces = getAffordances<Supportable>(surface);
  for (Affordance* affordance : supportSurfaces)
  {
    affordanceMap.emplace_back(std::make_tuple(affordance, (Capability*)graspCapability));
  }

  if (affordanceMap.empty())
  {
    throw ActionException(ActionException::ParamNotFound, "Cannot drop object on " + surface->name);
  }

  // We have one or several matches and sort them according to their cost. Lower
  // costs pairs are at the beginning.
  sort(graph, affordanceMap, 1.0, 0.0);

  // Initialize with the best solution.
  bool successInit = initialize(domain, graph, 0);

  if (!successInit)
  {
    throw ActionException(ActionException::ParamInvalid, "Failed to initialize best solution");
  }

}

ActionDrop::~ActionDrop()
{
}

bool ActionDrop::initialize(const ActionScene& domain,
                            const RcsGraph* graph,
                            size_t solutionRank)
{
  if (solutionRank >= getNumSolutions())
  {
    return false;
  }

  // Get the frame of the affordance from the second capability
  Affordance* winningAff = std::get<0>(affordanceMap[solutionRank]);

  // This is true if the surface has not been detected by raycasting
  if (surfaceName.empty())
  {
    const RcsBody* dropFrame = RcsGraph_getBodyByName(graph, winningAff->frame.c_str());
    HTr_copy(&dropTransform, &dropFrame->A_BI);
  }

  this->surfaceName = winningAff->frame;

  return true;
}

std::vector<std::string> ActionDrop::createTasksXML() const
{
  std::vector<std::string> tasks;
  std::string xmlTask;

  // Task to incline the hand so that the held object does not roll over
  // the fingers. We incline the y-axis of a grasp frame.
  if (!graspFrameName.empty())
  {
    xmlTask = "<Task name=\"" + taskInclination + "\" controlVariable=\"Inclination\" " +
              "effector=\"" + graspFrameName + "\" axisDirection=\"Y\" />";
    tasks.push_back(xmlTask);

    xmlTask = "<Task name=\"" + taskPosition + "\" controlVariable=\"XYZ\" " +
              "effector=\"" + graspFrameName + "\" />";
    tasks.push_back(xmlTask);
  }

  // Fingers
  xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerJoints + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionDrop::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  const double duration = t_end - t_start;
  auto a1 = std::make_shared<tropic::ActivationSet>();

  if (!graspFrameName.empty())
  {
    // Keep hand position where it is
    a1->addActivation(t_start, true, 0.5, taskPosition);
    a1->addActivation(t_end, false, 0.5, taskPosition);

    // Incline hand to have fingers out of the way
    a1->addActivation(t_start, true, 0.5, taskInclination);
    a1->addActivation(t_end, false, 0.5, taskInclination);
    a1->add(t_start + 0.5 * duration, M_PI_2, 0.0, 0.0, 7, taskInclination + " 0");
  }

  // Open fingers. The fingers are not affected by any null space gradient,
  // therefore ther angles don't change without activation. We use this
  // to activate them only for the opening phase.
  a1->addActivation(t_start, true, 0.5, taskFingers);
  a1->addActivation(t_end, false, 0.5, taskFingers);
  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, std::vector<double> {fingersOpen, fingersOpen, fingersOpen}, taskFingers));

  // After we drop the object, we change its parent to be the surface it
  std::shared_ptr<tropic::ConnectBodyConstraint> cbc(new tropic::ConnectBodyConstraint(t_end, objectToDrop, surfaceName));
  cbc->setConnectTransform(HTr_identity());
  a1->add(cbc);

  return a1;
}

double ActionDrop::getDurationHint() const
{
  return 3.0;
}

size_t ActionDrop::getNumSolutions() const
{
  return affordanceMap.size();
}

std::vector<std::string> ActionDrop::getManipulators() const
{
  return std::vector<std::string>();
}

std::unique_ptr<ActionBase> ActionDrop::clone() const
{
  return std::make_unique<ActionDrop>(*this);
}


}   // namespace aff
