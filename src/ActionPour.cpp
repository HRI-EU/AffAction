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

#include "ActionPour.h"
#include "ActionFactory.h"
#include "ActivationSet.h"
#include "PositionConstraint.h"
#include "PolarConstraint.h"
#include "ConnectBodyConstraint.h"
#include "VectorConstraint.h"

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#include <algorithm>



namespace aff
{
REGISTER_ACTION(ActionPour, "pour");
REGISTER_ACTION(ActionPour, "id_635640f302a8c9f3ffa00811");

ActionPour::ActionPour(const ActionScene& domain,
                       const RcsGraph* graph,
                       std::vector<std::string> params) :
  glasInHand(false), bottleInRightHand(true), pouringVolume(0.1)
{
  if (params.size()<2)
  {
    throw ActionException("ERROR REASON: Action has less than two parameters. At least two parameters are needed:"
                          "The container to pour from, and the container to pour into. "
                          "SUGGESTION: Correct action command.", ActionException::ParamInvalid);
  }

  defaultDuration = 20.0;
  auto it = std::find(params.begin(), params.end(), "duration");
  if (it != params.end())
  {
    defaultDuration = std::stod(*(it+1));
    params.erase(it+1);
    params.erase(it);
  }

  const std::string& objectToPourFrom = params[0];
  const std::string& objectToPourInto = params[1];
  double amountToPour = pouringVolume;
  if (params.size()>2)
  {
    amountToPour = std::stod(params[2]);
  }

  if (amountToPour < 0.0)
  {
    throw ActionException("ERROR REASON: The amount to pour has a negative value. It must be equal or larger than zero."
                          "SUGGESTION: Pour only liquid amounts with positive volume.", ActionException::ParamInvalid);
  }

  init(domain, graph, objectToPourFrom, objectToPourInto, amountToPour, "base_footprint");
}

void ActionPour::init(const ActionScene& domain,
                      const RcsGraph* graph,
                      const std::string& objectToPourFrom,
                      const std::string& objToPourInto,
                      double amountToPour,
                      const std::string& roboBase)
{
  std::string errorMsg = "ERROR ";

  this->pouringVolume = amountToPour;

  const AffordanceEntity* pourFromAff = domain.getAffordanceEntity(objectToPourFrom);

  // Didn't find object to pour from
  if (!pourFromAff)
  {
    throw ActionException(errorMsg + "REASON: The " + objectToPourFrom +
                          " to pour from is unknown. SUGGESTION: Use an object name that is defined in the environment",
                          ActionException::ParamNotFound);
  }

  auto openings = getAffordances<Pourable>(pourFromAff);

  // Both bottle need to have an opening
  if (openings.empty())
  {
    throw ActionException(errorMsg + "REASON: The " + objectToPourFrom + " is not a container that can be poured from."
                          "SUGGESTION: Choose another object to pour from.",
                          ActionException::ParamNotFound);
  }

  const AffordanceEntity* pourToAff = domain.getAffordanceEntity(objToPourInto);

  if (!pourToAff)
  {
    throw ActionException(errorMsg + "REASON: The " + objToPourInto +
                          " to pour into is unknown. SUGGESTION: Use an object name that is defined in the environment",
                          ActionException::ParamNotFound);
  }

  auto containers = getAffordances<Containable>(pourToAff);

  if (containers.empty())
  {
    throw ActionException(errorMsg + "REASON: The " + objToPourInto +
                          " to pour into is not a container that can be poured into. "
                          "SUGGESTION: Choose another object to pour into.", ActionException::ParamNotFound);
  }

  // The bottle object must be in a hand
  const Manipulator* pouringHand = domain.getGraspingHand(graph, pourFromAff);
  if (!pouringHand)
  {
    throw ActionException(errorMsg + "REASON: The " + objectToPourFrom + " to pour from is not held in a hand."
                          " SUGGESTION: First get the object " + objectToPourFrom + " before performing this action",
                          ActionException::ParamNotFound);
  }

  usedManipulators.push_back(pouringHand->name);

  // We make a local copy of the body strings, since they might be resolved
  // into different names when being a GenericBody
  RCHECK_MSG(openings.size()==1, "Can't deal with more than 1 opening");
  RCHECK_MSG(containers.size()==1, "Can't deal with more than 1 container");
  this->bottle = openings[0]->frame;
  this->glas = containers[0]->frame;
  this->roboBaseFrame = roboBase;

  // We go through the following look-ups to resolve the names of possible
  // generic bodies.
  const RcsBody* bottleBdy = RcsGraph_getBodyByName(graph, bottle.c_str());
  RCHECK(bottleBdy);
  this->bottle = std::string(bottleBdy->name);

  const RcsBody* glasBdy = RcsGraph_getBodyByName(graph, glas.c_str());
  RCHECK(glasBdy);
  this->glas = std::string(glasBdy->name);

  const RcsBody* roboBaseBdy = RcsGraph_getBodyByName(graph, roboBaseFrame.c_str());
  RCHECK(roboBaseBdy);
  this->roboBaseFrame = std::string(roboBaseBdy->name);

  // Transition the liquid from bottle to glas.
  performLiquidTransition(pourFromAff, pourToAff);

  // Collect solid items from the pouring affordance. We don't care how it got
  // there, therefore we don't go through the Containables. But ideally, the
  // "particles" should be only children of Containables. Otherwise, also
  // objects that stand on the pouring object will move into the goal container.
  // \todo(MG): Think about desired behavior
  auto containedSolids = domain.getAllChildren(graph, pourFromAff);
  for (auto particle : containedSolids)
  {
    particlesToPour.push_back(particle->bdyName);
  }


  // Task naming
  this->taskRelPos = bottle + "-" + glas + "-" + roboBase + "-XYZ";
  this->taskBottleOri = bottle + "-POLAR";
  this->taskGlasOri = glas + "-POLAR";
  this->taskGlasPosX = glas + "-X";

  // Determine if pouring from left to right or from right to left.
  // The robot base frame's y-axis points to left. We transform the
  // bottle and gles into the robot's base frame and check which
  // y-component is larger. That's the left body.
  const HTr* A_RI = &roboBaseBdy->A_BI;

  double R_r_glas[3];   // Glas origin in robot base frame
  Vec3d_invTransform(R_r_glas, A_RI, glasBdy->A_BI.org);

  double R_r_bottle[3];   // Bottle origin in robot base frame
  Vec3d_invTransform(R_r_bottle, A_RI, bottleBdy->A_BI.org);

  if (R_r_bottle[1] < R_r_glas[1])   // Pouring with right hand
  {
    bottleInRightHand = true;
    RLOG(0, "Pouring with right hand");
  }
  else   // Pouring with left hand
  {
    RLOG(0, "Pouring with left hand");
    bottleInRightHand = false;
  }

  // Determine if glas is held in the hand or standing around. We use this
  // intormation for trajectory generation to constrain the glas orientation
  // if in the hand. We shouldn't do this for a glas not connected to any
  // joint, since it leads to singularity.
  //glasInHand = RcsBody_isArticulated(graph, glasBdy);
  const Manipulator* receivingHand = domain.getGraspingHand(graph, pourToAff);

  if (receivingHand)
  {
    usedManipulators.push_back(pouringHand->name);
    RLOG(0, "Glass in hand");
    glasInHand = true;
  }
  else
  {
    RLOG(0, "Glass standing somewhere");
    glasInHand = false;
  }

  explanation = "I'm pouring " + std::to_string(this->pouringVolume) + " liters from the " +
                pourFromAff->name + " into the " + pourToAff->name;
}

ActionPour::~ActionPour()
{
}

std::vector<std::string> ActionPour::createTasksXML() const
{
  std::vector<std::string> tasks;

  // taskRelPos: XYZ-task with effector=bottle and refBdy=glas and refFrame=roboBase
  std::string xmlTask;
  xmlTask = "<Task name=\"" + taskRelPos + "\" " + "controlVariable=\"XYZ\" " +
            "effector=\"" + bottle + "\" " + "refBdy=\"" + glas + "\" "  +
            "refFrame = \"" + roboBaseFrame + "\" />";
  tasks.push_back(xmlTask);

  // taskBottleOri: Polar-task with effector=bottle and refBdy=roboBase (todo)
  xmlTask = "<Task name=\"" + taskBottleOri + "\" " + "controlVariable=\"POLAR\" " +
            "effector=\"" + bottle + "\" />";
  tasks.push_back(xmlTask);

  // taskGlasOri: Polar-task with effector=glas. We don't need any refBdy,
  // since we only keep it constant in case the glas is held with a hand.
  xmlTask = "<Task name=\"" + taskGlasOri + "\" " + "controlVariable=\"POLAR\" " +
            "effector=\"" + glas + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskGlasPosX + "\" " + "controlVariable=\"X\" " +
            "effector=\"" + glas + "\" refFrame = \"" + roboBaseFrame + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskGlasPosZ + "\" " + "controlVariable=\"Z\" " +
            "effector=\"" + glas + "\" refFrame = \"" + roboBaseFrame + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

/*
 * This action pours one container into antoher one. Here is the detailed
 * description of the timings:
 *
 * - t_prep: At this time point, the bottle tip is aligned with the glas rim
 *           but the bottle is only a little bit inclined. The bottle tip is
 *           still a bit above the glas rim.
 * - t_up:   The bottle tip is aligned exactly with the glas rim, and the
 *           bottle has been tilted so that the contents of it run into the
 *           glas.
 * - t_down: Like t_prep
 * - t_end:  The bottle separates sideways from the glas.
 * - t_afterTime: All tasks are deactivated.
 *
 * t_start   t_prep        t_up        t_down       t_end   t_afterTime
 *    |         |            |            |           |           |
 *  -----------------------------------------------------------------> time
 *
 * Pouring with right hand: polar theta is 90 degrees
 * Pouring with left hand: polar theta is -90 degrees
 */
tropic::TCS_sptr ActionPour::createTrajectory(double t_start, double t_end) const
{
  const double duration = t_end - t_start;
  const double t_prep = t_start + 0.15 * duration;
  const double t_up = t_start + 0.45 * duration;
  const double t_down = t_start + 0.9 * duration;
  const double afterTime = 0.5;
  const double thetaTilt = (bottleInRightHand) ? M_PI_2 : -M_PI_2;

  // How much do hands go away from each other once pouring finished
  const double d_separate = (bottleInRightHand) ? -0.25 : +0.25;
  const double heightAboveGlas = 0.08;

  auto a1 = std::make_shared<tropic::ActivationSet>();


  // Hand position with respect to bottle. We move the bottle tip over the
  // glas tip, keep it a little bit (while the bottle tilts), and then
  // move bottle and glas sideways apart.
  a1->addActivation(t_start, true, 0.5, taskRelPos);
  a1->addActivation(t_end+ afterTime, false, 0.5, taskRelPos);

  // At the time point t_prep, we keep a bit distance between bottle and glas
  // so that they don't collide. On the way of tilting the bottle up, we align
  // the opening frames.
  a1->add(t_prep, 0.6 * d_separate, 0.0, 0.0, 7, taskRelPos + " 1");
  a1->add(t_prep + 0.5*(t_up-t_prep), 0.0, 0.0, 0.0, 7, taskRelPos + " 1");
  a1->add(t_prep, heightAboveGlas, 0.0, 0.0, 7, taskRelPos + " 2");

  a1->add(std::make_shared<tropic::PositionConstraint>(t_up, 0.0, 0.0, 0.0, taskRelPos));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_up + 0.5*(t_down-t_up), 0.0, 0.0,
                                                       heightAboveGlas, taskRelPos, 1));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_down, 0.0, d_separate, heightAboveGlas, taskRelPos, 1));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_end, 0.0, d_separate, heightAboveGlas, taskRelPos));

  // Orientations
  a1->addActivation(t_start, true, 0.5, taskBottleOri);
  a1->addActivation(t_end + afterTime, false, 0.5, taskBottleOri);
  a1->add(std::make_shared<tropic::PolarConstraint>(t_prep, RCS_DEG2RAD(30.0), thetaTilt, taskBottleOri, 1));
  a1->add(std::make_shared<tropic::PolarConstraint>(t_up, RCS_DEG2RAD(150.0), thetaTilt, taskBottleOri));
  //a1->add(std::make_shared<tropic::PolarConstraint>(t_down, RCS_DEG2RAD(30.0), thetaTilt, taskBottleOri));
  a1->add(std::make_shared<tropic::PolarConstraint>(t_end, RCS_DEG2RAD(10.0), thetaTilt, taskBottleOri));

  if (glasInHand)
  {
    a1->addActivation(t_start, true, 0.5, taskGlasOri);
    a1->addActivation(t_end + afterTime, false, 0.5, taskGlasOri);
    a1->addActivation(t_start, true, 0.5, taskGlasPosX);
    a1->addActivation(t_end + afterTime, false, 0.5, taskGlasPosX);
    a1->addActivation(t_start, true, 0.5, taskGlasPosZ);
    a1->addActivation(t_end + afterTime, false, 0.5, taskGlasPosZ);
  }


  // Particles pouring
  for (const auto& particle : particlesToPour)
  {
    std::shared_ptr<tropic::ConnectBodyConstraint> cbc(new tropic::ConnectBodyConstraint(t_up, particle, glas));
    cbc->setConnectTransform(HTr_identity());
    a1->add(cbc);
  }

  return a1;
}

std::string ActionPour::explain() const
{
  return explanation;
}

std::vector<std::string> ActionPour::getManipulators() const
{
  return usedManipulators;
}

void ActionPour::performLiquidTransition(const AffordanceEntity* pourFromAff,
                                         const AffordanceEntity* pourToAff)
{
  auto toContainers = getAffordances<Containable>(pourToAff);
  auto fromContainers = getAffordances<Containable>(pourFromAff);

  // \todo(MG): Allow this, example pizza slices
  if (fromContainers.size()!=1)
  {
    throw ActionException("ERROR REASON: The " + pourFromAff->name + " that is poured from has " +
                          std::to_string(fromContainers.size()) + " containers - currently only one is supported",
                          ActionException::ParamInvalid);
  }

  if (toContainers.size()!=1)
  {
    throw ActionException("ERROR REASON: The object " + pourToAff->name + " that is poured into has " +
                          std::to_string(toContainers.size()) + " containers - currently only one is supported",
                          ActionException::ParamInvalid);
  }

  Containable* fromContainer = dynamic_cast<Containable*>(fromContainers[0]);
  Containable* toContainer = dynamic_cast<Containable*>(toContainers[0]);

  // Initial overflow check. We want to early exit here in case we overflow
  // the target container. This should leave the Affordances unchanged.
  double volumeToBePoured = Math_clip(pouringVolume, 0.0, fromContainer->getVolume());

  if (volumeToBePoured + toContainer->getVolume() > toContainer->maxVolume)
  {
    throw ActionException("ERROR REASON: The container " + toContainer->frame + " does only hold " +
                          std::to_string(toContainer->maxVolume) +
                          " liters, pouring would overflow it. SUGGESTION: Pour less than " +
                          std::to_string(toContainer->maxVolume-toContainer->getVolume()) + " liters",
                          ActionException::KinematicallyImpossible);
  }

  // Here we do the actual adjustments.

  // The volume that transitions is the smaller of the remaining target
  // container volume and the pouring volume.
  volumeToBePoured = std::min(volumeToBePoured, toContainer->maxVolume-toContainer->getVolume());

  // For each liquid, we pre-scale its volume with the ratio of the
  // volumeToBePoured and the originally given volume (per command). This
  // ensures that we don't get negative values in the pouring container, and no
  // overflow in the target container.
  const double scaleVolume = Math_clip(volumeToBePoured/pouringVolume, 0.0, 1.0);

  for (size_t from=0; from<fromContainer->liquidIngredients.size(); ++from)
  {
    // We first check if there's already some ingredient in the target container. If not, we
    // initialze it with a volume of 0.0
    bool hasIngredient = false;
    for (size_t to=0; to<toContainer->liquidIngredients.size(); ++to)
    {
      if (toContainer->liquidIngredients[to].first == fromContainer->liquidIngredients[from].first)
      {
        hasIngredient = true;
      }
    }

    if (!hasIngredient)
    {
      toContainer->liquidIngredients.push_back(std::make_pair(fromContainer->liquidIngredients[from].first,0.0));
    }

    // Here we do the transition
    for (size_t to=0; to<toContainer->liquidIngredients.size(); ++to)
    {
      // Found ingredient with the same name
      if (fromContainer->liquidIngredients[from].first == toContainer->liquidIngredients[to].first)
      {
        toContainer->liquidIngredients[to].second += scaleVolume*pouringVolume;
        fromContainer->liquidIngredients[from].second -= scaleVolume*pouringVolume;
      }
    }

    // We should also check if the "from" container is empty and we remove the ingredient
    // \todo(MG): Also check overflow ...
  }

  // RLOG(0, "Pouring ingredient start");
  // fromContainer->print();
  // toContainer->print();
  // RLOG(0, "Pouring ingredient end");
}

std::unique_ptr<ActionBase> ActionPour::clone() const
{
  return std::make_unique<ActionPour>(*this);
}

}   // namespace aff
