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

#include "ActionScrew.h"
#include "ActionFactory.h"

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <EulerConstraint.h>
#include <VectorConstraint.h>
#include <CollisionModelConstraint.h>

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#define T_FINGERMOVE  (2.0)


namespace aff
{
REGISTER_ACTION(ActionScrew, "screw");

ActionScrew::ActionScrew(const ActionScene& scene,
                         const RcsGraph* graph,
                         std::vector<std::string> params)
{
  parseParams(params);
  std::string screwingHand = params.size() > 1 ? params[1] : std::string();
  init(scene, graph, params[0], screwingHand);
}

void ActionScrew::init(const ActionScene& scene,
                       const RcsGraph* graph,
                       const std::string& objectToScrew,
                       const std::string& screwingHandName)
{
  std::vector<const AffordanceEntity*> nttsToScrew = scene.getAffordanceEntities(objectToScrew);

  if (nttsToScrew.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The " + objectToScrew + " is unknown.",
                          " Use an object name that is defined in the environment",
                          std::string(__FILENAME__) + " line " + std::to_string(__LINE__));
  }

  // We take the first one \todo(MG): Make generic.
  const AffordanceEntity* screwAff = nttsToScrew[0];
  this->bottleEntity = screwAff->bdyName;

  auto screwables = getAffordances<Twistable>(screwAff);


  // Both bottle and glas need to have an opening
  if (screwables.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The " + objectToScrew + " cannot be screwed open or close.",
                          "Open it differently or use an alternative container",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  // We make a local copy of the body strings, since they might be resolved
  // into different names when being a GenericBody
  this->bottleScrewable = screwables[0]->frame;

  // We go through the following look-ups to resolve the names of possible
  // generic bodies.
  const RcsBody* bottleCap = RcsGraph_getBodyByName(graph, bottleScrewable.c_str());
  RCHECK(bottleCap);

  this->bottleScrewable = std::string(bottleCap->name);


  const Manipulator* screwingHand = NULL;

  if (!screwingHandName.empty())
  {
    screwingHand = scene.getManipulator(screwingHandName);

    // If a manipulator has been passed but cannot be found, we consider this as an error.
    if (!screwingHand)
    {
      std::string reasonMsg = "Can't find a hand with the name " + screwingHandName;
      std::string suggestionMsg = "Use a hand name that is defined in the environment";
      throw ActionException(ActionException::ParamNotFound, reasonMsg, suggestionMsg,
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__));
    }

    if (!screwingHand->isEmpty(graph))
    {
      auto heldObjects = screwingHand->getGraspedEntities(scene, graph);
      RCHECK(!heldObjects.empty());
      std::string reasonMsg = "The hand " + screwingHandName + " is already holding the " + heldObjects[0]->name;
      if (heldObjects.size()>1)
      {
        reasonMsg = " and some other objects";
      }
      std::string suggestionMsg = "Free your hand before performing this command, or use another hand";
      throw ActionException(ActionException::ParamNotFound, reasonMsg, suggestionMsg,
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__));
    }
  }
  else
  {
    // The bottle object must be in a hand
    std::vector<const Manipulator*> allManipulators = scene.getFreeManipulators(graph);

    // We only consider hands (not head etc.)
    std::vector<const Manipulator*> freeHands;
    for (const auto& m : allManipulators)
    {
      if (m->isOfType("hand"))
      {
        freeHands.push_back(m);
      }

    }

    if (freeHands.empty())
    {
      throw ActionException(ActionException::KinematicallyImpossible,
                            "All hands are already full",
                            "Free one or all your hands before performing this command",
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__));
    }
    else if (freeHands.size() == 1)
    {
      screwingHand = freeHands[0];
    }
    else
    {
      const RcsBody* handBdy = RcsGraph_getBodyByName(graph, freeHands[0]->name.c_str());
      screwingHand = freeHands[0];
      double d = Vec3d_distance(handBdy->A_BI.org, bottleCap->A_BI.org);
      for (size_t i = 1; i < freeHands.size(); ++i)
      {
        const RcsBody* handBdy_i = RcsGraph_getBodyByName(graph, freeHands[i]->name.c_str());
        double di = Vec3d_distance(handBdy_i->A_BI.org, bottleCap->A_BI.org);
        if (di < d)
        {
          handBdy = handBdy_i;
          screwingHand = freeHands[i];
        }
      }
    }
  }


  this->hand = screwingHand->name;
  usedManipulators.push_back(this->hand);

  for (const auto& capability : screwingHand->capabilities)
  {
    if (dynamic_cast<const TwistgraspCapability*>(capability))
    {
      this->twistGraspFrame = capability->frame;
      break;
    }
  }

  if (twistGraspFrame.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Grasping hand cannot twist",
                          "Make sure manipulator has TwistgraspCapability",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  // Task naming
  this->taskRelXYZ = bottleScrewable + "-" + hand + "-XYZ";
  this->taskRelPolar = bottleScrewable + "-" + hand + "-POLAR";
  this->taskRelInclination = bottleScrewable + "-" + hand + "-Inclination";
  this->taskRelABC = bottleScrewable + "-" + hand + "-ABC";
  this->taskBottlePolar = bottleScrewable + "-" + "-POLAR";
  this->taskBottleInclination = bottleScrewable + "-" + "-Inclination";
  this->taskBottleX = bottleScrewable + "-" + "-X";
  this->taskBottleY = bottleScrewable + "-" + "-Y";
  this->taskBottleZ = bottleScrewable + "-" + "-Z";
  this->taskFingers = "Fingers";

  // this->fingerJnts = screwingHand->getFingerJointsString();
  fingerJnts.clear();
  for (auto& f : screwingHand->fingerJoints)
  {
    fingerJnts += f;
    fingerJnts += " ";
  }

  fingersOpen = screwingHand->getFingerAnglesFromModelState(graph, "open_fingers");
  fingersClosed = screwingHand->getFingerAnglesFromModelState(graph, "close_fingers");

  if (screwingHand->fingerJoints.size() != fingersOpen.size())
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Screwing hand fingers open mismatch: ",
                          "Finger joints: " + std::to_string(screwingHand->fingerJoints.size()) + " != finger angles " + std::to_string(fingersOpen.size()),
                          "Graph xml file: " + std::string(graph->cfgFile) + " " + std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  if (screwingHand->fingerJoints.size() != fingersClosed.size())
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Screwing hand fingers open mismatch: ",
                          "Finger joints: " + std::to_string(screwingHand->fingerJoints.size()) + " != finger angles " + std::to_string(fingersClosed.size()),
                          "Graph xml file: " + std::string(graph->cfgFile) + " " + std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }
}

ActionScrew::~ActionScrew()
{
}

std::vector<std::string> ActionScrew::createTasksXML() const
{
  std::vector<std::string> tasks;

  // taskRelXYZ: XYZ-task with effector=hand and refBdy=bottle
  std::string xmlTask;
  xmlTask = "<Task name=\"" + taskRelXYZ + "\" " + "controlVariable=\"XYZ\" " +
            "effector=\"" + twistGraspFrame + "\" " + "refBdy=\"" + bottleScrewable + "\" />";
  tasks.push_back(xmlTask);

  // Same for orientations
  xmlTask = "<Task name=\"" + taskRelABC + "\" " + "controlVariable=\"ABC\" " +
            "effector=\"" + hand + "\" " + "refBdy=\"" + bottleScrewable + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskRelPolar + "\" " + "controlVariable=\"POLAR\" " +
            "effector=\"" + hand + "\" " + "refBdy=\"" + bottleScrewable + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskRelInclination + "\" " + "controlVariable=\"Inclination\" " +
            "effector=\"" + hand + "\" " + "refBdy=\"" + bottleScrewable + "\" />";
  tasks.push_back(xmlTask);

  // Bottle orientation for bi-manual case
  xmlTask = "<Task name=\"" + taskBottlePolar + "\" " + "controlVariable=\"POLAR\" " +
            "effector=\"" + bottleScrewable + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskBottleInclination + "\" " + "controlVariable=\"Inclination\" " +
            "effector=\"" + bottleScrewable + "\" >";
  xmlTask += "\n<TaskRegion type=\"BoxInterval\" ";
  xmlTask += "min=\"" + std::to_string(RCS_DEG2RAD(-30.0)) + "\" ";
  xmlTask += "max=\"" + std::to_string(RCS_DEG2RAD(30.0)) + "\" ";
  xmlTask += "dxScaling=\"0.01\" slowDownRatio=\"0.5\" />\n";
  xmlTask += "</Task>";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskBottleZ + "\" " + "controlVariable=\"Z\" " +
            "effector=\"" + bottleScrewable + "\" >";
  xmlTask += "\n<TaskRegion type=\"BoxInterval\" ";
  xmlTask += "min=\"" + std::to_string(0.0) + "\" max=\"" + std::to_string(0.05) + "\" dxScaling=\"0.1\" slowDownRatio=\"0.5\" />\n";
  xmlTask += "</Task>";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskBottleX + "\" " + "controlVariable=\"X\" " +
            "effector=\"" + bottleScrewable + "\" " + "refBdy=\"" + "adapter_shoulder" + "\" >";
  xmlTask += "\n<TaskRegion type=\"BoxInterval\" ";
  xmlTask += "min=\"" + std::to_string(-0.05) + "\" max=\"" + std::to_string(0.05) + "\" dxScaling=\"0.1\" slowDownRatio=\"0.5\" />\n";
  xmlTask += "</Task>";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskBottleY + "\" " + "controlVariable=\"Y\" " +
            "effector=\"" + bottleScrewable + "\" " + "refBdy=\"" + "adapter_shoulder" + "\" >";
  xmlTask += "\n<TaskRegion type=\"BoxInterval\" ";
  xmlTask += "min=\"" + std::to_string(-0.05) + "\" max=\"" + std::to_string(0.05) + "\" dxScaling=\"0.1\" slowDownRatio=\"0.5\" />\n";
  xmlTask += "</Task>";
  tasks.push_back(xmlTask);

  // Fingers open-close task
  xmlTask = "<Task name=\"" + taskFingers + "\" "  "controlVariable=\"Joints\" " +
            "jnts=\"" + this->fingerJnts + "\" />";
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
 * t_start t_preshape        t_up        t_down       t_end   t_afterTime
 *    |         |            |            |           |           |
 *  -----------------------------------------------------------------> time
 *
 * Pouring with right hand: polar theta is 90 degrees
 * Pouring with left hand: polar theta is -90 degrees
 */
tropic::TCS_sptr ActionScrew::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  const double duration = t_end - t_start;
  const double t_preshape = t_start + 0.3 * duration;
  const double t_handalign = t_start + 0.4 * duration;
  const double t_fwd1 = t_start + 0.5 * duration;
  const double t_bwd1 = t_start + 0.6 * duration;
  const double t_fwd2 = t_start + 0.7 * duration;
  const double t_bwd2 = t_start + 0.8 * duration;
  const double t_fwd3 = t_start + 0.9 * duration;

  const double heightAboveBottle = 0.02;
  const double screwAmplitude = 0.8 * M_PI_2;

  auto a1 = std::make_shared<tropic::ActivationSet>();


  // Hand position with respect to bottle. We move the bottle tip over the
  // glas tip, keep it a little bit (while the bottle tilts), and then
  // move bottle and glas sideways apart.
  a1->addActivation(t_start, true, 0.5, taskRelXYZ);
  a1->addActivation(t_end+afterTime, false, 0.5, taskRelXYZ);

  a1->addActivation(t_start, true, 0.5, taskRelPolar);
  //a1->addActivation(t_preshape, false, 0.5, taskRelPolar);
  a1->addActivation(t_start + 0.3*duration, false, 0.5, taskRelPolar);

  a1->addActivation(t_start + 0.3*duration, true, 0.5, taskRelABC);
  //a1->addActivation(t_preshape, true, 0.5, taskRelABC);
  a1->addActivation(t_fwd3, false, 0.5, taskRelABC);

  a1->addActivation(t_fwd3, true, 0.5, taskRelInclination);
  //a1->addActivation(t_fwd3 + 0.25*(t_end-t_fwd3), false, 0.5, taskRelPolar);
  a1->addActivation(t_end+afterTime, false, 0.5, taskRelInclination);

  // This keeps the bottle axis fixed (upright).
  //a1->addActivation(t_start, true, 0.5, taskBottlePolar);
  //a1->addActivation(t_end + afterTime, false, 0.5, taskBottlePolar);
  a1->addActivation(t_start, true, 0.5, taskBottleInclination);
  a1->addActivation(t_end + afterTime, false, 0.5, taskBottleInclination);

  a1->addActivation(t_start, true, 0.5, taskBottleZ);
  a1->addActivation(t_end + afterTime, false, 0.5, taskBottleZ);

  a1->addActivation(t_start, true, 0.5, taskBottleX);
  a1->addActivation(t_end + afterTime, false, 0.5, taskBottleX);

  a1->addActivation(t_fwd3, true, 0.5, taskBottleY);
  a1->addActivation(t_end + afterTime, false, 0.5, taskBottleY);

  // At the time point t_prep, we keep a bit distance between bottle and glas
  // so that they don't collide. On the way of tilting the bottle up, we align
  // the opening frames.
  //a1->add(std::make_shared<tropic::PositionConstraint>(t_preshape, 0.0, 0.0, heightAboveBottle, taskRelXYZ, 7));
  a1->add(t_preshape, 0.0, 0.0, 0.0, 7, taskRelXYZ + " 0");
  a1->add(t_preshape, 0.0, 0.0, 0.0, 7, taskRelXYZ + " 1");
  a1->add(t_preshape, heightAboveBottle, 0.0, 0.0, 1, taskRelXYZ + " 2");



  a1->add(std::make_shared<tropic::PolarConstraint>(t_preshape, 0.0, 0.0, taskRelPolar));

  a1->add(std::make_shared<tropic::PositionConstraint>(t_handalign, 0.0, 0.0, 0.0, taskRelXYZ));

  // This constraint aligns the orientation with the bottle. Since we don't know the relative
  // pose, the speeds might be high
  a1->add(std::make_shared<tropic::EulerConstraint>(t_handalign, 0.0, 0.0, 0.0, taskRelABC));

  a1->add(std::make_shared<tropic::EulerConstraint>(t_fwd1, 0.0, 0.0, screwAmplitude, taskRelABC));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_bwd1, 0.0, 0.0, 0.0, taskRelABC));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_fwd2, 0.0, 0.0, screwAmplitude, taskRelABC));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_bwd2, 0.0, 0.0, 0.0, taskRelABC));
  a1->add(std::make_shared<tropic::EulerConstraint>(t_fwd3, 0.0, 0.0, screwAmplitude, taskRelABC));

  a1->add(std::make_shared<tropic::PositionConstraint>(t_fwd3, 0.0, 0.0, 0.0, taskRelXYZ));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_end, 0.0, 0.0, 2.0*heightAboveBottle, taskRelXYZ));

  // Incline hand wrt bottle at the end of the retract
  std::vector<double> relInc;
  relInc.push_back(RCS_DEG2RAD(50.0));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, relInc, taskRelInclination));

  //std::vector<double> ypos;
  //ypos.push_back(-0.0);
  //a1->add(std::make_shared<tropic::VectorConstraint>(t_preshape, ypos, taskBottleY));

  //std::vector<double> xpos;
  //xpos.push_back(0.6);
  //a1->add(std::make_shared<tropic::VectorConstraint>(t_preshape, xpos, taskBottleX));


  ///////Fingers////////////
  a1->addActivation(t_start, true, 0.5, taskFingers);
  a1->addActivation(t_end + afterTime, false, 0.5, taskFingers);

  RLOG_CPP(0, "FingersOpen: " << fingersOpen[0]);
  RLOG_CPP(0, "FingersClosed: " << fingersClosed[0]);

  a1->add(std::make_shared<tropic::VectorConstraint>(t_preshape, fingersOpen, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_handalign - 1.0*T_FINGERMOVE, fingersOpen, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_handalign, fingersClosed, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_fwd1, fingersClosed, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_fwd1 + 1.0 * T_FINGERMOVE, fingersOpen, taskFingers));

  a1->add(std::make_shared<tropic::VectorConstraint>(t_bwd1 - 1.0 * T_FINGERMOVE, fingersOpen, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_bwd1, fingersClosed, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_fwd2, fingersClosed, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_fwd2 + 1.0 * T_FINGERMOVE, fingersOpen, taskFingers));

  a1->add(std::make_shared<tropic::VectorConstraint>(t_bwd2 - 1.0 * T_FINGERMOVE, fingersOpen, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_bwd2, fingersClosed, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_fwd3, fingersClosed, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_fwd3 + 1.0 * T_FINGERMOVE, fingersOpen, taskFingers));

  // Deactivate object collisions when in preshape
  a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_preshape, bottleEntity, false));

  // Reactivate object collisions when finished
  a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_end, bottleEntity, true));

  return a1;
}

double ActionScrew::getDefaultDuration() const
{
  return 80.0;
}

std::vector<std::string> ActionScrew::getManipulators() const
{
  return usedManipulators;
}

std::unique_ptr<ActionBase> ActionScrew::clone() const
{
  return std::make_unique<ActionScrew>(*this);
}

std::string ActionScrew::getActionCommand() const
{
  std::string str = "screw " + bottleEntity;

  if (getDuration() != getDefaultDuration())
  {
    str += " duration " + std::to_string(getDuration());
  }

  return str;
}

}   // namespace aff
