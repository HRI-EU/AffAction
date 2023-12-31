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

#include "ActionGet.h"
#include "ActionFactory.h"
#include "TrajectoryPredictor.h"
#include "CollisionModelConstraint.h"

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <EulerConstraint.h>
#include <ConnectBodyConstraint.h>
#include <VectorConstraint.h>

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#include <limits>
#include <tuple>
#include <algorithm>


#define fingersOpen   (0.01)
#define fingersHalfClosed (0.7)
#define fingersClosed (0.7)   // Powergrasp for bottle etc
#define t_fingerMove  (2.0)
#define DEFAULT_LIFTHEIGHT (0.12)
#define DEFAULT_PREGRASPDIST (0.2)
#define LIFT_SAFETY_DISTANCE (0.05)   // Safety distance before colliding with object above



namespace aff
{
/*******************************************************************************
 *
 ******************************************************************************/
REGISTER_ACTION(ActionGet, "get");
REGISTER_ACTION(ActionGet, "id_6355842184f61faabb83cb1b");


ActionGet::ActionGet() :
  graspType(GraspType::Other), liftHeight(DEFAULT_LIFTHEIGHT), preGraspDist(DEFAULT_PREGRASPDIST),
  shoulderBase(0.0), handOver(false), isObjCollidable(false)
{
}

ActionGet::ActionGet(const ActionScene& domain,
                     const RcsGraph* graph,
                     std::vector<std::string> params) : ActionGet()
{
  auto it = std::find(params.begin(), params.end(), "from");
  if (it != params.end())
  {
    whereFrom = *(it+1);
    params.erase(it+1);
    params.erase(it);
  }

  it = std::find(params.begin(), params.end(), "liftHeight");
  if (it != params.end())
  {
    liftHeight = std::stod(*(it+1));
    params.erase(it+1);
    params.erase(it);
  }

  it = std::find(params.begin(), params.end(), "duration");
  if (it != params.end())
  {
    defaultDuration = std::stod(*(it+1));
    params.erase(it+1);
    params.erase(it);
  }

  std::string objectToGet = params[0];
  std::string manipulator = params.size()>1?params[1]:std::string();
  init(domain, graph, objectToGet, manipulator, whereFrom);
}

void ActionGet::init(const ActionScene& domain,
                     const RcsGraph* graph,
                     const std::string& objectToGet,
                     const std::string& manipulator,
                     const std::string& whereFrom)
{
  RLOG_CPP(1, "Calling get with manipulator='" << manipulator
           << "' object='" << objectToGet << "' from='" << whereFrom << "'");

  std::string errorMsg = "ERROR: ";

  // Initialize object to get.
  const AffordanceEntity* entityToGet = NULL;

  if (whereFrom.empty())
  {
    // \todo(MG): Here we should consider all affordance entities, and exclude
    // the ones that are held in a hand already.
    std::vector<const AffordanceEntity*> nttsToGet = domain.getAffordanceEntities(objectToGet);

    if (nttsToGet.empty())
    {
      std::string reasonMsg = " REASON: The " + objectToGet +
                              " is unknown. SUGGESTION: Use an object name that is defined in the environment";
      throw ActionException(errorMsg + reasonMsg, ActionException::ParamNotFound);
    }

    // We go through all entities with the same name.
    for (auto ntt : nttsToGet)
    {
      // We succeed if there is already an object with this name in the hand
      const Manipulator* graspingHand = domain.getGraspingHand(graph, ntt);
      const Manipulator* specifiedHand = domain.getManipulator(manipulator);

      // \todo(MG): This code is duplicate to further below.
      if ((!manipulator.empty()) && (!specifiedHand))
      {
        std::string reasonMsg = " REASON: Can't find a hand with the name " + manipulator;
        std::string suggestionMsg = " SUGGESTION: Use a hand name that is defined in the environment";
        throw ActionException(errorMsg + reasonMsg + suggestionMsg, ActionException::ParamNotFound);
      }

      // We are done if:
      // - the object is grasped with the hand that has been specified (parameter "manipulator")
      // - the object is grasped, and no hand to grasp it has been specified (parameter "manipulator" is empty)
      if (graspingHand && ((graspingHand == specifiedHand) || (!specifiedHand)))
      {
        throw ActionException("SUCCESS DEVELOPER: Object is already held in the hand", ActionException::NoError);
      }
      // If not, we take the first one we find.
      else
      {
        entityToGet = ntt;
        break;
      }
    }

    RCHECK_MSG(entityToGet, "There is an object and it is not held in a hand");

    // Ending up here means we do have an entityToGet
  }
  // This case handles that the action has a "whereFrom" parameter, i.e. from
  // which supporting object the entity is to be gotten. This is a hint
  // helping in cases with several identically named AffordanceEntities.
  else
  {
    const AffordanceEntity* fromNTT = domain.getAffordanceEntity(whereFrom);
    std::string fromBdyName;

    if (fromNTT)
    {
      fromBdyName = fromNTT->bdyName;
    }
    else
    {
      // Here we check if whereFrom refers to a Manipulator so that we do a
      // hand-over.
      const Manipulator* fromHand = domain.getManipulator(whereFrom);

      if (!fromHand)
      {
        std::string reasonMsg = " REASON: The " + whereFrom + " is neither a hand nor a part of the environment";
        std::string suggestionMsg = " SUGGESTION: Use an object name or a hand that is defined in the environment";
        throw ActionException(errorMsg + reasonMsg + suggestionMsg, ActionException::ParamNotFound);
      }

      fromBdyName = fromHand->name;
    }

    const RcsBody* bdyFrom = RcsGraph_getBodyByName(graph, fromBdyName.c_str());
    RCHECK_MSG(bdyFrom, "%s", fromBdyName.c_str());   // Should never happen after initial check

    std::vector<const AffordanceEntity*> ntts = domain.getAffordanceEntities(objectToGet);

    for (auto ntt : ntts)
    {
      const RcsBody* bdy = RcsGraph_getBodyByName(graph, ntt->bdyName.c_str());
      if (RcsBody_isChild(graph, bdy, bdyFrom))
      {
        entityToGet = ntt;
      }
    }

    if (!entityToGet)
    {
      std::string reasonMsg = " REASON: The " + whereFrom + " is empty";
      std::string suggestionMsg = " SUGGESTION: Use an alternative object";
      throw ActionException(errorMsg + reasonMsg, ActionException::ParamNotFound);
    }

  }

  RCHECK(entityToGet); // Never happens

  isObjCollidable = entityToGet->isCollideable(graph);

  auto graspables = getAffordances<Graspable>(entityToGet);

  if (graspables.empty())
  {
    std::string reasonMsg = " REASON: " + objectToGet + " cannot be grasped";
    throw ActionException(errorMsg + reasonMsg, ActionException::ParamNotFound);
  }

  // ResolveBodyName changes the objectName in certain cases (e.g. generic bodies)
  objectName = entityToGet->bdyName;
  const RcsBody* objBdy = resolveBodyName(graph, objectName);
  RCHECK_MSG(objBdy, "Couldn't resolve object body \"%s\". Is it part of the graph?", objectName.c_str());

  // Adjust lift height: First we compute an axis-aligned bounding box around
  // the body to get. We raycast upwards from its top. This avoids
  // self-intersections.
  double xyzMin[3], xyzMax[3], castFrom[3], surfPt[3], dMin = 0.0;
  bool aabbValid = RcsGraph_computeBodyAABB(graph, objBdy->id, RCSSHAPE_COMPUTE_DISTANCE, xyzMin, xyzMax, NULL);
  if (aabbValid)
  {
    Vec3d_set(castFrom, 0.5 * (xyzMin[0] + xyzMax[0]), 0.5 * (xyzMin[1] + xyzMax[1]), xyzMax[2] + 1.0e-8);
  }
  else
  {
    Vec3d_copy(castFrom, objBdy->A_BI.org);   // Body origin in case no AABB can be determined.
  }

  const RcsBody* surfaceBdy = RcsBody_closestRigidBodyInDirection(graph, castFrom, Vec3d_ez(), surfPt, &dMin);

  if (surfaceBdy && !RcsBody_isChild(graph, surfaceBdy, objBdy))
  {
    this->liftHeight = Math_clip(this->liftHeight, 0.0, std::min(this->liftHeight, dMin-LIFT_SAFETY_DISTANCE));
    RLOG(0, "Reducing lift height to %f because of %s", this->liftHeight, surfaceBdy->name);
  }


  // The lift height is in absolute coordinates, so we add the object's current
  // z-coordinate.
  this->liftHeight += objBdy->A_BI.org[2];

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
      auto heldObjects = hand->getGraspedEntities(domain, graph);
      RCHECK(!heldObjects.empty());
      std::string reasonMsg = " REASON: The hand " + manipulator + " is already holding the " + heldObjects[0]->name;
      if (heldObjects.size()>1)
      {
        reasonMsg = " and some other objects";
      }

      std::string suggestionMsg = " SUGGESTION: Free your hand before performing this command, or use another hand";
      throw ActionException(errorMsg + reasonMsg + suggestionMsg, ActionException::ParamNotFound);
    }

    freeManipulators.push_back(hand);
  }

  if (freeManipulators.empty())
  {
    std::string reasonMsg = " REASON: All hands are already full";
    std::string suggestionMsg = " SUGGESTION: Free one or all your hands before performing this command";
    std::string devMsg = std::string(" DEVELOPER: ") + std::string(__FILENAME__) + " line " +
                         std::to_string(__LINE__);
    throw ActionException(errorMsg + reasonMsg + suggestionMsg + devMsg,
                          ActionException::KinematicallyImpossible);
  }

  // From here on we have one or several manipulators that can be tentatively used
  // to get the object. We now go through each of them and compile a list of pairs
  // matching each manipulator's capability to the object affordances.
  for (const Manipulator* hand : freeManipulators)
  {
    auto graspMap_i = match<Graspable>(entityToGet, hand);
    affordanceMap.insert(affordanceMap.end(), graspMap_i.begin(), graspMap_i.end());
  }

  // If this vector is empty, there's no capability that can get the object in any way.
  // How sad that we have to stop here.
  if (affordanceMap.empty())
  {
    std::string reasonMsg = " REASON: I don't have the capability of grasping the " + objectToGet;
    throw ActionException(errorMsg + reasonMsg, ActionException::KinematicallyImpossible);
  }

  // We have one or several matches and sort them according to their cost. Lower
  // costs pairs are at the beginning.
  sort(graph, affordanceMap, 1.0, 0.0);

  // If the object has been grasped already, we eliminate the affordance frames
  // that the other hand is currently using so that we can realize a hand-over.
  const Manipulator* graspingHand = domain.getGraspingHand(graph, entityToGet);

  if (graspingHand)
  {
    handOver = true;
    handOverHand = graspingHand->name;

    // We also reset the lift height so that we don't get increased height
    // per each hand-over.
    this->liftHeight = objBdy->A_BI.org[2];
    RLOG(0, "\"%s\" to grasp is held in hand", entityToGet->name.c_str());

#if 0
    const double frameRadius = 0.05;
    std::vector<const Affordance*> graspAffs = graspingHand->getGraspAffordances(graph, entityToGet, frameRadius);

    // \todo(MG): This seems to be incorrect when deleting several iterators. On each delete, the indices
    //            change. We might need to do a reverse_iterator or so.
    for (size_t i=0; i<graspAffs.size(); ++i)
    {
      RLOG(0, "graspAff: \"%s\"", graspAffs[i]->frame.c_str());
      for (auto it = affordanceMap.begin(); it != affordanceMap.end(); ++it)
      {
        if (std::get<0>(*it)==graspAffs[i])
        {
          RLOG(0, "Deleting affordance %s from affordanceMap", graspAffs[i]->frame.c_str());
          affordanceMap.erase(it);
          break;
        }
      }
    }
#endif
  }

#if 0
  // We need to check this again for the case that the object is grasped, and
  // the grasp affordances have been removed above.
  if (affordanceMap.empty())
  {
    std::string reasonMsg = " REASON: It is too difficult to hand the " + objectToGet + " over to the other hand";
    throw ActionException(errorMsg + reasonMsg, ActionException::KinematicallyImpossible);
  }
#endif

  // Initialize with the best solution.
  bool successInit = initialize(domain, graph, 0);
  RCHECK(successInit);
}

bool ActionGet::initialize(const ActionScene& domain,
                           const RcsGraph* graph,
                           size_t solutionRank)
{
  if (solutionRank >= getNumSolutions())
  {
    return false;
  }


  // Get the frame of the capability from the first entry
  const Capability* winningCap = std::get<1>(affordanceMap[solutionRank]);
  capabilityFrame = winningCap->frame;

  // Get the hand that corresponds to the selected capability.
  const Manipulator* hand = domain.getManipulator(winningCap);
  RCHECK(hand);   // Never happens since already checked in constructor

  // Initialize hand open and close with defaults. We currently update it for the BallGrasp
  handOpen = std::vector<double>(hand->getNumFingers(), fingersOpen);
  handClosed = std::vector<double>(hand->getNumFingers(), fingersClosed);

  // Get the frame of the affordance from the second capability
  Affordance* winningAff = std::get<0>(affordanceMap[solutionRank]);
  affordanceFrame = winningAff->frame;

  if (dynamic_cast<PowerGraspable*>(winningAff))
  {
    graspType = GraspType::PowerGrasp;
  }
  else if (dynamic_cast<BallGraspable*>(winningAff))
  {
    BallGraspable* bg = dynamic_cast<BallGraspable*>(winningAff);
    handOpen = std::vector<double>(hand->getNumFingers(), fingersHalfClosed);
    handClosed = hand->fingerAnglesFromFingerTipDistance(2.0*bg->radius);
    graspType = GraspType::BallGrasp;
  }
  else if (dynamic_cast<TwistGraspable*>(winningAff))   // This is true for Twistables as well
  {
    TwistGraspable* tg = dynamic_cast<TwistGraspable*>(winningAff);
    handClosed = hand->fingerAnglesFromFingerTipDistance(2.0 * tg->radius);
    graspType = GraspType::TopGrasp;
  }
  else if (dynamic_cast<CircularGraspable*>(winningAff))
  {
    graspType = GraspType::RimGrasp;
  }
  else
  {
    graspType = GraspType::OrientedGrasp;
  }

  RLOG_CPP(1, "Capability frame: " << capabilityFrame << " affordance frame: "
           << affordanceFrame << " graspType: " << graspTypeToString(graspType));

  // Find the shoulder link where the arms branch
  {
    const RcsBody* bdyPtr = RcsGraph_getBodyByName(graph, hand->name.c_str());
    RCHECK_MSG(bdyPtr, "Hand body \"%s\" not found", hand->name.c_str());
    bdyPtr = RCSBODY_BY_ID(graph, bdyPtr->parentId);
    RCHECK_MSG(bdyPtr, "Hand body first parent not found");
    while (bdyPtr->parentId !=1)
    {
      const RcsBody* parentPtr = RCSBODY_BY_ID(graph, bdyPtr->parentId);
      //RLOG(0, "Checking %s", parentPtr ? parentPtr->name : "NULL");
      if ((!parentPtr) || (parentPtr->firstChildId != parentPtr->lastChildId))
      {
        //RLOG(0, "Found %s to be root of hand - returning %s", parentPtr ? parentPtr->name : "NULL", bdyPtr->name);
        shoulderBase = bdyPtr->A_BI.org[1];// \todo: Transform into base.
        break;
      }
      bdyPtr = parentPtr;
    }
  }

  //RCHECK(shoulderBase!=0.0);
  //RLOG(0, "Shoulder base coordinate: %f", shoulderBase);

  usedManipulators.clear();
  usedManipulators.push_back(hand->name);

  // We keep the other manipulator if it is occupied
  auto occupiedManipulators = domain.getOccupiedManipulators(graph);
  for (auto m : occupiedManipulators)
  {
    if (m->name != hand->name)
    {
      usedManipulators.push_back(m->name);
    }
  }


  // Assemble finger task name
  fingerJoints.clear();
  for (auto& f : hand->fingerJoints)
  {
    fingerJoints += f;
    fingerJoints += " ";
  }

  // Task naming
  this->taskObjHandPos = objectName + "-" + capabilityFrame + "-XYZ";
  this->taskObjPosX = objectName + "-X";
  this->taskObjPosY = objectName + "-Y";
  this->taskObjPosZ = objectName + "-Z";
  this->taskObjOri = objectName + "-POLAR";
  this->taskHandObjOri = capabilityFrame + "-" + objectName + "-" + graspTypeToString(graspType);
  this->taskFingers = capabilityFrame + "_fingers";

  if (!handOverHand.empty())
  {
    this->taskHandOverHand = handOverHand + "_XYZ";
  }

  return true;
}

ActionGet::~ActionGet()
{
}

tropic::TCS_sptr ActionGet::createTrajectory(double t_start, double t_end) const
{
  const double t_grasp = t_start + 0.66*(t_end - t_start);
  const double t_pregrasp = t_start + 0.5*(t_grasp-t_start);
  std::shared_ptr<tropic::ActivationSet> a1;

  if (graspType == GraspType::BallGrasp)
  {
    a1 = std::dynamic_pointer_cast<tropic::ActivationSet>(createTrajectoryBallGrasp(t_start, t_grasp, t_end));
  }
  else if (graspType == GraspType::TopGrasp)
  {
    a1 = std::dynamic_pointer_cast<tropic::ActivationSet>(createTrajectoryTopGrasp(t_start, t_grasp, t_end));
  }
  else if (graspType == GraspType::RimGrasp)
  {
    a1 = std::dynamic_pointer_cast<tropic::ActivationSet>(createTrajectoryRimGrasp(t_start, t_grasp, t_end));
  }
  else
  {
    a1 = std::dynamic_pointer_cast<tropic::ActivationSet>(createTrajectory(t_start, t_grasp, t_end));
  }

  RCHECK(a1);

  if ((!handOpen.empty()) && (!handClosed.empty()))
  {
    a1->addActivation(t_start, true, 0.5, taskFingers);
    a1->add(std::make_shared<tropic::VectorConstraint>(t_grasp-0.5*t_fingerMove, handOpen, taskFingers));
    a1->add(std::make_shared<tropic::VectorConstraint>(t_grasp+0.5*t_fingerMove, handClosed, taskFingers));
  }

  if (isObjCollidable)
  {
    // Deactivate object collisions when in pregrasp
    a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_pregrasp, objectName, false));

    // Reactivate object collisions when in grasped
    a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_end, objectName, true));
  }

  // if (getOptimDim()==3)
  // {
  //   const double t_optim = t_start + 0.25 * (t_grasp - t_start);
  //   auto oState = getOptimState();
  //   RLOG(1, "Adding optimization constraint [%.4f %.4f %.4f] at t=%f",
  //        oState[0], oState[1], oState[2], t_optim);
  //   a1->add(std::make_shared<tropic::PositionConstraint>(t_optim, oState.data(), taskObjHandPos, 1));
  // }

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
ActionGet::createTrajectory(double t_start,
                            double t_grasp,
                            double t_end) const
{
  const double afterTime = 0.5;

  // Grasp the object and lift it up
  auto a1 = std::make_shared<tropic::ActivationSet>();

  // Hand position with respect to object
  const double t_pregrasp = t_start + 0.5*(t_grasp-t_start);
  a1->addActivation(t_start, true, 0.5, taskObjHandPos);
  a1->addActivation(t_grasp, false, 0.5, taskObjHandPos);
  a1->add(t_pregrasp, preGraspDist, 0.0, 0.0, 1, taskObjHandPos + " 0");// some distance in front
  a1->add(t_pregrasp, 0.0, 0.0, 0.0, 7, taskObjHandPos + " 1");// and centered
  a1->add(t_pregrasp, 0.0, 0.0, 0.0, 7, taskObjHandPos + " 2");// and adjusted height \todo check
  a1->add(std::make_shared<tropic::PositionConstraint>(t_grasp, 0.0, 0.0, 0.0, taskObjHandPos));

  // Hand orientation with respect to object for the approach motion
  a1->addActivation(t_start, true, 0.5, taskHandObjOri);
  a1->addActivation(t_grasp, false, 0.5, taskHandObjOri);

  if (graspType==GraspType::PowerGrasp)
  {
    a1->add(std::make_shared<tropic::PolarConstraint>(t_grasp, 0.0, 0.0, taskHandObjOri));
  }
  else if (graspType==GraspType::OrientedGrasp)  // PalmGraspable
  {
    a1->add(std::make_shared<tropic::EulerConstraint>(t_grasp, 0.0, 0.0, 0.0, taskHandObjOri));
  }
  else
  {
    RFATAL("Unknown grasp orientation: %d", (int)graspType);
  }

  // Object orientation with respect to world frame. We keep it upright, but
  // don't enforce the orientation around the up-axis. For something like an
  // apple (Inclination constraint only, we don't need to enforce any object
  // orientation. \todo(MG): Do this.
  a1->addActivation(handOver ? t_start : t_grasp, true, 0.5, taskObjOri);
  a1->addActivation(t_end+afterTime, false, 0.5, taskObjOri);

  // Lift object
  a1->addActivation(handOver ? t_start : t_grasp, true, 0.5, taskObjPosZ);
  a1->addActivation(t_end+afterTime, false, 0.5, taskObjPosZ);
  a1->add(t_end, liftHeight, 0.0, 0.0, 7, taskObjPosZ + " 0");
  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_grasp, objectName, capabilityFrame));

  // Lift it up in the y-z plane (keep forward position constant)
  //if (!handOver)
  {
    a1->addActivation(handOver ? t_start : t_grasp, true, 0.5, taskObjPosX);
    a1->addActivation(t_end+afterTime, false, 0.5, taskObjPosX);
  }

  // Reduce snap to null space y-position by moving up vertically
  a1->addActivation(t_grasp, true, 0.5, taskObjPosY);
  //a1->addActivation(t_grasp+0.5*(t_end-t_grasp), false, 0.5, taskObjPosY);
  a1->addActivation(t_end+afterTime, false, 0.5, taskObjPosY);

  // HACK
  if (shoulderBase != 0.0)
  {
    a1->add(t_end, -0.35, 0.0, 0.0, 7, taskObjPosX + " 0");
    a1->add(t_end, shoulderBase<0.0?-0.3:0.3, 0.0, 0.0, 7, taskObjPosY + " 0");
  }

  // If the object is handed over, we keep the hand-over hand at its position
  if (!taskHandOverHand.empty())
  {
    a1->addActivation(t_grasp, true, 0.5, taskHandOverHand);
    a1->addActivation(t_end, false, 0.5, taskHandOverHand);
  }

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
ActionGet::createTrajectoryBallGrasp(double t_start,
                                     double t_grasp,
                                     double t_end) const
{
  const double afterTime = 0.5;

  // Grasp the object and lift it up
  auto a1 = std::make_shared<tropic::ActivationSet>();

  // Hand position with respect to object
  const double t_pregrasp = t_start + 0.5*(t_grasp-t_start);
  a1->addActivation(t_start, true, 0.5, taskObjHandPos);
  a1->addActivation(t_grasp, false, 0.5, taskObjHandPos);
  a1->add(std::make_shared<tropic::PositionConstraint>(t_pregrasp, 0.0, 0.0, preGraspDist, taskObjHandPos));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_grasp, 0.0, 0.0, 0.0, taskObjHandPos));

  // Hand orientation with respect to object for the approach motion
  a1->addActivation(t_start, true, 0.5, taskHandObjOri);
  a1->addActivation(t_end+afterTime, false, 0.5, taskHandObjOri);

  a1->add(t_grasp, RCS_DEG2RAD(160.0), 0.0, 0.0, 7, taskHandObjOri + " 0");
  a1->add(t_end, RCS_DEG2RAD(160.0), 0.0, 0.0, 7, taskHandObjOri + " 0");

  // Lift object
  a1->addActivation(t_grasp, true, 0.5, taskObjPosZ);
  a1->addActivation(t_end+ afterTime, false, 0.5, taskObjPosZ);
  a1->add(t_end, liftHeight, 0.0, 0.0, 7, taskObjPosZ + " 0");
  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_grasp, objectName, capabilityFrame));

  // Lift it up in the y-z plane (keep forward position constant)
  if (!handOver)
  {
    a1->addActivation(t_grasp, true, 0.5, taskObjPosX);
    a1->addActivation(t_end+afterTime, false, 0.5, taskObjPosX);
  }

  // Reduce snap to null space y-position by moving up vertically
  a1->addActivation(t_grasp, true, 0.5, taskObjPosY);
  a1->addActivation(t_end+ afterTime, false, 0.5, taskObjPosY);

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
ActionGet::createTrajectoryTopGrasp(double t_start,
                                    double t_grasp,
                                    double t_end) const
{
  const double afterTime = 0.5;

  // Grasp the object and lift it up
  auto a1 = std::make_shared<tropic::ActivationSet>();

  // Hand position with respect to object
  const double t_pregrasp = t_start + 0.5*(t_grasp-t_start);
  a1->addActivation(t_start, true, 0.5, taskObjHandPos);
  a1->addActivation(t_grasp, false, 0.5, taskObjHandPos);
  a1->add(std::make_shared<tropic::PositionConstraint>(t_pregrasp, 0.0, 0.0, 0.5*preGraspDist, taskObjHandPos));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_grasp, 0.0, 0.0, 0.0, taskObjHandPos));

  // Hand orientation with respect to object for the approach motion
  a1->addActivation(t_start, true, 0.5, taskHandObjOri);
  a1->addActivation(t_grasp, false, 0.5, taskHandObjOri);

  a1->add(std::make_shared<tropic::PolarConstraint>(t_grasp, M_PI, 0.0, taskHandObjOri));
  a1->add(std::make_shared<tropic::PolarConstraint>(t_end, M_PI, 0.0, taskHandObjOri));

  // Object orientation with respect to world frame. We keep it upright, but
  // don't enforce the orientation around the up-axis. For something like an
  // apple (Inclination constraint only, we don't need to enforce any object
  // orientation. \todo(MG): Do this.
  a1->addActivation(handOver ? t_start : t_grasp, true, 0.5, taskObjOri);
  a1->addActivation(t_end+afterTime, false, 0.5, taskObjOri);

  // Lift object
  a1->addActivation(handOver ? t_start : t_grasp, true, 0.5, taskObjPosZ);
  a1->addActivation(t_end+afterTime, false, 0.5, taskObjPosZ);

  if (!handOver)
  {
    a1->add(t_end, liftHeight, 0.0, 0.0, 7, taskObjPosZ + " 0");
  }
  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_grasp, objectName, capabilityFrame));

  // Lift it up in the y-z plane (keep forward position constant)
  //if (!handOver)
  {
    a1->addActivation(handOver ? t_start : t_grasp, true, 0.5, taskObjPosX);
    a1->addActivation(t_end+afterTime, false, 0.5, taskObjPosX);
  }

  // Reduce snap to null space y-position by moving up vertically
  a1->addActivation(t_grasp, true, 0.5, taskObjPosY);
  a1->addActivation(t_end+afterTime, false, 0.5, taskObjPosY);

  // HACK
  a1->add(t_end, -0.35, 0.0, 0.0, 7, taskObjPosX + " 0");
  a1->add(t_end, 0.0, 0.0, 0.0, 7, taskObjPosY + " 0");

  // If the object is handed over, we keep the hand-over hand at its position
  if (!taskHandOverHand.empty())
  {
    a1->addActivation(t_grasp, true, 0.5, taskHandOverHand);
    a1->addActivation(t_end, false, 0.5, taskHandOverHand);
  }

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
ActionGet::createTrajectoryRimGrasp(double t_start,
                                    double t_grasp,
                                    double t_end) const
{
  const double afterTime = 0.5;

  // Grasp the object and lift it up
  auto a1 = std::make_shared<tropic::ActivationSet>();

  // Hand position with respect to object
  const double t_pregrasp = t_start + 0.5 * (t_grasp - t_start);
  a1->addActivation(t_start, true, 0.5, taskObjHandPos);
  a1->addActivation(t_grasp, false, 0.5, taskObjHandPos);
  std::vector<double> rimCoords = {0.1, 0.1, 0.0, 0.0};
  a1->add(std::make_shared<tropic::VectorConstraint>(t_pregrasp, rimCoords, taskObjHandPos));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_grasp, rimCoords, taskObjHandPos));

  // Hand inclination with respect to object. \todo(MG): Make inclination depend on object
  a1->addActivation(t_start, true, 0.5, taskHandObjOri);
  a1->addActivation(t_end + afterTime, false, 0.5, taskHandObjOri);
  a1->add(t_grasp, RCS_DEG2RAD(160.0), 0.0, 0.0, 7, taskHandObjOri + " 0");

  // Object orientation with respect to world frame. We keep it upright, but
  // don't enforce the orientation around the up-axis. For something like an
  // apple (Inclination constraint only, we don't need to enforce any object
  // orientation. \todo(MG): Do this.
  a1->addActivation(handOver ? t_start : t_grasp, true, 0.5, taskObjOri);
  a1->addActivation(t_end + afterTime, false, 0.5, taskObjOri);

  // Lift object
  a1->addActivation(handOver ? t_start : t_grasp, true, 0.5, taskObjPosZ);
  a1->addActivation(t_end + afterTime, false, 0.5, taskObjPosZ);

  if (!handOver)
  {
    a1->add(t_end, liftHeight, 0.0, 0.0, 7, taskObjPosZ + " 0");
  }
  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_grasp, objectName, capabilityFrame));

  // Lift it up in the y-z plane (keep forward position constant)
  //if (!handOver)
  {
    a1->addActivation(handOver ? t_start : t_grasp, true, 0.5, taskObjPosX);
    a1->addActivation(t_end + afterTime, false, 0.5, taskObjPosX);
  }

  // Reduce snap to null space y-position by moving up vertically
  a1->addActivation(t_grasp, true, 0.5, taskObjPosY);
  a1->addActivation(t_end + afterTime, false, 0.5, taskObjPosY);

  // HACK
  a1->add(t_end, -0.35, 0.0, 0.0, 7, taskObjPosX + " 0");
  a1->add(t_end, 0.0, 0.0, 0.0, 7, taskObjPosY + " 0");

  // If the object is handed over, we keep the hand-over hand at its position
  if (!taskHandOverHand.empty())
  {
    a1->addActivation(t_grasp, true, 0.5, taskHandOverHand);
    a1->addActivation(t_end, false, 0.5, taskHandOverHand);
  }

  return a1;
}
void ActionGet::print() const
{
  std::cout << "objectName: " << objectName << std::endl;
  std::cout << "capabilityFrame: " << capabilityFrame << std::endl;
  std::cout << "affordanceFrame: " << affordanceFrame << std::endl;
  std::cout << "fingerJoints: " << fingerJoints << std::endl;
  std::cout << "graspType: " << graspTypeToString(graspType) << std::endl;

  REXEC(5)
  {
    auto xmlTasks = createTasksXML();

    for (const auto& t : xmlTasks)
    {
      std::cout << t << std::endl;
    }
  }

  for (size_t i=0; i<affordanceMap.size(); ++i)
  {
    RLOG(0, "Solution %zu: %s - %s", i,
         std::get<0>(affordanceMap[i])->frame.c_str(),
         std::get<1>(affordanceMap[i])->frame.c_str());
    //CapabilityCompare::eval(graph, gmi, 1.0, 1.0));
  }

}

std::vector<std::string> ActionGet::createTasksXML() const
{
  std::vector<std::string> tasks;
  std::string xmlTask;

  if (graspType == GraspType::BallGrasp ||
      graspType == GraspType::TopGrasp)
  {
    // Relative to the object
    xmlTask = "<Task name=\"" + taskObjHandPos + "\" " +
              "controlVariable=\"XYZ\" effector=\"" + capabilityFrame +
              "\" " + "refBdy=\"" + affordanceFrame + "\" />";
  }
  else if (graspType == GraspType::RimGrasp)
  {
    // Double radial-z composite task (4 dimensions)
    xmlTask = "<Task name=\"" + taskObjHandPos + "\" " + "controlVariable=\"Composite\" >\n";
    xmlTask += "  <Task name=\"r1\" controlVariable=\"Radial\" effector=\"hand_right_finger_tip_2\" refBdy=\"" +
               objectName + "\" />\n";
    xmlTask += "  <Task name=\"r2\" controlVariable=\"Radial\" effector=\"hand_right_finger_tip_3\" refBdy=\"" +
               objectName + "\" />\n";
    xmlTask += "  <Task name=\"z1\" controlVariable=\"Z\" effector=\"hand_right_finger_tip_2\" refBdy=\"" +
               objectName + "\" />\n";
    xmlTask += "  <Task name=\"z2\" controlVariable=\"Z\" effector=\"hand_right_finger_tip_3\" refBdy=\"" +
               objectName + "\" />\n";
    xmlTask += "</Task>";
  }
  else
  {
    // taskObjHandPos: XYZ-task with effector=object and refBdy=hand
    xmlTask = "<Task name=\"" + taskObjHandPos + "\" " +
              "controlVariable=\"XYZ\" effector=\"" + affordanceFrame +
              "\" " + "refBdy=\"" + capabilityFrame + "\" />";
  }
  tasks.push_back(xmlTask);

  // taskObjPosition z and sideways velocities
  xmlTask = "<Task name=\"" + taskObjPosX + "\" " +
            "controlVariable=\"X\" effector=\"" + objectName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjPosY + "\" " +
            "controlVariable=\"Y\" effector=\"" + objectName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjPosZ + "\" " +
            "controlVariable=\"Z\" effector=\"" + objectName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjOri + "\" " +
            "controlVariable=\"POLAR\" " + "effector=\"" +
            affordanceFrame + "\" />";
  tasks.push_back(xmlTask);

  // taskHandObjOri
  if (graspType == GraspType::PowerGrasp)
  {
    xmlTask = "<Task name=\"" + taskHandObjOri + "\" " +
              "controlVariable=\"POLAR\" " + "effector=\"" + capabilityFrame + "\" " +
              "refBdy=\"" + affordanceFrame + "\" />";
  }
  else if (graspType == GraspType::TopGrasp)
  {
    xmlTask = "<Task name=\"" + taskHandObjOri + "\" " +
              "controlVariable=\"POLAR\" " + "effector=\"" + capabilityFrame + "\" " +
              "refBdy=\"" + affordanceFrame + "\" axisDirection=\"X\" />";
  }
  else if (graspType == GraspType::BallGrasp || graspType == GraspType::RimGrasp)
  {
    xmlTask = "<Task name=\"" + taskHandObjOri + "\" " +
              "controlVariable=\"Inclination\" " + "effector=\"" + capabilityFrame + "\" " +
              //"refBdy=\"" + affordanceFrame + "\" axisDirection=\"X\" >";
              " axisDirection=\"X\" >";
    // xmlTask += "\n<TaskRegion type=\"BoxInterval\" ";
    // xmlTask += "min=\"" + std::to_string(RCS_DEG2RAD(-10.0)) + "\" ";
    // xmlTask += "max=\"" + std::to_string(RCS_DEG2RAD(10.0)) + "\" ";
    // // xmlTask += "dxScaling=\"0.01\" ";
    // xmlTask += " />\n";
    xmlTask += "</Task>";
    // std::cout << "HERE:\n" << xmlTask << std::endl;
  }
  else
  {
    xmlTask = "<Task name=\"" + taskHandObjOri + "\" " +
              "controlVariable=\"ABC\" " + "effector=\"" + capabilityFrame + "\" " +
              "refBdy=\"" + affordanceFrame + "\" />";
  }
  tasks.push_back(xmlTask);

  // Hand over hand (if any)
  if (!taskHandOverHand.empty())
  {
    xmlTask = "<Task name=\"" + taskHandOverHand + "\" " +
              "controlVariable=\"XYZ\" effector=\"" + handOverHand + "\" />";
    tasks.push_back(xmlTask);
  }

  // Fingers
  xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerJoints + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

std::string ActionGet::explain() const
{
  //return std::string("I'm getting the " + objectName + " with my " + capabilityFrame);
  return std::string("I'm getting the " + objectName);
}

std::vector<std::string> ActionGet::getManipulators() const
{
  return usedManipulators;
}

std::vector<double> ActionGet::getInitOptimState(tropic::TrajectoryControllerBase* tc,
                                                 double duration) const
{
  //return std::vector<double>();
  double pos[3];
  tropic::TrajectoryND* trj = tc->getTrajectory(taskObjHandPos);
  RCHECK_MSG(trj, "Trajectory for task \"%s\" not found", taskObjHandPos.c_str());
  trj->getPosition(0.66*duration, pos);
  std::vector<double> o(pos, pos+3);
  return o;
}

size_t ActionGet::getNumSolutions() const
{
  return affordanceMap.size();
}

double ActionGet::getDurationHint() const
{
  const double timeScaling = (graspType == GraspType::TopGrasp) ? 1.5 : 1.0;
  return defaultDuration;
}

std::string ActionGet::graspTypeToString(GraspType gType)
{
  std::string str;

  switch (gType)
  {
    case GraspType::PowerGrasp:
      str = "PowerGrasp";
      break;

    case GraspType::TopGrasp:
      str = "TopGrasp";
      break;

    case GraspType::BallGrasp:
      str = "BallGrasp";
      break;

    case GraspType::RimGrasp:
      str = "RimGrasp";
      break;

    case GraspType::OrientedGrasp:
      str = "OrientedGrasp";
      break;

    default:
      str = "Other";
  }

  return str;
}

std::unique_ptr<ActionBase> ActionGet::clone() const
{
  return std::make_unique<ActionGet>(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/


class ActionGetAndHold : public ActionGet
{
public:

  ActionGetAndHold(const ActionScene& domain,
                   const RcsGraph* graph,
                   std::vector<std::string> params) : ActionGet()
  {
    liftHeight = 0.0;
    preGraspDist = DEFAULT_PREGRASPDIST;
    handOver = false;
    isObjCollidable = false;

    auto it = std::find(params.begin(), params.end(), "from");
    if (it != params.end())
    {
      whereFrom = *(it+1);
      params.erase(it+1);
      params.erase(it);
    }

    it = std::find(params.begin(), params.end(), "liftHeight");
    if (it != params.end())
    {
      liftHeight = 0.0*std::stod(*(it+1));
      params.erase(it+1);
      params.erase(it);
    }

    std::string objectToGet = params[0];
    std::string manipulator = params.size()>1?params[1]:std::string();

    init(domain, graph, objectToGet, manipulator, whereFrom);
  }

};

REGISTER_ACTION(ActionGetAndHold, "get_and_hold");

}   // namespace aff
