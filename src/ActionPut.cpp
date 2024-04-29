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

#include "ActionPut.h"
#include "ActionFactory.h"
#include "CollisionModelConstraint.h"

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <ConnectBodyConstraint.h>
#include <VectorConstraint.h>

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#define fingersOpen   (0.01)
#define fingersClosed (0.7)
#define t_fingerMove  (2.0)

#define IS_NEAR_THRESHOLD (0.4)


namespace aff
{
template<typename T>
static void eraseAffordancesNearerOrFartherAgent(const ActionScene& domain,
                                                 const RcsGraph* graph,
                                                 const Agent* nearToAgent,
                                                 double d_limit,
                                                 bool eraseIfRechable,
                                                 std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  const RcsBody* agentFrame = nearToAgent->body(graph);
  auto it = affordanceMap.begin();

  while (it != affordanceMap.end())
  {
    const Affordance* stackable = std::get<0>(*it);
    const RcsBody* putFrame = stackable->getFrame(graph);
    bool reachable;

    if (d_limit == 0.0)
    {
      reachable = nearToAgent->canReachTo(&domain, graph, putFrame->A_BI.org);
    }
    else
    {
      reachable = Vec3d_distance(agentFrame->A_BI.org, putFrame->A_BI.org) <= d_limit ? true : false;
    }

    if (eraseIfRechable)
    {
      it = reachable ? affordanceMap.erase(it) : it + 1;
    }
    else
    {
      it = reachable ? it + 1 : affordanceMap.erase(it);
    }
  }

}

template<typename T>
static void eraseAffordancesNearerOrFartherNtt(const ActionScene& domain,
                                               const RcsGraph* graph,
                                               const std::vector<const AffordanceEntity*>& nearToNtts,
                                               double d_limit,
                                               bool trueForNearer,
                                               std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  bool trueForFarther = !trueForNearer;

  // Memorize the frames of all entities that are to be checked against
  // closeness. We store them up front to avoid searching them in each
  // iteration of the affordanceMap.
  std::vector<const RcsBody*> nttFrames;
  for (const auto& ntt : nearToNtts)
  {
    nttFrames.push_back(ntt->body(graph));
  }

  auto it = affordanceMap.begin();
  while (it != affordanceMap.end())
  {
    const Affordance* affordance = std::get<0>(*it);
    const T* supportable = dynamic_cast<const T*>(affordance);

    // We only consider affordances with the given template type
    if (!supportable)
    {
      it++;
      continue;
    }

    // Here we go through all relevant entity frames, and check if any of
    // the entities is below the limit distance. If it is the case, we can
    // safely keep the affordance and do an early exit of the loop.
    const RcsBody* putFrame = supportable->getFrame(graph);
    bool eraseSupportable = trueForFarther;
    for (const auto& nttFrame : nttFrames)
    {
      double di = Vec3d_distance(nttFrame->A_BI.org, putFrame->A_BI.org);
      if (di <= d_limit)
      {
        eraseSupportable = !trueForFarther;
        RLOG(5, "%s - %s is %s (d=%f, limit=%f)",
             nttFrames[0]->name, putFrame->name,
             (eraseSupportable ? "erased" : "not erased"), di, d_limit);
        break;
      }
    }

    it = eraseSupportable ? affordanceMap.erase(it) : it + 1;
  }

}

// Prune T's that are farther away than d_limit
static void eraseSupportablesFarther(const ActionScene& domain,
                                     const RcsGraph* graph,
                                     const std::string& ntt,
                                     double d_limit,
                                     std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  std::vector<const AffordanceEntity*> ntts = domain.getAffordanceEntities(ntt);

  if (!ntts.empty())
  {
    eraseAffordancesNearerOrFartherNtt<Supportable>(domain, graph, ntts, d_limit,
                                                    false, affordanceMap);
  }

  const Agent* agent = domain.getAgent(ntt);

  if (agent)
  {
    eraseAffordancesNearerOrFartherAgent<Supportable>(domain, graph, agent, d_limit,
                                                      false, affordanceMap);
  }

}

// Prune T's that are closer than d_limit
static void eraseSupportablesNearer(const ActionScene& domain,
                                    const RcsGraph* graph,
                                    const std::string& ntt,
                                    double d_limit,
                                    std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  std::vector<const AffordanceEntity*> ntts = domain.getAffordanceEntities(ntt);

  if (!ntts.empty())
  {
    eraseAffordancesNearerOrFartherNtt<Supportable>(domain, graph, ntts, d_limit,
                                                    true, affordanceMap);
  }

  const Agent* agent = domain.getAgent(ntt);

  if (agent)
  {
    eraseAffordancesNearerOrFartherAgent<Supportable>(domain, graph, agent, d_limit,
                                                      true, affordanceMap);
  }
}

}   // namespace

namespace aff
{
REGISTER_ACTION(ActionPut, "put");

ActionPut::ActionPut() :
  putDown(false), isObjCollidable(false), isPincerGrasped(false),
  supportRegionX(0.0), supportRegionY(0.0), polarAxisIdx(2), distance(0.0)
{
  Vec3d_setZero(downProjection);
}

ActionPut::ActionPut(const ActionScene& domain,
                     const RcsGraph* graph,
                     std::vector<std::string> params) : ActionPut()
{
  parseArgs(domain, graph, params);

  std::string objectToPut = params[0];
  std::string surfaceToPutOn = params.size() > 1 ? params[1] : std::string();

  if (objectToPut == surfaceToPutOn)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The " + objectToPut + " cannot be put on itself.",
                          "Put it onto another object in the environment");
  }

  const AffordanceEntity* object = initHands(domain, graph, objectToPut);

  initOptions(domain, graph, object, surfaceToPutOn);

  // Initialize with the best solution.
  bool successInit = initialize(domain, graph, 0);
  RCHECK(successInit);
}

void ActionPut::parseArgs(const ActionScene& domain,
                          const RcsGraph* graph,
                          std::vector<std::string>& params)
{
  parseParams(params);

  auto it = std::find(params.begin(), params.end(), "frame");
  if (it != params.end())
  {
    whereOn = *(it + 1);
    params.erase(it + 1);
    params.erase(it);
  }

  it = std::find(params.begin(), params.end(), "distance");
  if (it != params.end())
  {
    distance = std::stod(*(it + 1));
    params.erase(it + 1);
    params.erase(it);
  }

  it = std::find(params.begin(), params.end(), "near");
  if (it != params.end())
  {
    nearTo = *(it + 1);

    if (!nearTo.empty() && domain.getSceneEntities(nearTo).empty())
    {
      throw ActionException(ActionException::ParamNotFound,
                            "Cannot put an object near " + nearTo + " because " + nearTo + " is unknown",
                            "Put it near another object in the environment");
    }

    params.erase(it + 1);
    params.erase(it);
  }

  it = std::find(params.begin(), params.end(), "far");
  if (it != params.end())
  {
    farFrom = *(it + 1);

    if (!farFrom.empty() && domain.getSceneEntities(farFrom).empty())
    {
      throw ActionException(ActionException::ParamNotFound,
                            "Cannot put an object far from " + farFrom + " because " + farFrom + " is unknown",
                            "Put it far away of another object in the environment");
    }

    params.erase(it + 1);
    params.erase(it);
  }

}

// This function assigns the following member variables:
// - this->objName (initialize)
// - this->isObjCollidable (used in createTrajectory)
// - this->usedManipulators (first one is grasping hand)
// - this->isPincerGrasped (tasks and trajectory generation)
// - this->graspFrame (used in createTasks)
// - this->objGraspFrame (used in createTasks)
const AffordanceEntity* ActionPut::initHands(const ActionScene& domain,
                                             const RcsGraph* graph,
                                             const std::string& objectToPut)
{
  // Initialize object to put
  std::vector<const AffordanceEntity*> objects = domain.getAffordanceEntities(objectToPut);

  if (objects.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The " + objectToPut + " is unknown.",
                          "Use an object name that is defined in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  auto gPair = domain.getGraspingHand(graph, objects);
  const Manipulator* graspingHand = std::get<0>(gPair);
  const AffordanceEntity* object = std::get<1>(gPair);

  if (!graspingHand)
  {
    throw ActionException(ActionException::KinematicallyImpossible,
                          "The " + objectToPut + " is not held in the hand.",
                          "First get the " + objectToPut + " in the hand before performing this command",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  if (getAffordances<Stackable>(object).empty())
  {
    throw ActionException(ActionException::KinematicallyImpossible,
                          "I don't know how to place the " + objectToPut + " on a surface.",
                          "Try to hand it over, or try something else",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  this->objName = object->bdyName;
  this->isObjCollidable = object->isCollideable(graph);
  this->usedManipulators.push_back(graspingHand->name);

  // We keep the other manipulator if it is occupied
  auto occupiedManipulators = domain.getOccupiedManipulators(graph);
  for (auto m : occupiedManipulators)
  {
    if (m->name != graspingHand->name)
    {
      usedManipulators.push_back(m->name);
    }
  }

  // From here on we know that the object is held in a manipulator. We determine the
  // frame from which we retract the hand after we opened the fingers.
  auto graspCapability = graspingHand->getGraspingCapability(graph, object);

  if (!graspCapability)
  {
    throw ActionException(ActionException::ParamInvalid, "Cannot grasp object", "",
                          "graspCapability cannot grasp " + object->name);
  }

  this->isPincerGrasped = dynamic_cast<const PincergraspCapability*>(graspCapability);
  this->graspFrame = graspCapability->frame;
  auto ca = graspingHand->getGrasp(graph, object);

  // Determine the frame of the object at which it is grasped. This is aligned with
  // the hand's grasping frame (not the orientation necessarily if powergrasped \todo(MG)).
  RLOG_CPP(4, "Grasp frame distance is " << std::get<2>(ca));
  const Affordance* a_grasp = std::get<1>(ca);

  if (!a_grasp)
  {
    throw ActionException(ActionException::UnknownError, "Internal error", "",
                          "Internal error: graspingHand->getGrasp failed with entity " + object->name);
  }

  this->objGraspFrame = a_grasp->frame;

  return object;
}

void ActionPut::initOptions(const ActionScene& domain,
                            const RcsGraph* graph,
                            const AffordanceEntity* object,
                            const std::string& surfaceToPutOn)
{
  // Detect surface
  const AffordanceEntity* surface = domain.getAffordanceEntity(surfaceToPutOn);

  // If a non-empty name for the surface has been given, we enforce that it exists
  if ((!surface) && (!surfaceToPutOn.empty()))
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The surface '" + surfaceToPutOn + "' to put the '" + object->bdyName + "' on is unknown.",
                          "Use an object name that is defined in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  // If no surface name was specified, we search through all the scene's Supportables
  if (!surface)
  {
    AffordanceEntity tmp;
    tmp.affordances = getAffordances<Supportable>(&domain);
    affordanceMap = match<Supportable, Stackable>(&tmp, object);
    tmp.affordances.clear();

    // Since we don't have a surface entity, the Supportable can possibly be a child of
    // the Stackable, leading to put the object on a child of itself. This creates
    // trouble which we resolve in the following.
    auto it = affordanceMap.begin();
    while (it != affordanceMap.end())
    {
      const Affordance* supportable = std::get<0>(*it);
      const Affordance* stackable = std::get<1>(*it);
      const RcsBody* supportBdy = RcsGraph_getBodyByName(graph, supportable->frame.c_str());
      const RcsBody* stackBdy = RcsGraph_getBodyByName(graph, stackable->frame.c_str());
      bool eraseMe = RcsBody_isChild(graph, supportBdy, stackBdy);
      it = eraseMe ? affordanceMap.erase(it) : it + 1;
    }

  }
  else
  {
    // From here on, we have a valid object and surface. We determine all
    // combinations of <Supportable, Stackable> affordances. We already know
    // that the object is stackable
    affordanceMap = match<Supportable, Stackable>(surface, object);
  }

  // If there are none, we give up.
  if (affordanceMap.empty())
  {
    std::string surfaceName = surface ? surface->name : "surface";
    throw ActionException(ActionException::ParamNotFound,
                          "I can't put the " + object->bdyName + " on the " + surfaceName + " because it does not support it.",
                          "Specify another object to put it on");
  }

  RLOG_CPP(1, "Affordance map has " << affordanceMap.size() << " entries");

  // Erase the Supportables that are out of reach. We check for the graspingHand, since
  // derived classes (like magic_put or so) don't have a grasping hand.
  const Manipulator* graspingHand = usedManipulators.empty() ? nullptr : domain.getManipulator(usedManipulators[0]);
  if (graspingHand)
  {
    auto it = affordanceMap.begin();
    while (it != affordanceMap.end())
    {
      const Supportable* place = dynamic_cast<const Supportable*>(std::get<0>(*it));
      if (place)
      {
        bool canReach = graspingHand->canReachTo(&domain, graph, place->getFrame(graph)->A_BI.org);
        it = (!canReach) ? affordanceMap.erase(it) : it + 1;
        if (!canReach)
        {
          RLOG_CPP(4, "Erasing " << place->frame << " - out of reach");
        }
      }
      else
      {
        it++;
      }
    }
    RLOG_CPP(1, "Done erase Supportables out of reach. Remaining: " << affordanceMap.size());
  }

  // Erase the Supportables that don't match the "whereOn" name if it has been specified
  if (!whereOn.empty())
  {
    auto it = affordanceMap.begin();
    while (it != affordanceMap.end())
    {
      const Supportable* s = dynamic_cast<const Supportable*>(std::get<0>(*it));
      const bool eraseMe = !(s && (s->frame == whereOn));
      NLOG(0, "%s Supportable %s", eraseMe ? "Erasing" : "Keeping", s->frame.c_str());
      it = eraseMe ? affordanceMap.erase(it) : it+1;
    }
  }
  RLOG_CPP(1, "Done erase Supportables on-top (frame: '" << whereOn << "'). Remaining: "
           << affordanceMap.size());

  // Erase the Supportables that are farther away from an entity or agent than d_limit
  if (!nearTo.empty())
  {
    bool isAgent = domain.getAgent(nearTo);
    double d_limit = ((!isAgent) && (distance == 0.0)) ? IS_NEAR_THRESHOLD : distance;
    eraseSupportablesFarther(domain, graph, nearTo, d_limit, affordanceMap);
  }
  RLOG_CPP(1, "Done erase Supportables farther than. Remaining: " << affordanceMap.size());

  // Erase the Supportables that are more close to an entity or agent than d_limit
  if (!farFrom.empty())
  {
    bool isAgent = domain.getAgent(farFrom);
    double d_limit = ((!isAgent) && (distance == 0.0)) ? IS_NEAR_THRESHOLD : distance;
    eraseSupportablesNearer(domain, graph, farFrom, d_limit, affordanceMap);
  }
  RLOG_CPP(1, "Done erase Supportables nearer than. Remaining: " << affordanceMap.size());

  // If the map is empty after removing all non-frame Supportables, we give up.
  if (affordanceMap.empty())
  {
    std::string surfaceName = surface ? surface->name : "surface";
    throw ActionException(ActionException::ParamNotFound,
                          "I can't put the " + object->bdyName + " on the " + surfaceName + "'s frame " + whereOn + ".",
                          "", "The affordance map is empty after removing all non-frame supportables");
  }

  // Erase the Supportables that are already occupied with a collideable
  {
    auto it = affordanceMap.begin();
    while (it != affordanceMap.end())
    {
      // If element is even number then delete it
      const Affordance* supportable = std::get<0>(*it);
      const Supportable* s = dynamic_cast<const Supportable*>(supportable);
      RCHECK(s);

      // \todo: Make better geometric check here
      if (s->extentsX>0.0 || s->extentsY>0.0)
      {
        it++;
        continue;
      }

      // Traverse children of support frame and look for collideable entities
      const RcsBody* supportFrame = supportable->getFrame(graph);
      RcsBody* child = RCSBODY_BY_ID(graph, supportFrame->firstChildId);
      bool foundCollideable = false;

      while (child)
      {
        const AffordanceEntity* childNTT = domain.getAffordanceEntity(child->name);

        if (childNTT && childNTT->isCollideable(graph))
        {
          foundCollideable = true;
          break;
        }

        child = RCSBODY_BY_ID(graph, child->nextId);
      }

      it = foundCollideable ? affordanceMap.erase(it) : it + 1;
    }
  }
  RLOG_CPP(1, "Done erase Supportables that are already occupied with a collideable. Remaining: " << affordanceMap.size());

  // There's already something on all supportables
  if (affordanceMap.empty())
  {
    std::string surfaceName = surface ? surface->name : "surface";
    throw ActionException(ActionException::ParamNotFound,
                          "I can't put the " + object->bdyName + " on the " + surfaceName + ". There is already something on it.",
                          "Put the object somewhere else, or remove the blocking object.");
  }

  // Create a vector of tuples with the third argument being the cost to be sorted for
  RLOG(5, "Creating sortMap");
  std::vector<std::tuple<Affordance*, Affordance*,double>> sortMap;
  for (auto& pair : affordanceMap)
  {
    Affordance* place = std::get<0>(pair);
    sortMap.push_back(std::make_tuple(place, std::get<1>(pair),
                                      actionCost(domain, graph, place->frame)));
  }

  // Sort with lambda compare function, lower cost at the beginning
  RLOG(5, "Sorting sortMap");
  std::sort(sortMap.begin(), sortMap.end(),
            [](std::tuple<Affordance*, Affordance*, double>& a,
               std::tuple<Affordance*, Affordance*, double>& b)
  {
    return std::get<2>(a) < std::get<2>(b);
  });

  // Copy back to affordance map
  RLOG(5, "Recomputing affordanceMap");
  affordanceMap.clear();
  for (auto& pair : sortMap)
  {
    RLOG_CPP(1, "Sorted: " << std::get<0>(pair)->frame << " - "
             << std::get<1>(pair)->frame << " : " << std::get<2>(pair));
    affordanceMap.push_back(std::make_tuple(std::get<0>(pair), std::get<1>(pair)));
  }
  RLOG_CPP(1, "Done initOptions() with heuristic re-sorting affordance map with "
           << affordanceMap.size() << " entries");
}

ActionPut::~ActionPut()
{
}

bool ActionPut::initialize(const ActionScene& domain,
                           const RcsGraph* graph,
                           size_t solutionRank)
{
  if (solutionRank >= affordanceMap.size())
  {
    return false;
  }

  // These are the "winning" affordances.
  Affordance* surfaceAff = std::get<0>(affordanceMap[solutionRank]);
  RCHECK_MSG(surfaceAff, "Affordance at solution index %zu is NULL", solutionRank);
  Supportable* supportable = dynamic_cast<Supportable*>(surfaceAff);
  RCHECK_MSG(supportable, "%s is not a Supportable", surfaceAff->frame.c_str());   // never happens
  surfaceFrameName = surfaceAff->frame;
  supportRegionX = supportable->extentsX;
  supportRegionY = supportable->extentsY;

  Affordance* bottomAff = std::get<1>(affordanceMap[solutionRank]);
  RCHECK_MSG(bottomAff, "affordanceMap.size()=%zu, solutionRank=%zu",
             affordanceMap.size(), solutionRank);   // never happens
  Stackable* stackable = dynamic_cast<Stackable*>(bottomAff);
  RCHECK_MSG(stackable, "'%s' has no Stackable affordance",
             bottomAff->frame.c_str());   // never happens
  this->polarAxisIdx = stackable->normalDir;

  this->objBottomName = bottomAff->frame;

  // This is guaranteed to exist after the scene class's check
  const RcsBody* surfaceBdy = resolveBodyName(graph, surfaceFrameName);
  RCHECK_MSG(surfaceBdy, "%s", surfaceFrameName.c_str());

  // This is the put position in the coordinates of the surface frame.
  Vec3d_invTransform(downProjection, &surfaceBdy->A_BI, surfaceBdy->A_BI.org);

  // Assemble finger task name. usedManipulators may be empty for derived classes
  if (!usedManipulators.empty())
  {
    const Manipulator* graspingHand = domain.getManipulator(usedManipulators[0]);

    fingerJoints.clear();
    for (auto& f : graspingHand->fingerJoints)
    {
      fingerJoints += f;
      fingerJoints += " ";
    }

    handOpen = std::vector<double>(graspingHand->getNumFingers(), fingersOpen);
  }

  // Task naming
  this->taskObjHandPos = objGraspFrame + "-" + graspFrame + "-XYZ";
  this->taskHandSurfacePos = graspFrame + "-" + surfaceFrameName + "-XYZ";
  this->taskObjSurfacePosX = objBottomName + "-" + surfaceFrameName + "-X";
  this->taskObjSurfacePosY = objBottomName + "-" + surfaceFrameName + "-Y";
  this->taskObjSurfacePosZ = objBottomName + "-" + surfaceFrameName + "-Z";
  this->taskObjSurfacePolar = objBottomName + "-" + surfaceFrameName + "-POLAR";
  this->taskHandPolar = graspFrame + "-POLAR";
  this->taskHandObjPolar = graspFrame + "-" + objBottomName + "-POLAR";
  this->taskSurfaceOri = surfaceFrameName + "-POLAR";
  this->taskFingers = graspFrame + "_fingers";

  //auto surfNtt = domain.getParentAffordanceEntity(graph, surfaceBdy);
  auto surfNtt = domain.getAffordanceEntity(RCSBODY_NAME_BY_ID(graph, surfaceBdy->parentId));
  detailedActionCommand = "put " + objName;
  if (surfNtt)
  {
    detailedActionCommand += " " + surfNtt->bdyName;
  }

  detailedActionCommand += " frame " + surfaceFrameName;
  // We add the duration in the getActionCommand() function, since initialize() is not
  // called after predict(), and we might adapt the duration after that.
  return true;
}

std::vector<std::string> ActionPut::createTasksXML() const
{
  std::vector<std::string> tasks;

  bool useTaskRegion = false;
  if (((supportRegionX != 0.0) || (supportRegionY != 0.0)))
  {
    useTaskRegion = true;
  }

  // taskObjHandPos: XYZ-task with effector=object and refBdy=hand
  std::string xmlTask = "<Task name=\"" + taskObjHandPos + "\" " +
                        "controlVariable=\"XYZ\" " + "effector=\"" +
                        objGraspFrame + "\" " + "refBdy=\"" + graspFrame + "\" />";
  tasks.push_back(xmlTask);

  // taskHandSurfacePos: XYZ-task with effector=hand, refBdy=object and refFrame=surface
  // Used for vertical retract after ballgrasp
  xmlTask = "<Task name=\"" + taskHandSurfacePos + "\" " +
            "controlVariable=\"XYZ\" " + "effector=\"" +
            graspFrame + "\" " + "refBdy=\"" + objGraspFrame + "\" refFrame=\"" +
            surfaceFrameName + "\" />";
  tasks.push_back(xmlTask);

  // taskObjSurfacePosition z and sideways velocities
  xmlTask = "<Task name=\"" + taskObjSurfacePosX + "\" " +
            "controlVariable=\"X\" " + "effector=\"" + objBottomName + "\" " +
            "refBdy=\"" + surfaceFrameName + "\" >";
  if (useTaskRegion)
  {
    xmlTask += "\n<TaskRegion type=\"BoxInterval\" ";
    xmlTask += "min=\"" + std::to_string(-0.5*supportRegionX) + "\" ";
    xmlTask += "max=\"" + std::to_string(0.5*supportRegionX) + "\" ";
    xmlTask += "dxScaling=\"0\" />";
  }
  xmlTask += "</Task>";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjSurfacePosY+ "\" " +
            "controlVariable=\"Y\" " + "effector=\"" + objBottomName + "\" " +
            "refBdy=\"" + surfaceFrameName + "\" >";
  if (useTaskRegion)
  {
    xmlTask += "\n<TaskRegion type=\"BoxInterval\" ";
    xmlTask += "min=\"" + std::to_string(-0.5*supportRegionY) + "\" ";
    xmlTask += "max=\"" + std::to_string(0.5*supportRegionY) + "\" ";
    xmlTask += "dxScaling=\"0\" />";
  }
  xmlTask += "</Task>";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjSurfacePosZ + "\" " +
            "controlVariable=\"Z\" " + "effector=\"" + objBottomName + "\" " +
            "refBdy=\"" + surfaceFrameName + "\" />";
  tasks.push_back(xmlTask);

  // taskObjSurfacePolar: Relative Polar angle orientation between
  // object and surface   polarAxisIdx
  xmlTask = "<Task name=\"" + taskObjSurfacePolar + "\" " +
            "controlVariable=\"POLAR\" " + "effector=\"" +
            objBottomName + "\" refBdy=\"" + surfaceFrameName + "\" ";

  if (polarAxisIdx==0)
  {
    xmlTask += "axisDirection=\"X\" ";
  }
  else if (polarAxisIdx == 1)
  {
    xmlTask += "axisDirection=\"Y\" ";
  }

  xmlTask += " />";


  tasks.push_back(xmlTask);

  // taskHandPolar
  xmlTask = "<Task name=\"" + taskHandPolar + "\" " +
            "controlVariable=\"Inclination\" " + "effector=\"" + graspFrame + "\" axisDirection=\"X\" />";
  tasks.push_back(xmlTask);

  // taskHandObjPolar
  if (isPincerGrasped)
  {
    xmlTask = "<Task name=\"" + taskHandObjPolar + "\" " +
              "controlVariable=\"Inclination\" " + "effector=\"" + graspFrame + "\" " +
              "refBdy=\"" + objBottomName + "\" axisDirection=\"X\" />";
  }
  else
  {
    xmlTask = "<Task name=\"" + taskHandObjPolar + "\" " +
              "controlVariable=\"POLAR\" " + "effector=\"" + graspFrame + "\" " +
              "refBdy=\"" + objBottomName + "\" />";
  }
  tasks.push_back(xmlTask);

  // Orientation of surface frame (if held in hand)
  xmlTask = "<Task name=\"" + taskSurfaceOri + "\" " +
            "controlVariable=\"POLAR\" " + "effector=\"" + surfaceFrameName + "\" />";
  tasks.push_back(xmlTask);


  // Fingers
  xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerJoints + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionPut::createTrajectory(double t_start, double t_end) const
{
  const double t_put = t_start + 0.66 * (t_end - t_start);

  auto a1 = std::make_shared<tropic::ActivationSet>();
  a1->add(createTrajectory(t_start, t_put, t_end));

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
ActionPut::createTrajectory(double t_start,
                            double t_put,
                            double t_release) const
{
  const double afterTime = 0.5;

  // Grasp the bottle and lift it up
  auto a1 = std::make_shared<tropic::ActivationSet>();


  a1->addActivation(t_start, true, 0.5, taskSurfaceOri);
  a1->addActivation(t_release, false, 0.5, taskSurfaceOri);

  // Put object on surface
  a1->addActivation(t_start, true, 0.5, taskObjSurfacePosX);
  a1->addActivation(t_put, false, 0.5, taskObjSurfacePosX);
  a1->addActivation(t_start, true, 0.5, taskObjSurfacePosY);
  a1->addActivation(t_put, false, 0.5, taskObjSurfacePosY);
  a1->addActivation(t_start, true, 0.5, taskObjSurfacePosZ);
  a1->addActivation(t_put, false, 0.5, taskObjSurfacePosZ);

  if (getOptimDim()==3)
  {
    const double t_optim = t_start + 0.33 * (t_release - t_start);
    auto oState = getOptimState();
    RLOG(1, "Adding optimization constraint [%.4f %.4f %.4f] at t=%f",
         oState[0], oState[1], oState[2], t_optim);
    a1->add(t_optim, oState[0], 0.0, 0.0, 1, taskObjSurfacePosX + " 0");
    a1->add(t_optim, oState[1], 0.0, 0.0, 1, taskObjSurfacePosY + " 0");
    a1->add(t_optim, oState[2], 0.0, 0.0, 1, taskObjSurfacePosZ + " 0");
  }

  const double* x = downProjection;

  a1->add(t_put, x[0], 0.0, 0.0, 7, taskObjSurfacePosX + " 0");
  a1->add(t_put, x[1], 0.0, 0.0, 7, taskObjSurfacePosY + " 0");
  a1->add(t_put, x[2], 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");

  // Move in a little bit of a curve above other objects
  if (!putDown)
  {
    a1->add(0.75*(t_start+t_put), 0.1, 0.0, 0.0, 1, taskObjSurfacePosZ + " 0");
  }

  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_put, objName, surfaceFrameName));

  // Retract hand from object
  if (isPincerGrasped)
  {
    const double releaseUp = 0.15;
    a1->addActivation(t_put, true, 0.5, taskHandSurfacePos);
    a1->addActivation(t_release + afterTime, false, 0.5, taskHandSurfacePos);
    a1->add(t_release, releaseUp, 0.0, 0.0, 7, taskHandSurfacePos + " 2");
    a1->addActivation(t_put, true, 0.5, taskHandPolar);
    a1->addActivation(t_put + 0.5*(t_release-t_put), false, 0.5, taskHandPolar);
  }
  else
  {
    const double releaseDistance = 0.15;
    const double releaseUp = -0.05;
    a1->addActivation(t_put, true, 0.5, taskObjHandPos);
    a1->addActivation(t_release + 0*afterTime, false, 0.5, taskObjHandPos);
    a1->add(t_release, releaseDistance, 0.0, 0.0, 7, taskObjHandPos + " 0");
    a1->add(t_release, releaseUp, 0.0, 0.0, 7, taskObjHandPos + " 2");
  }

  // Object orientation wrt world frame. The object is re-connected to the
  // table at t=t_put. Therefore we must switch of the hand-object relative
  // orientation to avoid conflicting constraints. If we want the hand to
  // remain upright a bit longer, we would need to activate an orientation
  // task that is absolute with respect to the hand, for instance taskHandObjPolar.
  a1->addActivation(t_start, true, 0.5, taskObjSurfacePolar);
  a1->addActivation(t_put, false, 0.5, taskObjSurfacePolar);
  a1->add(std::make_shared<tropic::PolarConstraint>(t_put, 0.0, 0.0, taskObjSurfacePolar));

  if (!isPincerGrasped)
  {
    a1->addActivation(t_put, true, 0.5, taskHandObjPolar);
    a1->addActivation(t_put + 0.5 * (t_release - t_put), false, 0.5, taskHandObjPolar);
  }

  // Open fingers. The fingers are not affected by any null space gradient,
  // therefore ther angles don't change without activation. We use this
  // to activate them only for the opening phase.
  a1->addActivation(t_put-0.5 * t_fingerMove, true, 0.5, taskFingers);
  a1->addActivation(t_release, false, 0.5, taskFingers);


  //a1->add(std::make_shared<tropic::VectorConstraint>(t_release, std::vector<double> {fingersOpen, fingersOpen, fingersOpen}, taskFingers));
  if (!handOpen.empty())
  {
    a1->add(std::make_shared<tropic::VectorConstraint>(t_put+0.5 * t_fingerMove, handOpen, taskFingers));
  }

  // Deactivate object collisions when released, and re-activate once the hand has been retracted.
  if (isObjCollidable)
  {
    a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_put, objBottomName, false));
    a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_release, objBottomName, true));
  }

  return a1;
}

double ActionPut::getDurationHint() const
{
  const double timeScaling = putDown ? 0.6 : 1.0;
  return timeScaling*ActionBase::getDurationHint();
}

std::vector<std::string> ActionPut::getManipulators() const
{
  return usedManipulators;
}

std::vector<double> ActionPut::getInitOptimState(tropic::TrajectoryControllerBase* tc,
                                                 double duration) const
{
  const double t_optim = 0.33 *duration;
  double pos[3];
  tc->getTrajectory(taskObjSurfacePosX)->getPosition(t_optim, pos+0);
  tc->getTrajectory(taskObjSurfacePosY)->getPosition(t_optim, pos+1);
  tc->getTrajectory(taskObjSurfacePosZ)->getPosition(t_optim, pos+2);
  std::vector<double> o(pos, pos+3);
  return o;
}

void ActionPut::print() const
{
  std::cout << "surfaceFrameName: " << surfaceFrameName << std::endl;
  std::cout << "taskObjHandPos: " << taskObjHandPos << std::endl;
  std::cout << "taskObjSurfacePosX: " << taskObjSurfacePosX << std::endl;
  std::cout << "taskObjSurfacePosY: " << taskObjSurfacePosY << std::endl;
  std::cout << "taskObjSurfacePosZ: " << taskObjSurfacePosZ << std::endl;
  std::cout << "taskObjSurfacePolar: " << taskObjSurfacePolar << std::endl;
  std::cout << "taskHandObjPolar: " << taskHandObjPolar << std::endl;
  std::cout << "taskFingers: " << taskFingers << std::endl;

  ActionBase::print();

  for (size_t i = 0; i < affordanceMap.size(); ++i)
  {
    RLOG(0, "Solution %zu: %s - %s", i,
         std::get<0>(affordanceMap[i])->frame.c_str(),
         std::get<1>(affordanceMap[i])->frame.c_str());
  }

}

size_t ActionPut::getNumSolutions() const
{
  return affordanceMap.size();
}

std::unique_ptr<ActionBase> ActionPut::clone() const
{
  return std::make_unique<ActionPut>(*this);
}

double ActionPut::actionCost(const ActionScene& domain,
                             const RcsGraph* graph) const
{
  return actionCost(domain, graph, surfaceFrameName);
}

double ActionPut::actionCost(const ActionScene& domain,
                             const RcsGraph* graph,
                             const std::string& place) const
{
  double cost = 0.0;

  if (!nearTo.empty())
  {
    const RcsBody* putLocation = RcsGraph_getBodyByName(graph, place.c_str());
    RCHECK_MSG(putLocation, "'%s' unknown", place.c_str());
    auto ntts = domain.getSceneEntities(nearTo);
    //RLOG(0, "Found %zu SceneEntities", ntts.size());
    double sumD = 0.0;
    for (const auto& ntt : ntts)
    {
      const RcsBody* nttLocation = ntt->body(graph);
      sumD += Vec3d_distance(putLocation->A_BI.org, nttLocation->A_BI.org);
      //RLOG(0, "Adding action cost for %s - %s: %f", nttLocation->name, place.c_str(), sumD);
    }
    cost += sumD / (1.0 + sumD);
  }

  if (!farFrom.empty())
  {
    const RcsBody* putLocation = RcsGraph_getBodyByName(graph, place.c_str());
    RCHECK_MSG(putLocation, "'%s' unknown", place.c_str());
    auto ntts = domain.getSceneEntities(farFrom);
    double sumD = 0.0;
    for (const auto& ntt : ntts)
    {
      const RcsBody* nttLocation = ntt->body(graph);
      sumD += Vec3d_distance(putLocation->A_BI.org, nttLocation->A_BI.org);
    }
    cost += 1.0 / (1.0 + sumD);
  }

  return cost;
}

std::string ActionPut::getActionCommand() const
{
  return detailedActionCommand + " duration " + std::to_string(getDurationHint());
}











/*******************************************************************************
 *
 ******************************************************************************/
class ActionMagicPut : public ActionPut
{
public:

  ActionMagicPut(const ActionScene& domain,
                 const RcsGraph* graph,
                 std::vector<std::string> params) : ActionPut()
  {
    defaultDuration = 0.1;

    parseArgs(domain, graph, params);

    objName = params[0];
    std::string surfaceToPutOn = params.size() > 1 ? params[1] : std::string();
    const AffordanceEntity* object = domain.getAffordanceEntity(objName);
    initOptions(domain, graph, object, surfaceToPutOn);

    // Initialize with the best solution.
    bool successInit = initialize(domain, graph, 0);
    RCHECK(successInit);
  }

  virtual std::vector<std::string> createTasksXML() const
  {
    std::vector<std::string> tasks;

    // Dummy task with no meaning, just needed for the activation points
    std::string xmlTask = "<Task name=\"dummy\" controlVariable=\"XYZ\" effector=\"" +
                          surfaceFrameName + "\" />";
    tasks.push_back(xmlTask);

    return tasks;
  }

  virtual std::shared_ptr<tropic::ConstraintSet> createTrajectory(double t_start, double t_end) const
  {
    auto a1 = std::make_shared<tropic::ActivationSet>();

    auto cbc = std::make_shared<tropic::ConnectBodyConstraint>(t_start, objName, surfaceFrameName);
    cbc->setConnectTransform(HTr_identity());
    a1->add(cbc);

    a1->addActivation(t_start, true, 0.5, "dummy");
    a1->addActivation(t_end, false, 0.5, "dummy");

    return a1;
  }

  std::unique_ptr<ActionBase> clone() const
  {
    return std::make_unique<ActionMagicPut>(*this);
  }

  std::string getActionCommand() const
  {
    return "magic_" + ActionPut::getActionCommand();
  }

};

REGISTER_ACTION(ActionMagicPut, "magic_put");

}   // namespace aff
