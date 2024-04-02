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
                                                 const std::string& nearTo,
                                                 double d_limit,
                                                 bool eraseIfRechable,
                                                 std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  const Agent* nearToAgent = domain.getAgent(nearTo);

  if (!nearToAgent)
  {
    return;
  }

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
                                               const std::string& nearTo,
                                               double d_limit,
                                               bool trueForFarther,
                                               std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  // Erase the Affordances of type T that are not near to an entity
  auto nearToNtts = domain.getAffordanceEntities(nearTo);

  // This early exit is important, since the below loop assumes
  // at least one entry.
  if (nearToNtts.empty())
  {
    return;
  }

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
    double di;
    for (const auto& nttFrame : nttFrames)
    {
      di = Vec3d_distance(nttFrame->A_BI.org, putFrame->A_BI.org);
      if (di <= d_limit)
      {
        eraseSupportable = !trueForFarther;
        break;
      }
    }

    RLOG(0, "%s - %s is %s (d=%f, limit=%f)",
         nttFrames[0]->name, putFrame->name,
         (eraseSupportable ? "erased" : "not erased"), di, d_limit);
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
  eraseAffordancesNearerOrFartherNtt<Supportable>(domain, graph, ntt, d_limit,
                                                  true, affordanceMap);

  eraseAffordancesNearerOrFartherAgent<Supportable>(domain, graph, ntt, d_limit,
                                                    false, affordanceMap);
}

// Prune T's that are closer than d_limit
static void eraseSupportablesNearer(const ActionScene& domain,
                                    const RcsGraph* graph,
                                    const std::string& ntt,
                                    double d_limit,
                                    std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  eraseAffordancesNearerOrFartherNtt<Supportable>(domain, graph, ntt, d_limit,
                                                  false, affordanceMap);

  eraseAffordancesNearerOrFartherAgent<Supportable>(domain, graph, ntt, d_limit,
                                                    true, affordanceMap);
}

}   // namespace

namespace aff
{
REGISTER_ACTION(ActionPut, "put");

ActionPut::ActionPut() :
  putDown(false), isObjCollidable(false), isPincerGrasped(false),
  supportRegionX(0.0), supportRegionY(0.0), polarAxisIdx(2), distance(0.0)
{
}

ActionPut::ActionPut(const ActionScene& domain,
                     const RcsGraph* graph,
                     std::vector<std::string> params) : ActionPut()
{
  auto it = std::find(params.begin(), params.end(), "frame");
  if (it != params.end())
  {
    whereOn = *(it+1);
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
    params.erase(it + 1);
    params.erase(it);
  }

  it = std::find(params.begin(), params.end(), "far");
  if (it != params.end())
  {
    farFrom = *(it + 1);
    params.erase(it + 1);
    params.erase(it);
  }

  init(domain, graph, params[0], params.size() > 1 ? params[1] : std::string());
}

void ActionPut::init(const ActionScene& domain,
                     const RcsGraph* graph,
                     const std::string& objectToPut,
                     const std::string& surfaceToPutOn)
{
  RLOG_CPP(1, "Calling put with object='" << objectToPut << "' surface='"
           << surfaceToPutOn << "'");

  if (objectToPut == surfaceToPutOn)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The " + objectToPut + " cannot be put on itself.",
                          "Put it onto another object in the environment");
  }

  Vec3d_setZero(downProjection);

  // Initialize object to put
  std::vector<const AffordanceEntity*> objects = domain.getAffordanceEntities(objectToPut);

  if (objects.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The " + objectToPut + " is unknown.",
                          "Use an object name that is defined in the environment");
  }

  auto gPair = domain.getGraspingHand(graph, objects);
  const Manipulator* graspingHand = std::get<0>(gPair);
  const AffordanceEntity* object = std::get<1>(gPair);

  if (!graspingHand)
  {
    throw ActionException(ActionException::KinematicallyImpossible,
                          "The " + objectToPut + " is not held in the hand.",
                          "First get the " + objectToPut + " in the hand before performing this command");
  }

  if (getAffordances<Stackable>(object).empty())
  {
    throw ActionException(ActionException::KinematicallyImpossible,
                          "I don't know how to place the " + objectToPut + " on a surface.",
                          "Try to hand it over, or try something else");
  }

  objName = object->bdyName;
  isObjCollidable = object->isCollideable(graph);
  usedManipulators.push_back(graspingHand->name);

  // We keep the other manipulator if it is occupied
  auto occupiedManipulators = domain.getOccupiedManipulators(graph);
  for (auto m : occupiedManipulators)
  {
    if (m->name!= graspingHand->name)
    {
      usedManipulators.push_back(m->name);
    }
  }

  // From here on we know that the object is held in a manipulator. We determine the
  // frame from which we retract the hand after we opened the fingers.
  auto graspCapability = graspingHand->getGraspingCapability(graph, object);
  isPincerGrasped = dynamic_cast<const PincergraspCapability*>(graspCapability);
  graspFrame = graspCapability->frame;//graspingHand->getGraspingFrame(graph, object);
  auto ca = graspingHand->getGrasp(graph, object);

  // Determine the frame of the object at which it is grasped. This is aligned with
  // the hand's grasping frame (not the orientation necessarily if powergrasped \todo(MG)).
  RLOG_CPP(1, "Grasp frame distance is " << std::get<2>(ca));
  const Affordance* a_grasp = std::get<1>(ca);

  if (!a_grasp)
  {
    throw ActionException(ActionException::UnknownError, "Internal error", "",
                          "Internal error: graspingHand->getGrasp failed with entity " + object->name);
  }

  objGraspFrame = a_grasp->frame;

  // Detect surface
  const AffordanceEntity* surface = domain.getAffordanceEntity(surfaceToPutOn);

  // If a non-empty name for the surface has been given, we enforce that it exists
  if ((!surface) && (!surfaceToPutOn.empty()))
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The surface " + surfaceToPutOn + " to put the " + objectToPut + " on is unknown.",
                          "Use an object name that is defined in the environment");
  }

  // If no surface name was specified, we search the surface by raycasting.
  if (!surface)
  {
    // Detect surface by down projecting a ray from the object's frame
    HTr surfTransform;
    std::string errMsg;
    surface = raycastSurface(domain, object, graph, &surfTransform, errMsg);

    // If we still didn't find a surface, we give up.
    if (!surface)
    {
      throw ActionException(ActionException::KinematicallyImpossible,
                            "There is nothing under the " + objectToPut + " where I can put it on.",
                            "Specify another object to put it on");
    }
    else
    {
      putDown = true;
    }
  }

  // From here on, we have a valid object and surface. We determine all
  // combinations of <Supportable, Stackable> affordances. We already know
  // that the object is stackable
  affordanceMap = match<Supportable, Stackable>(surface, object);

  // If there are none, we give up.
  if (affordanceMap.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "I can't put the " + objectToPut + " on the " + surface->name + " because it does not support it.",
                          "Specify another object to put it on");
  }

  // Erase the Supportables that don't match the "whereOn" name if it has been specified
  if (!whereOn.empty())
  {
    auto it = affordanceMap.begin();
    while (it != affordanceMap.end())
    {
      const Stackable* stackable = dynamic_cast<const Stackable*>(std::get<0>(*it));
      it = (stackable&&(stackable->frame==whereOn)) ? affordanceMap.erase(it) : it+1;
    }
  }

  // Erase the Supportables that are farther away from an entity or agent than d_limit
  double d_limit = (distance == 0.0) ? IS_NEAR_THRESHOLD : distance;
  if (!nearTo.empty())
  {
    eraseSupportablesFarther(domain, graph, nearTo, d_limit, affordanceMap);
  }

  // Erase the Supportables that are more close to an entity or agent than d_limit
  if (!farFrom.empty())
  {
    eraseSupportablesNearer(domain, graph, farFrom, d_limit, affordanceMap);
  }

  // If the map is empty after removing all non-frame Supportables, we give up.
  if (affordanceMap.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "I can't put the " + objectToPut + " on the " + surface->name + "'s frame " + whereOn + ".",
                          "", "The affordance map is empty after removing all non-frame supportables");
  }

  // Erase the Supportables that are already occupied with a collideable
  {
    // RLOG_CPP(-1, "BEFORE: " << affordanceMap.size());

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

      const RcsBody* supportFrame = RcsGraph_getBodyByName(graph, supportable->frame.c_str());
      RCHECK(supportFrame);
      // RLOG(-1, "Checking supportFrame %s", supportFrame->name);

      // Traverse children of support frame and look for collideable entities
      RcsBody* child = RCSBODY_BY_ID(graph, supportFrame->firstChildId);
      bool foundCollideable = false;

      while (child)
      {
        const AffordanceEntity* childNTT = domain.getAffordanceEntity(child->name);
        // RLOG(-1, "Checking  %s %s", child->name, childNTT ? childNTT->bdyName.c_str() : "NULL");

        if (childNTT && childNTT->isCollideable(graph))
        {
          foundCollideable = true;
          // RLOG(-1, "Found collideable %s", childNTT->bdyName.c_str());
          break;
        }

        child = RCSBODY_BY_ID(graph, child->nextId);
      }


      if (foundCollideable)
      {
        // RLOG(-1, "Found collideable ntt %s - removing", child->name);
        // Due to deletion in loop, iterator became invalidated. So reset the iterator to next item.
        it = affordanceMap.erase(it);
      }
      else
      {
        it++;
      }



    }

    // RLOG_CPP(-1, "AFTER: " << affordanceMap.size());
  }

  // There's already something on all supportables
  if (affordanceMap.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "I can't put the " + objectToPut + " on the " + surface->name + ". There is already something on it.",
                          "Put the object somewhere else, or remove the blocking object.");
  }

  // We have one or several matches and sort them according to their cost. Lower
  // costs pairs are at the beginning.
  sort(graph, affordanceMap, 1.0, 1.0);

  // Initialize with the best solution.
  bool successInit = initialize(domain, graph, 0);
  RCHECK(successInit);
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

  // This is the put position in the coordinates of the surface frame.
  Vec3d_invTransform(downProjection, &surfaceBdy->A_BI, surfaceBdy->A_BI.org);

  // Assemble finger task name
  fingerJoints.clear();
  const Manipulator* graspingHand = domain.getManipulator(usedManipulators[0]);
  for (auto& f : graspingHand->fingerJoints)
  {
    fingerJoints += f;
    fingerJoints += " ";
  }

  handOpen = std::vector<double>(graspingHand->getNumFingers(), fingersOpen);

  // Task naming
  this->taskObjHandPos = /*objBottomName*/ objGraspFrame + "-" + graspFrame + "-XYZ";
  this->taskHandSurfacePos = graspFrame + "-" + surfaceFrameName + "-XYZ";
  this->taskObjSurfacePosX = objBottomName + "-" + surfaceFrameName + "-X";
  this->taskObjSurfacePosY = objBottomName + "-" + surfaceFrameName + "-Y";
  this->taskObjSurfacePosZ = objBottomName + "-" + surfaceFrameName + "-Z";
  this->taskObjSurfacePolar = objBottomName + "-" + surfaceFrameName + "-POLAR";
  this->taskHandPolar = graspFrame + "-POLAR";
  this->taskHandObjPolar = graspFrame + "-" + objBottomName + "-POLAR";
  this->taskSurfaceOri = surfaceFrameName + "-POLAR";
  this->taskFingers = graspFrame + "_fingers";

  explanation = "I'm putting the " + objName + " on the " + surfaceFrameName;

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

std::string ActionPut::explain() const
{
  return explanation;
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
  std::cout << "explanation: " << explanation << std::endl;

  REXEC(5)
  {
    auto xmlTasks = createTasksXML();

    for (const auto& t : xmlTasks)
    {
      std::cout << t << std::endl;
    }
  }

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
  // \todo(MG): Needs to be re-worked to handle agents consistentls
  if (!nearTo.empty())
  {
    double d_min = std::numeric_limits<double>::max();
    const RcsBody* putLocation = RcsGraph_getBodyByName(graph, surfaceFrameName.c_str());
    std::vector<const AffordanceEntity*> ntts = domain.getAffordanceEntities(nearTo);
    for (const auto& ntt : ntts)
    {
      const RcsBody* nttLocation = ntt->body(graph);
      d_min = std::min(d_min, Vec3d_distance(putLocation->A_BI.org, nttLocation->A_BI.org));
    }

    return d_min / (1.0 + d_min);
  }
  else if (!farFrom.empty())
  {
    double d_max = 0.0;
    const RcsBody* putLocation = RcsGraph_getBodyByName(graph, surfaceFrameName.c_str());
    std::vector<const AffordanceEntity*> ntts = domain.getAffordanceEntities(farFrom);
    for (const auto& ntt : ntts)
    {
      const RcsBody* nttLocation = ntt->body(graph);
      d_max = std::max(d_max, Vec3d_distance(putLocation->A_BI.org, nttLocation->A_BI.org));
    }

    return 1.0 / (1.0 + d_max);
  }

  return 0.0;
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

    if (params.size()<2)
    {
      throw ActionException(ActionException::ParamNotFound,
                            "The action magic_get requires at least two parameters.",
                            "Call it with the object to put and the location.",
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__));
    }

    objName = params[0];
    auto nttsToPut = domain.getAffordanceEntities(objName);

    if (nttsToPut.empty())
    {
      throw ActionException(ActionException::ParamNotFound,
                            "The " + objName + " to put down is unknown.",
                            "Use an object name that is defined in the environment",
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__));
    }

    objName = nttsToPut[0]->bdyName;

    surfaceFrameName = params[1];
    auto surfNtts = domain.getAffordanceEntities(surfaceFrameName);

    if (surfNtts.empty())
    {
      throw ActionException(ActionException::ParamNotFound,
                            "The surface " + surfaceFrameName + " to stack on is unknown.",
                            "Use an object name that is defined in the environment",
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__));
    }

    auto supportables = getAffordances<Supportable>(surfNtts[0]);

    if (supportables.empty())
    {
      throw ActionException(ActionException::ParamNotFound,
                            "Nothing can be put on the surface " + surfaceFrameName + ".",
                            "Use a surface with a Supportable affordance.",
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__));
    }

    surfaceFrameName = supportables[0]->frame;
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

  size_t getNumSolutions() const
  {
    return 1;
  }

  bool initialize(const ActionScene& domain, const RcsGraph* graph, size_t solutionRank)
  {
    if (solutionRank >= getNumSolutions())
    {
      return false;
    }

    return true;
  }

  std::unique_ptr<ActionBase> clone() const
  {
    return std::make_unique<ActionMagicPut>(*this);
  }

  std::string getActionCommand() const
  {
    return ActionBase::getActionCommand();
  }

};

REGISTER_ACTION(ActionMagicPut, "magic_put");

}   // namespace aff
