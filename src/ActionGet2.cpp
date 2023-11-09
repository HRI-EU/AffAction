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

#include "ActionGet2.h"
//#include "ActionFactory.h"
#include "TrajectoryPredictor.h"

#include <ControllerBase.h>

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


#define fingersOpen   (0.01)
#define fingersClosed (0.7)
#define t_fingerMove  (1.5)


#if 1

const RcsBody* resolveBodyName(const RcsGraph* graph, std::string& bdyName)
{
  // Rapid exit in this case
  if (bdyName.empty())
  {
    return NULL;
  }

  const RcsBody* bdy = RcsGraph_getBodyByNameNoCase(graph, bdyName.c_str());

  if (!bdy)
  {
    RLOG(1, "Failed to find graph body for \"%s\"", bdyName.c_str());
    return NULL;
  }

  bdyName = std::string(bdy->name);

  return bdy;
}


namespace sepp
{
// REGISTER_ACTION(ActionGet2, "get");
// REGISTER_ACTION(ActionGet2, "id_6355842184f61faabb83cb1b");


ActionGet2::ActionGet2(const ActionScene& domain,
                       const RcsGraph* graph,
                       std::vector<std::string> params) :
  ActionGet2(domain, graph,
             params[0],
             params.size()>1?params[1]:std::string())
{
}

ActionGet2::ActionGet2(const ActionScene& domain,
                       const RcsGraph* graph,
                       const std::string& objectToGet,
                       const std::string& manipulator,
                       double liftHeight_,
                       double preGraspDist_) :
  liftHeight(liftHeight_), preGraspDist(preGraspDist_)
{
  RLOG_CPP(1, "Calling get with manipulator='" << manipulator
           << "' object='" << objectToGet << "'");
  std::string errorMsg;

  // Initialize object to get. It must have a grasp feature
  const AffordanceEntity* object = domain.getAffordanceEntity(objectToGet);

  if (!object)
  {
    throw ActionException("ActionGet2: object '" + objectToGet +
                          "' not found", ActionException::ParamNotFound);
  }

  auto graspables = getAffordances<Graspable>(object);

  if (graspables.empty())
  {
    throw ActionException("ActionGet2: object '" + objectToGet +
                          "' has no grasp affordances", ActionException::ParamNotFound);
  }

  // ResolveBodyName changes the objectName in certain cases (e.g. generic bodies)
  objectName = object->bdyName;
  const RcsBody* objBdy = resolveBodyName(graph, objectName);

  if (!objBdy)
  {
    errorMsg = "Couldn't resolve object body '" + objectName +
               "'. Is it part of the graph?";
    throw ActionException(errorMsg, ActionException::ParamNotFound);
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
      throw ActionException("ActionGet2: Manipulator '" + manipulator + "' not found",
                            ActionException::ParamNotFound);
    }

    if (!hand->isEmpty(graph))
    {
      throw ActionException("ActionGet2: Manipulator '" + manipulator +
                            "' is already holding something", ActionException::ParamNotFound);
    }

    freeManipulators.push_back(hand);
  }

  if (freeManipulators.empty())
  {
    throw ActionException("ActionGet2: Could not find free manipulator",
                          ActionException::KinematicallyImpossible);
  }

  // From here on we have one or several manipulators that can be tentatively used
  // to get the object. We now go through each of them and compile a list of pairs
  // matching each manipulator's capability to the object affordances.
  for (const Manipulator* hand : freeManipulators)
  {
    auto graspMap_i = match<Graspable>(object, hand);
    affordanceMap.insert(affordanceMap.end(), graspMap_i.begin(), graspMap_i.end());
  }

  // If this vector is empty, there's no capability that can get the object in any way.
  // How sad that we have to stop here.
  if (affordanceMap.empty())
  {
    throw ActionException("ActionGet2: Could not find valid grasp",
                          ActionException::KinematicallyImpossible);
  }

  // We have one or several matches and sort them according to their cost. Lower
  // costs pairs are at the beginning.
  sort(graph, affordanceMap);

  // Initialize with the best solution.
  initialize(domain, 0);
}

bool ActionGet2::initialize(const ActionScene& domain,
                            size_t solutionRank)
  {
  if (solutionRank >= getNumSolutions())
  {
    return false;
  }

  // Get the frame of the capability from the first entry
  const Capability* winningCap = std::get<1>(affordanceMap[solutionRank]);
  capabilityFrame = winningCap->frame;

  // Get the frame of the affordance from the second capability
  Affordance* winningAff = std::get<0>(affordanceMap[solutionRank]);
  affordanceFrame = winningAff->frame;
  if (dynamic_cast<PowerGraspable*>(winningAff))
  {
    graspOrientation = "Polar";
  }
  else
  {
    graspOrientation = "Euler";
  }

  RLOG_CPP(1, "Capability frame: " << capabilityFrame << " affordance frame: "
           << affordanceFrame);

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

  // Task naming
  this->taskObjHandPos = objectName + "-" + capabilityFrame + "-XYZ";
  this->taskObjSurfacePosX = objectName + "-X";
  this->taskObjSurfacePosY = objectName + "-Y";
  this->taskObjSurfacePosZ = objectName + "-Z";
  this->taskObjOri = objectName + "-POLAR";
  this->taskHandObjOri = capabilityFrame + "-" + objectName + "-POLAR";
  this->taskFingers = capabilityFrame + "_fingers";

  return true;
}

ActionGet2::~ActionGet2()
{
}

tropic::TCS_sptr ActionGet2::createTrajectory(double t_start, double t_end) const
{
  const double t_grasp = t_start + 0.66*(t_end - t_start);
  auto a1 = createTrajectory(t_start, t_grasp, t_end);

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
ActionGet2::createTrajectory(double t_start,
                             double t_grasp,
                             double t_end) const
{
  const double afterTime = 0.5;//2.0;

  // Grasp the object and lift it up
  auto a1 = std::make_shared<tropic::ActivationSet>();

  // Hand position with respect to object
  const double t_pregrasp = t_start + 0.5*(t_grasp-t_start);
  a1->addActivation(t_start, true, 0.5, taskObjHandPos);
  a1->addActivation(t_grasp, false, 0.5, taskObjHandPos);
  a1->add(t_pregrasp, preGraspDist, 0.0, 0.0, 1, taskObjHandPos + " 0");// some distance in front
  a1->add(t_pregrasp, 0.0, 0.0, 0.0, 7, taskObjHandPos + " 1");// and centered
  a1->add(std::make_shared<tropic::PositionConstraint>(t_grasp, 0.0, 0.0, 0.0, taskObjHandPos));

  // if (getOptimDim()==3)
  // {
  //   const double t_optim = t_start + 0.25 * (t_grasp - t_start);
  //   auto oState = getOptimState();
  //   RLOG(1, "Adding optimization constraint [%.4f %.4f %.4f] at t=%f",
  //        oState[0], oState[1], oState[2], t_optim);
  //   a1->add(std::make_shared<tropic::PositionConstraint>(t_optim, oState.data(), taskObjHandPos, 1));
  // }

  // Hand orientation with respect to object
  a1->addActivation(t_start, true, 0.5, taskHandObjOri);
  a1->addActivation(t_grasp, false, 0.5, taskHandObjOri);

  if (graspOrientation=="Polar")
  {
    a1->add(std::make_shared<tropic::PolarConstraint>(t_grasp, 0.0, 0.0, taskHandObjOri));
  }
  else if (graspOrientation=="Euler")
  {
    a1->add(std::make_shared<tropic::EulerConstraint>(t_grasp, 0.0, 0.0, 0.0, taskHandObjOri));
  }
  else
  {
    RFATAL("Unknown grasp orientation: \"%s\"", graspOrientation.c_str());
  }

  // Object orientation with respect to world frame.
  a1->addActivation(t_grasp, true, 0.5, taskObjOri);
  a1->addActivation(t_end+ afterTime, false, 0.5, taskObjOri);

  // We don't add a constraint here in order to preserve the previous orientation
  //if (graspOrientation=="Polar")
  //{
  //  a1->add(std::make_shared<tropic::PolarConstraint>(t_end, 0.05, M_PI_2, taskObjOri));
  //}
  //else
  //{
  //  a1->add(std::make_shared<tropic::EulerConstraint>(t_end, 0.0, 0.0, -M_PI*0, taskObjOri));
  //}

  // Lift object
  a1->addActivation(t_grasp, true, 0.5, taskObjSurfacePosZ);
  a1->addActivation(t_end+ afterTime, false, 0.5, taskObjSurfacePosZ);
  a1->add(t_end, liftHeight, 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");
  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_grasp, objectName, capabilityFrame));

  // Lift it up in the y-z plane (keep forward position constant)
  a1->addActivation(t_grasp, true, 0.5, taskObjSurfacePosX);
  a1->addActivation(t_end+afterTime, false, 0.5, taskObjSurfacePosX);

  // Reduce snap to null space y-position by moving up vertically
  a1->addActivation(t_grasp, true, 0.5, taskObjSurfacePosY);
  a1->addActivation(t_end+ afterTime, false, 0.5, taskObjSurfacePosY);

  // Close fingers
  a1->addActivation(t_start, true, 0.5, taskFingers);
  a1->add(std::make_shared<tropic::VectorConstraint>(t_grasp-0.5*t_fingerMove, std::vector<double> {fingersOpen, fingersOpen, fingersOpen}, taskFingers));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_grasp+0.5*t_fingerMove, std::vector<double> {fingersClosed, fingersClosed, fingersClosed}, taskFingers));

  // Deactivate object collisions when in pregrasp
  a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_pregrasp, objectName, false));

  return a1;
}

void ActionGet2::print() const
{
  std::cout << "objectName: " << objectName << std::endl;
  std::cout << "capabilityFrame: " << capabilityFrame << std::endl;
  std::cout << "affordanceFrame: " << affordanceFrame << std::endl;
  std::cout << "fingerJoints: " << fingerJoints << std::endl;
  std::cout << "graspOrientation: " << graspOrientation << std::endl;

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
    RLOG(1, "Solution %zu: %s - %s", i,
         std::get<0>(affordanceMap[i])->frame.c_str(),
         std::get<1>(affordanceMap[i])->frame.c_str());
    //CapabilityCompare::eval(graph, gmi, 1.0, 1.0));
  }

}

std::vector<std::string> ActionGet2::createTasksXML() const
{
  std::vector<std::string> tasks;
  std::string xmlTask;

  // taskObjHandPos: XYZ-task with effector=object and refBdy=hand
  xmlTask = "<Task name=\"" + taskObjHandPos + "\" " +
            "controlVariable=\"XYZ\" effector=\"" + affordanceFrame +
            "\" " + "refBdy=\"" + capabilityFrame + "\" />";
  tasks.push_back(xmlTask);

  // taskObjSurfacePosition z and sideways velocities
  xmlTask = "<Task name=\"" + taskObjSurfacePosX + "\" " +
            "controlVariable=\"X\" effector=\"" + objectName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjSurfacePosY + "\" " +
            "controlVariable=\"Y\" effector=\"" + objectName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjSurfacePosZ + "\" " +
            "controlVariable=\"Z\" effector=\"" + objectName + "\" />";
  tasks.push_back(xmlTask);

  // taskObjPolar
  if (graspOrientation == "Polar")
  {
    xmlTask = "<Task name=\"" + taskObjOri + "\" " +
              "controlVariable=\"POLAR\" " + "effector=\"" +
              affordanceFrame + "\" />";
  }
  else if (graspOrientation == "Euler")
  {
    xmlTask = "<Task name=\"" + taskObjOri + "\" " +
              "controlVariable=\"ABC\" " + "effector=\"" +
              affordanceFrame + "\" />";
  }
  else
  {
    RLOG(1, "Unknown grasp orientation: %s", graspOrientation.c_str());
    return std::vector<std::string>();
  }

  tasks.push_back(xmlTask);

  // taskHandObjOri
  if (graspOrientation == "Polar")
  {
    xmlTask = "<Task name=\"" + taskHandObjOri + "\" " +
              "controlVariable=\"POLAR\" " + "effector=\"" + capabilityFrame + "\" " +
              "refBdy=\"" + affordanceFrame + "\" />";
  }
  else
  {
    xmlTask = "<Task name=\"" + taskHandObjOri + "\" " +
              "controlVariable=\"ABC\" " + "effector=\"" + capabilityFrame + "\" " +
              "refBdy=\"" + affordanceFrame + "\" />";
  }
  tasks.push_back(xmlTask);

  // Fingers
  xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerJoints + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

std::string ActionGet2::explain() const
{
  //return std::string("I'm getting the " + objectName + " with my " + capabilityFrame);
  return std::string("I'm getting the " + objectName);
}

std::vector<std::string> ActionGet2::getManipulators() const
{
  return usedManipulators;
}

std::vector<double> ActionGet2::getInitOptimState(tropic::TrajectoryControllerBase* tc,
                                                  double duration) const
{
  double pos[3];
  tropic::TrajectoryND* trj = tc->getTrajectory(taskObjHandPos);
  trj->getPosition(0.66*duration, pos);
  std::vector<double> o(pos, pos+3);
  return o;
}

size_t ActionGet2::getNumSolutions() const
{
  return affordanceMap.size();
}

std::unique_ptr<ActionBase> ActionGet2::clone() const {
  return std::make_unique<ActionGet2>(*this);
}


}   // namespace aff



#endif//0
