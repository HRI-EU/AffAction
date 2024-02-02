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

#include "ActionPass.h"
#include "ActionFactory.h"
#include "CollisionModelConstraint.h"

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <VectorConstraint.h>
#include <ConnectBodyConstraint.h>

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#include <algorithm>


#define fingersOpen   (0.01)



namespace aff
{
REGISTER_ACTION(ActionPass, "pass");

ActionPass::ActionPass(const ActionScene& domain,
                       const RcsGraph* graph,
                       std::vector<std::string> params)
{
  auto it = std::find(params.begin(), params.end(), "duration");
  if (it != params.end())
  {
    defaultDuration = std::stod(*(it+1));
    params.erase(it+1);
    params.erase(it);
  }

  init(domain, graph, params[0], params.size() > 1 ? params[1] : std::string());
}

ActionPass::~ActionPass()
{
}

void ActionPass::init(const ActionScene& domain,
                      const RcsGraph* graph,
                      const std::string& objectToPass,
                      const std::string& receivingAgent)
{
  RLOG_CPP(1, "Calling pass with object='" << objectToPass << "' to pass to agent='"
           << receivingAgent << "'");

  // Initialize object to pass. We support several objects with the same name.
  std::vector<const AffordanceEntity*> objects = domain.getAffordanceEntities(objectToPass);

  if (objects.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "REASON: The " + objectToPass + " is unknown.",
                          "SUGGESTION: Use an object name that is defined in the environment");
  }

  auto gPair = domain.getGraspingHand(graph, objects);
  const Manipulator* graspingHand = std::get<0>(gPair);
  const AffordanceEntity* object = std::get<1>(gPair);

  if ((!object) || (!graspingHand))
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The " + objectToPass + " is not held in the hand.",
                          "First get the " + objectToPass + " in the hand before passing it to somebody");
  }

  // From here on, we have a valid object and grasping hand
  objectRcsName = object->bdyName;

  this->graspingHandRcsName = graspingHand->bdyName;
  usedManipulators.push_back(graspingHand->name);
  handOpen = std::vector<double>(graspingHand->getNumFingers(), fingersOpen);


  // Get robot shoulder coordinates. We need this to compute the pass over position
  const RcsJoint* shldr = RcsGraph_getJointByName(graph, graspingHand->baseJointName.c_str());

  if (!shldr)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The " + graspingHand->name + " is not connected to a shoulder.",
                          "Are you calling this action with a robot agent?",
                          "baseJntName: " + graspingHand->baseJointName);
  }

  if (graspingHand->reach<=0.0)
  {
    throw ActionException(ActionException::ParamInvalid,
                          "The passing agent has no reach.",
                          "Fix the xml file");
  }

  // From here on, we have a valid RcsJoint shldr
  const double* roboShlrdPos = shldr->A_JI.org;

  // Find the agent to pass the object to
  const Agent* agent = domain.getAgent(receivingAgent);

  if (!agent)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The agent " + receivingAgent + " is not in the scene.",
                          "Pass the " + objectToPass + " to somebody else");
  }

  if (!dynamic_cast<const HumanAgent*>(agent))
  {
    throw ActionException(ActionException::ParamInvalid,
                          "The agent " + receivingAgent + " is not a human.",
                          "Pass the " + objectToPass + " to a human, and not a robot");
  }

  // From here on, we have a valid agent. Find the agent's head
  const double* agentHeadPos = NULL;
  for (const auto& mName : agent->manipulators)
  {
    const Manipulator* m = domain.getManipulator(mName);
    RCHECK_MSG(m, "Manipulator %s of agent %s not found",
               mName.c_str(), agent->name.c_str());

    if (m->isOfType("head"))
    {
      const RcsBody* head = RcsGraph_getBodyByName(graph, m->bdyName.c_str());
      if (!head)
      {
        throw ActionException(ActionException::ParamNotFound,
                              "The " + agent->name + "'s head manipulator has no RcsBody.",
                              "Use an object name that is defined in the environment");
      }
      else
      {
        this->agentHeadRcsName = head->name;
        agentHeadPos = head->A_BI.org;
      }
    }
  }

  if (agentHeadRcsName.empty() ||(!agentHeadPos))
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The " + agent->name + " has no head. ");
  }

  // Compute pass over position. The vector dir points from the shoulder to
  // the head, its length is a fraction of the reach.
  double dir[3], passDir[3];
  Vec3d_sub(dir, agentHeadPos, roboShlrdPos);
  Vec3d_normalizeSelf(dir);
  Vec3d_constMul(passDir, dir, 0.65*graspingHand->reach);

  // The passFarPt is the point of passing if the agent is far away.
  double passFarPt[3];
  Vec3d_add(passFarPt, roboShlrdPos, passDir);

  // The passPt is the point that considers the distance to the agent. If the
  // agent with some peripersonal space around the head is closer than the
  // passFarPt, we reduce the distance to not pass the object unnaturally
  // close to the agent's body.
  const double faceDist = Vec3d_distance(passFarPt, agentHeadPos);
  const double peripersonalHeadRadius = 0.5;
  const double headDownOffset = 0.2;

  if (faceDist < peripersonalHeadRadius)
  {
    double corrDist = peripersonalHeadRadius - faceDist;
    for (int i = 0; i < 3; ++i)
    {
      this->passPt[i] = passFarPt[i] - corrDist * dir[i];
    }
  }
  else
  {
    Vec3d_copy(this->passPt, passFarPt);
  }

  this->passPt[2] -= headDownOffset;

  // Move a bit back so that after changing object parent to root we don't get a finger collision
  for (int i = 0; i < 3; ++i)
  {
    this->retractPt[i] = passPt[i] - 0.15*dir[i];
  }

  // Assemble finger task name
  for (auto& f : graspingHand->fingerJoints)
  {
    fingerJoints += f;
    fingerJoints += " ";
  }

  // Task naming
  this->taskHandPos = graspingHand->name + "-XYZ";
  this->taskHandOri = graspingHand->name + "-Polar";
  this->taskFingers = graspingHand->name + "_fingers";

  explanation = "I'm passing the " + object->bdyName + " to " + agent->name;

  // Initialize with the best solution.
  bool successInit = initialize(domain, graph, 0);
  RCHECK(successInit);
}

bool ActionPass::initialize(const ActionScene& domain,
                            const RcsGraph* graph,
                            size_t solutionRank)
{
  return true;
}

std::vector<std::string> ActionPass::createTasksXML() const
{
  std::vector<std::string> tasks;

  // taskObjHandPos: XYZ-task with effector=object and refBdy=hand
  std::string xmlTask = "<Task name=\"" + taskHandPos + "\" " +
                        "controlVariable=\"XYZ\" effector=\"" +
                        graspingHandRcsName + "\" />";
  tasks.push_back(xmlTask);

  // taskHandPolar
  // \todo(MG): axisDirection should be more general, for isntance finding the GraspCapability and using it's z-axis
  xmlTask = "<Task name=\"" + taskHandOri + "\" " +
            "controlVariable=\"POLAR\" axisDirection=\"X\" effector=\"" +
            graspingHandRcsName + "\" />";
  tasks.push_back(xmlTask);

  // Fingers
  xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerJoints + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

std::shared_ptr<tropic::ConstraintSet>
ActionPass::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  const double t_release = t_start+0.75*(t_end-t_start);

  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskHandPos);
  a1->addActivation(t_start, true, 0.5, taskHandOri);
  a1->addActivation(t_start+0.5*(t_end-t_start), true, 0.5, taskFingers);

  a1->addActivation(t_end + afterTime, false, 0.5, taskHandPos);
  a1->addActivation(t_end + afterTime, false, 0.5, taskHandOri);
  a1->addActivation(t_end + afterTime, false, 0.5, taskFingers);

  a1->add(std::make_shared<tropic::PositionConstraint>(t_release, passPt, taskHandPos));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_end, retractPt, taskHandPos));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_end, handOpen, taskFingers));

  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_release, objectRcsName, ""));

  a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_start, objectRcsName, false));
  a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_end, objectRcsName, true));

  return a1;
}

std::string ActionPass::explain() const
{
  return explanation;
}

std::vector<std::string> ActionPass::getManipulators() const
{
  return usedManipulators;
}

size_t ActionPass::getNumSolutions() const
{
  return 1;
}

std::unique_ptr<ActionBase> ActionPass::clone() const
{
  return std::make_unique<ActionPass>(*this);
}

}   // namespace aff
