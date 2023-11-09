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

#include "ActionFingerPush.h"
#include "ActionFactory.h"
#include "ActivationSet.h"
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <VectorConstraint.h>

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#include <limits>

#define fingersOpen   (0.01)
#define fingersClosed (1.32)

/*! Push a button with something that can poke:
 *  finger_push <object_to_poke> (<object_that_pokes>)
 *
 *  The second parameter is optional. If it is not given, then the closest
 *  FingerpushCapability will be used. If an object_that_pokes is given, it
 *  must be held in a hand. Kinematically, the z-axes of the two frames must
 *  oppose.
 */

namespace aff
{
REGISTER_ACTION(ActionFingerPush, "finger_push");
REGISTER_ACTION(ActionFingerPush, "poke");
REGISTER_ACTION(ActionFingerPush, "id_63558c7984f61faabb83cb1c"); // switch on
REGISTER_ACTION(ActionFingerPush, "id_63563f7302a8c9f3ffa00810"); // switch off


ActionFingerPush::ActionFingerPush(const ActionScene& domain,
                                   const RcsGraph* graph,
                                   std::vector<std::string> params)
{
  if (params.empty())
  {
    throw ActionException("ActionFingerPush: needs at least 1 parameter, but received 0",
                          ActionException::ParamInvalid);
  }







  {
    // Initialize object to push. It must have a PointPushable affordance
    const AffordanceEntity* object = domain.getAffordanceEntity(params[0]);

    if (!object)
    {
      throw ActionException("ERROR REASON: The " + params[0] +
                            " is unknown. SUGGESTION: Use an object name that is defined in the environment",
                            ActionException::ParamNotFound);
    }

    auto pushables = getAffordances<PointPushable>(object);
    if (pushables.empty())
    {
      throw ActionException("ERROR REASON: The " + params[0] +
                            " is not switchable. SUGGESTION: Replace this action with a more clever one.",
                            ActionException::ParamNotFound);
    }

    // \todo(MG)
    if (pushables.size() != 1)
    {
      throw ActionException("ERROR REASON: found " + std::to_string(pushables.size()) +
                            " frames, but only 1 is supported", ActionException::ParamInvalid);
    }

    RLOG_CPP(0, "Found object to push: " << object->name);
    frameToBePoked = pushables[0]->frame;
  }

  // We require this to assign the finger trajectories, for the case we don't
  // poke with an object
  const Manipulator* theHand = NULL;

  // We have a second argument which is the object to push with.
  if (params.size()>1)
  {
    // \todo (MG): Check that pusher is held in hand

    // Initialize object to push. It must have a PointPokable affordance.
    const AffordanceEntity* pusher = domain.getAffordanceEntity(params[1]);

    if (!pusher)
    {
      throw ActionException("ERROR REASON: The tool to push with " + params[1] +
                            " is unknown. SUGGESTION: Use an object name that is defined in the environment",
                            ActionException::ParamNotFound);
    }
    auto pokables = getAffordances<PointPokable>(pusher);

    if (pokables.empty())
    {
      throw ActionException("ERROR REASON: The tool " + params[1] +
                            " cannot be used to switch something on. SUGGESTION: Use another tool",
                            ActionException::ParamNotFound);
    }

    // \todo(MG)
    if (pokables.size() != 1)
    {
      throw ActionException("ActionFingerPush: found " + std::to_string(pokables.size()) +
                            " pokables, but only 1 is supported", ActionException::ParamInvalid);
    }

    frameThatPokes = pokables[0]->frame;
  }
  // We only received the object to poke, and need to figure out the finger
  // to poke it.
  else
  {
    const RcsBody* pushBdy = RcsGraph_getBodyByName(graph, frameToBePoked.c_str());
    RCHECK(pushBdy);
    const RcsBody* closestFinger = NULL;

    RLOG(0, "Collecting fingers that can push");
    double dMin = std::numeric_limits<double>::max();
    for (auto& m : domain.getFreeManipulators(graph))   // Disallow using occupied manipulator.
    {
      for (auto g : m->capabilities)
      {
        if (g->classType == Capability::Type::FingerpushCapability)
        {
          const RcsBody* fbdy = RcsGraph_getBodyByNameNoCase(graph, g->frame.c_str());
          RCHECK_MSG(fbdy, "%s", g->frame.c_str());
          RLOG(0, "Comparing %s", fbdy->name);

          // Compute the distance between finger and object and keep track of the closest finger.
          double d = Vec3d_distance(fbdy->A_BI.org, pushBdy->A_BI.org);
          if (d < dMin)
          {
            dMin = d;
            closestFinger = fbdy;
            theHand = m;
          }
        }
      }
    }

    if (!closestFinger)
    {
      throw ActionException("ERROR REASON: Robot has no fingers to perform this action. SUGGESTION: Use a tool",
                            ActionException::ParamNotFound);
    }

    if (frameThatPokes.empty())
    {
      frameThatPokes = closestFinger->name;
    }
  }


  RLOG_CPP(0, "****** finger is " << frameThatPokes);




  // Task naming
  this->taskPushPos = "PushButton-XYZ-" + frameThatPokes + "-" + frameToBePoked;
  this->taskPushOri = "PushButton-Polar-" + frameThatPokes + "-" + frameToBePoked;
  this->taskFingers = frameThatPokes + "_fingers";

  // Assemble finger task name
  if (theHand)
  {
    for (auto& f : theHand->fingerJoints)
    {
      fingerJoints += f;
      fingerJoints += " ";
    }
  }

  explanation = "I'm pushing the " + params[0];
  if (params.size() > 1)
  {
    explanation += " with the ";
    explanation += params[1];
  }
  RLOG_CPP(0, explanation);
  RLOG(0, "Done constructing ActionFingerPush");
}

ActionFingerPush::~ActionFingerPush()
{
}

std::vector<std::string> ActionFingerPush::createTasksXML() const
{
  std::vector<std::string> tasks;

  std::string xmlTask;
  xmlTask = "<Task name=\"" + taskPushPos + "\" " + "controlVariable=\"XYZ\" " +
            "effector=\"" + frameThatPokes + "\" " + "refBdy=\"" + frameToBePoked + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskPushOri + "\" " + "controlVariable=\"POLAR\" " +
            "effector=\"" + frameThatPokes + "\" " + "refBdy=\"" + frameToBePoked + "\" />";
  tasks.push_back(xmlTask);

  if (!fingerJoints.empty())
  {
    xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
              "jnts=\"" + fingerJoints + "\" />";
    tasks.push_back(xmlTask);
  }

  return tasks;
}


/*
*  This action pushes on a button. Here is the detailed
 * description of the timings:
 *
 * - t_prep:  At this time point, the finger traverses a through-point above
 *            the switch.
 * - t_touch: The finger touches the switch.
 * - t_down:  The switch is pushed down
 * - t_end:   The finger is retracted from the switch
 * - t_afterTime: All tasks are deactivated.
 *
 * t_start   t_prep        t_touch     t_down       t_end   t_afterTime
 *    |         |            |            |           |           |
 *  -----------------------------------------------------------------> time
 */
tropic::TCS_sptr ActionFingerPush::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  const double duration = t_end - t_start;
  const double t_prep = t_start + 0.5 * duration;
  const double t_touch = t_start + 0.7 * duration;
  const double t_down = t_start + 0.8 * duration;
  const double push_depth = -0.03;   // How far to push the button in
  const double push_retract = 0.1;   // How far to retract the hand after pushing
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, taskPushPos);
  a1->addActivation(t_start, true, 0.5, taskPushOri);
  a1->addActivation(t_end + afterTime, false, 0.5, taskPushPos);
  //a1->addActivation(t_down, false, 0.5, taskPushOri);
  a1->addActivation(t_end, false, 0.5, taskPushOri);

  a1->add(t_prep, 0.0, 0.0, 0.0, 7, taskPushPos + " 0");// and centered
  a1->add(t_prep, 0.05, 0.0, 0.0, 7, taskPushPos + " 2");// and centered

  a1->add(std::make_shared<tropic::PositionConstraint>(t_touch, 0.0, 0.0, 0.0, taskPushPos));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_down, 0.0, 0.0, push_depth, taskPushPos));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_end, 0.0, 0.0, push_retract, taskPushPos));
  a1->add(std::make_shared<tropic::PolarConstraint>(t_touch, M_PI, 0.0, taskPushOri));
  a1->add(std::make_shared<tropic::PolarConstraint>(t_end, M_PI, 0.0, taskPushOri));

  // Fingers
  if (!fingerJoints.empty())
  {
    a1->addActivation(t_start, true, 0.5, taskFingers);
    a1->addActivation(t_end + afterTime, false, 0.5, taskFingers);
    a1->add(std::make_shared<tropic::VectorConstraint>(t_prep, std::vector<double>
    {fingersClosed, fingersClosed, fingersClosed}, taskFingers));
    a1->add(std::make_shared<tropic::VectorConstraint>(t_end, std::vector<double>
    {fingersClosed, fingersClosed, fingersClosed}, taskFingers));
  }

  return a1;
}

double ActionFingerPush::getDurationHint() const
{
  return 2.0*8.0;
}

std::string ActionFingerPush::explain() const
{
  return explanation;
}

std::vector<std::string> ActionFingerPush::getManipulators() const
{
  return usedManipulators;
}

std::unique_ptr<ActionBase> ActionFingerPush::clone() const
{
  return std::make_unique<ActionFingerPush>(*this);
}

}   // namespace aff
