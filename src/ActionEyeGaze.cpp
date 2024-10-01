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

#include "ActionEyeGaze.h"
#include "ActionFactory.h"
#include <ActivationSet.h>
#include <VectorConstraint.h>
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <JointWeightConstraint.h>

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#include <algorithm>

namespace aff
{
REGISTER_ACTION(ActionEyeGaze, "eye_gaze");

// These bodies constitute to the gaze model
static const std::string screenSurface  = "head_front_glass";
static const std::string rightPupil     = "RightPupil";
static const std::string leftPupil      = "LeftPupil";
static const std::string leftGazePoint  = "GazePointL";
static const std::string rightGazePoint = "GazePointR";
static const std::string gazePoint      = "GazePoint";



ActionEyeGaze::ActionEyeGaze(const ActionScene& scene,
                             const RcsGraph* graph,
                             std::vector<std::string> params) :
  isGazeTargetInHand(false), keepTasksActiveAfterEnd(true)
{
  parseParams(params);

  if (params.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The object to gaze at is not specified.",
                          "Use an object name that is defined in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  if (!params.size()==1)
  {
    throw ActionException(ActionException::ParamInvalid,
                          "Received " + std::to_string(params.size()) + " objects to gaze at the same time.",
                          "Specify only one object to gaze at.DEVELOPER : Number of passed strings is not 1",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  std::vector<const AffordanceEntity*> ntts = scene.getAffordanceEntities(params[0]);

  if (!ntts.empty())
  {
    if (ntts.size()!=1)
    {
      RLOG_CPP(1, "Found several object with the name " << params[0]);
    }

    gazeTargetInstance = ntts[0]->bdyName;
    gazeTarget = ntts[0]->name;
  }

  if (gazeTarget.empty())
  {
    const Agent* agent = scene.getAgent(params[0]);

    if (!agent)
    {
      throw ActionException(ActionException::ParamNotFound,
                            "The agent or object " + params[0] + " is unknown. ",
                            "Gaze at an object or agent name that is defined in the environment",
                            "This name was checked: " + params[0]);
    }

    // From here on, we have a valid agent. We look at its (first) head
    auto m = agent->getManipulatorsOfType(&scene, "head");
    if (!m.empty())
    {
      gazeTargetInstance = m[0]->bdyName;
      gazeTarget = m[0]->name;
    }

  }

  // Check if gaze target is a native RcsBody name
  if (gazeTarget.empty())
  {
    const RcsBody* gazeBdy = RcsGraph_getBodyByName(graph, params[0].c_str());
    if (gazeBdy)
    {
      gazeTargetInstance = params[0];
      gazeTarget = params[0];
    }
  }

  // Give up: gazeTarget is no entity, no agent, no RcsBody.
  if (gazeTarget.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't gaze at " + params[0] + ". It is neither an entity nor an agent nor a body",
                          "Check if the given target is correct, or use another one",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }





  // // Retrieve camera body \todo(MG): no explicit naming here
  // cameraFrame = "head_kinect_rgb_link";
  // const RcsBody* camBdy = RcsGraph_getBodyByNameNoCase(graph, cameraFrame.c_str());

  // \todo(CP): handle multiple agents
  std::vector<const Manipulator*> headsInScene = scene.getManipulatorsOfType("head");
  if (headsInScene.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't find a head(camera) in the scene to gaze with.",
                          "Check configuration file",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  for (const auto h : headsInScene)
  {
    cameraFrame = h->getGazingFrame();

    // BAD: using the first available `head` with `GazeCapability` in the scene atm. OK with only 1 robot in the scene.
    if (!cameraFrame.empty())
    {
      auto agent = Agent::getAgentOwningManipulator(&scene, h->name);
      if (agent)
      {
        this->agentName = agent->name;
      }
      else
      {
        this->agentName = "Unknown agent for manipulator " + h->name;
      }

      break;
    }
  }

  if (cameraFrame.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't find a head with gazing capability in the scene.",
                          "Check configuration file",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  const RcsBody* camBdy = RcsGraph_getBodyByNameNoCase(graph, cameraFrame.c_str());

  if (!camBdy)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "Can't find a head with gazing capability: Body '" + cameraFrame + "' not found.",
                          "Check configuration file",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  // Determine if object to be looked at has been grasped
  if (!ntts.empty())
  {
    isGazeTargetInHand = scene.getGraspingHand(graph, ntts[0]) ? true : false;
  }


  // Task naming
  this->taskGaze = "Gaze-" + cameraFrame + "-" + gazeTargetInstance;
}

ActionEyeGaze::~ActionEyeGaze()
{
}
/*

  <Task name="Eye C1" effector="LeftPupil" refBdy="head_front_glass" controlVariable="Z" active="true" />
  <Task name="Eye C2" effector="LeftPupil" refBdy="head_front_glass" controlVariable="POLAR" axisDirection="X" active="true" />
  <Task name="GazeL"  effector="GazePoint" refBdy="GazePointL" controlVariable="XYZ" active="true" />

  <Task name="Eye C1" effector="RightPupil" refBdy="head_front_glass" controlVariable="Z" active="true" />
  <Task name="Eye C2" effector="RightPupil" refBdy="head_front_glass" controlVariable="POLAR" axisDirection="X" active="true" />
  <Task name="GazeR"  effector="GazePoint"  refBdy="GazePointR" controlVariable="XYZ" active="true" />

*/
std::vector<std::string> ActionEyeGaze::createTasksXML() const
{
  std::vector<std::string> tasks;
  std::string xmlTask;

  xmlTask = "<Task name=\"EyeL C1\" effector=\"" + leftPupil + "\" refBdy=\"" + screenSurface + "\" controlVariable=\"Z\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"EyeL C2\" effector=\"" + leftPupil + "\" refBdy=\"" + screenSurface + "\" controlVariable=\"POLAR\" axisDirection=\"X\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"GazeL\"  effector=\"" + gazePoint + "\" refBdy=\"" + leftGazePoint + "\" controlVariable=\"XYZ\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"EyeR C1\" effector=\"" + rightPupil + "\" refBdy=\"" + screenSurface + "\" controlVariable=\"Z\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"EyeR C2\" effector=\"" + rightPupil + "\" refBdy=\"" + screenSurface + "\" controlVariable=\"POLAR\" axisDirection=\"X\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"GazeR\" effector=\"" + gazePoint + "\" refBdy=\"" + rightGazePoint + "\" controlVariable=\"XYZ\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"GazePoint\" effector=\"" + gazePoint + "\" refBdy=\"" + gazeTargetInstance + "\" refFrame=\"Johnnie\" controlVariable=\"XYZ\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionEyeGaze::createTrajectory(double t_start, double t_end) const
{
  const double afterTime = 0.5;
  const double t_gaze = 0.5 * (t_start+t_end);
  auto a1 = std::make_shared<tropic::ActivationSet>();

  a1->addActivation(t_start, true, 0.5, "EyeL C1");
  a1->addActivation(t_start, true, 0.5, "EyeL C2");
  a1->addActivation(t_start, true, 0.5, "GazeL");
  a1->addActivation(t_start, true, 0.5, "EyeR C1");
  a1->addActivation(t_start, true, 0.5, "EyeR C2");
  a1->addActivation(t_start, true, 0.5, "GazeR");
  a1->addActivation(t_start, true, 0.5, "GazePoint");

  if (!keepTasksActiveAfterEnd)
  {
    a1->addActivation(t_end + afterTime, false, 0.5, "EyeL C1");
    a1->addActivation(t_end + afterTime, false, 0.5, "EyeL C2");
    a1->addActivation(t_end + afterTime, false, 0.5, "GazeL");
    a1->addActivation(t_end + afterTime, false, 0.5, "EyeR C1");
    a1->addActivation(t_end + afterTime, false, 0.5, "EyeR C2");
    a1->addActivation(t_end + afterTime, false, 0.5, "GazeR");
    a1->addActivation(t_end + afterTime, false, 0.5, "GazePoint");
  }

  a1->add(std::make_shared<tropic::VectorConstraint>(t_gaze, std::vector<double> {0.0}, "EyeL C1"));
  a1->add(std::make_shared<tropic::PolarConstraint>(t_gaze, 0.0, 0.0, "EyeL C2"));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_gaze, 0.0, 0.0, 0.0, "GazeL"));
  a1->add(std::make_shared<tropic::VectorConstraint>(t_gaze, std::vector<double> {0.0}, "EyeR C1"));
  a1->add(std::make_shared<tropic::PolarConstraint>(t_gaze, 0.0, 0.0, "EyeR C2"));
  a1->add(std::make_shared<tropic::PositionConstraint>(t_gaze, 0.0, 0.0, 0.0, "GazeR"));

  a1->add(std::make_shared<tropic::PositionConstraint>(t_end, 0.0, 0.0, 0.0, "GazePoint"));

  return a1;
}

std::vector<std::string> ActionEyeGaze::getManipulators() const
{
  return usedManipulators;
}

std::string ActionEyeGaze::getGazeTarget() const
{
  return gazeTarget;
}

std::unique_ptr<ActionBase> ActionEyeGaze::clone() const
{
  return std::make_unique<ActionEyeGaze>(*this);
}

double ActionEyeGaze::getDefaultDuration() const
{
  return 5.0;
}

std::string ActionEyeGaze::getActionCommand() const
{
  std::string actionCommand = "eye_gaze " + gazeTargetInstance;

  if (getDuration() != getDefaultDuration())
  {
    actionCommand += " duration " + std::to_string(getDuration());
  }

  return actionCommand;
}

// 0: Neck only, 1: pupils only
bool ActionEyeGaze::setPupilSpeedWeight(RcsGraph* graph, double weight)
{
  if ((weight<0.0) || (weight>1.0))
  {
    RLOG(1, "Weight is %f but must be [0...1]", weight);
    return false;
  }

  RcsJoint* pan = RcsGraph_getJointByName(graph, "ptu_pan_joint");
  RcsJoint* tilt = RcsGraph_getJointByName(graph, "ptu_tilt_joint");

  if (!pan)
  {
    RLOG_CPP(1, "Joint with name \"ptu_pan_joint\" not found - skipping setting weight");
    return false;
  }

  if (!tilt)
  {
    RLOG_CPP(1, "Joint with name \"ptu_tilt_joint\" not found - skipping setting weight");
    return false;
  }

  pan->weightMetric = 1.0-weight;
  tilt->weightMetric = 1.0-weight;

  return true;
}

bool ActionEyeGaze::computePupilCoordinates(const RcsGraph* graph, double p_right[3], double p_left[3])
{
  const RcsBody* leftPupilBdy = RcsGraph_getBodyByName(graph, leftPupil.c_str());
  const RcsBody* rightPupilBdy = RcsGraph_getBodyByName(graph, rightPupil.c_str());

  // z points outwards, x points left, y points down. Origin is screen center
  const RcsBody* screenSurfaceBdy = RcsGraph_getBodyByName(graph, screenSurface.c_str());

  if (!leftPupilBdy)
  {
    RLOG_CPP(1, "Body with name \"" << leftPupil << "\" not found - skipping pupil cordinates calculation");
    return false;
  }

  if (!rightPupilBdy)
  {
    RLOG_CPP(1, "Body with name \"" << rightPupil << "\" not found - skipping pupil cordinates calculation");
    return false;
  }

  if (!screenSurfaceBdy)
  {
    RLOG_CPP(1, "Body with name \"" << screenSurface << "\" not found - skipping pupil cordinates calculation");
    return false;
  }

  Vec3d_invTransform(p_left, &screenSurfaceBdy->A_BI, leftPupilBdy->A_BI.org);
  Vec3d_invTransform(p_right, &screenSurfaceBdy->A_BI, rightPupilBdy->A_BI.org);

  return true;
}


}   // namespace aff
