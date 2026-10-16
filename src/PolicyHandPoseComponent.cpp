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

#include "PolicyHandPoseComponent.h"
#include "IKTeleOp.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_resourcePath.h>


namespace aff
{



PolicyHandPoseComponent::PolicyHandPoseComponent(EntityBase* parent) :
  ComponentBase(parent)
{
  subscribe("UpdateScene", &PolicyHandPoseComponent::onUpdateScene);
}








/*******************************************************************************
 * Action querying interface
 ******************************************************************************/
static bool getActionHumanHandRetarget(const RcsGraph* desired,
                                       const ActionScene* scene,
                                       HTr* handRightPose,
                                       HTr* handLeftPose)
{
  if (!scene)
  {
    RLOG(0, "Scene not valid");
    return false;
  }

  auto agents = scene->getAgents<HumanAgent>();

  if (agents.size() != 1)
  {
    RLOG(0, "There's no or more than one agent");
    return false;
  }


  const HumanAgent* agent = agents[0];
  const RcsBody* shl = nullptr, * shr = nullptr, * ell = nullptr, * elr = nullptr, * hl = nullptr, * hr = nullptr;

  auto it = agent->trackedFrames.find(HumanAgent::BodyType::ShoulderLeft);
  if (it != agent->trackedFrames.end())
  {
    shl = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::ShoulderRight);
  if (it != agent->trackedFrames.end())
  {
    shr = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::ElbowLeft);
  if (it != agent->trackedFrames.end())
  {
    ell = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::ElbowRight);
  if (it != agent->trackedFrames.end())
  {
    elr = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::HandLeft);
  if (it != agent->trackedFrames.end())
  {
    hl = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  it = agent->trackedFrames.find(HumanAgent::BodyType::HandRight);
  if (it != agent->trackedFrames.end())
  {
    hr = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  if ((!shl) || (!shr) || (!ell) || (!elr) || (!hl) || (!hr))
  {
    RLOG(0, "Human agent body not fully defined");
    return false;
  }

  // Construct a frame that has:
  // origin in the center between the shoulders
  // y-axis pointing from right to left shoulder
  // x-axis pointing perpendicular to y-axis and up-vector (forward)
  // z-axis pointing up (right hand rule)
  // It rotates the world magnitudes into the shoulder-fixed frame
  HTr A_SI;
  Vec3d_sub(A_SI.rot[1], shl->A_BI.org, shr->A_BI.org);
  Vec3d_normalizeSelf(A_SI.rot[1]);
  Vec3d_crossProduct(A_SI.rot[0], A_SI.rot[1], Vec3d_ez());
  Vec3d_normalizeSelf(A_SI.rot[0]);
  Vec3d_crossProduct(A_SI.rot[2], A_SI.rot[0], A_SI.rot[1]);
  Vec3d_addAndConstMul(A_SI.org, shl->A_BI.org, shr->A_BI.org, 0.5);

  HTr_invTransform(handRightPose, &A_SI, &hr->A_BI);
  HTr_invTransform(handLeftPose, &A_SI, &hl->A_BI);

  return true;
}

static bool getActionAgentFingerAngles(const ActionScene* scene,
                                       std::vector<double>& fingerAnglesRight,
                                       std::vector<double>& fingerAnglesLeft,
                                       double& gripperAngleRight,
                                       double& gripperAngleLeft)
{
  if (!scene)
  {
    RLOG(0, "Scene not valid");
    return false;
  }

  auto agents = scene->getAgents<HumanAgent>();

  if (agents.size() != 1)
  {
    RLOG(0, "There's no or more than one agent");
    return false;
  }

  const HumanAgent* agent = agents[0];

  fingerAnglesRight = agent->fingersRight;
  fingerAnglesLeft = agent->fingersLeft;

  if (agent->gripperRight>=0.0)
  {
    gripperAngleRight = agent->gripperRight;
  }

  if (agent->gripperLeft>=0.0)
  {
    gripperAngleLeft = agent->gripperLeft;
  }

  return true;
}

void PolicyHandPoseComponent::onUpdateScene(RcsGraph* desired, RcsGraph* current, ActionScene* scene)
{
  nlohmann::json command;

  HTr handRightPose, handLeftPose;
  bool success = getActionHumanHandRetarget(desired, scene,
                                            &handRightPose,
                                            &handLeftPose);

  if (success)
  {
    command["input_type"] = IKTeleOp::ActionType::BiManualPose;

    double poseVec[12];
    HTr_toVector(poseVec, &handRightPose);
    // Vec3d_printFormatted("rh pos", "%.4f", poseVec);
    command["right_hand_pose"] = std::vector<double>(poseVec, poseVec + 9);
    HTr_toVector(poseVec, &handLeftPose);
    command["left_hand_pose"] = std::vector<double>(poseVec, poseVec + 9);
  }


  std::vector<double> fingerAnglesRight, fingerAnglesLeft;
  double gripperAngleRight=-1.0, gripperAngleLeft=-1.0;
  success = getActionAgentFingerAngles(scene, fingerAnglesRight, fingerAnglesLeft,
                                       gripperAngleRight, gripperAngleLeft);

  if (success)
  {
    command["finger_angles_right"] = fingerAnglesRight;
    command["finger_angles_left"] = fingerAnglesLeft;

    if ((gripperAngleRight>=0.0) && (gripperAngleLeft>=0.0))
    {
      command["left_fingers_pose_0"] = "open_fingers_left";
      command["left_fingers_pose_1"] = "close_fingers_left";
      command["s_left_01"] = gripperAngleLeft;

      command["right_fingers_pose_0"] = "open_fingers_right";
      command["right_fingers_pose_1"] = "close_fingers_right";
      command["s_right_01"] = gripperAngleRight;
    }

  }
  else
  {
    RLOG_CPP(0, "Failed: getActionAgentFingerAngles()");
  }

  getEntity()->publish("SetPolicyCommand", command);
}



}   // namespace aff
