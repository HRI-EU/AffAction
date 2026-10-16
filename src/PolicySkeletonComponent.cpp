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

#include "PolicySkeletonComponent.h"
#include "IKTeleOp.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_resourcePath.h>


namespace aff
{



PolicySkeletonComponent::PolicySkeletonComponent(EntityBase* parent,
                                                 const Rcs::ControllerBase* controller) :
  ComponentBase(parent), ctrlPtr(controller)
{
  subscribe("UpdateScene", &PolicySkeletonComponent::onUpdateScene);
}








/*******************************************************************************
 * Action querying interface
 ******************************************************************************/
static bool getActionHumanPostureRetarget(const RcsGraph* desired,
                                          const ActionScene* scene,
                                          std::vector<double>& upperArmRightDir,
                                          std::vector<double>& foreArmRightDir,
                                          std::vector<double>& handRightOri,
                                          std::vector<double>& upperArmLeftDir,
                                          std::vector<double>& foreArmLeftDir,
                                          std::vector<double>& handLeftOri,
                                          std::vector<double>& headOri)
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
  const RcsBody* shl = nullptr, *shr = nullptr, *ell = nullptr, *elr = nullptr, *hl = nullptr, *hr = nullptr, *head = nullptr;

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

  it = agent->trackedFrames.find(HumanAgent::BodyType::Head);
  if (it != agent->trackedFrames.end())
  {
    head = RcsGraph_getBodyByName(desired, it->second.c_str());
  }

  if ((!shl) || (!shr) || (!ell) || (!elr) || (!hl) || (!hr) || (!head))
  {
    RLOG(0, "Human agent body not fully defined");
    return false;
  }

  // Construct a frame that has:
  // y-axis pointing from right to left shoulder
  // x-axis pointing perpendicular to y-axis and up-vector (forward)
  // z-axis pointing up (right hand rule)
  // It rotates the world magnitudes into the shoulder-fixed frame
  double A_SI[3][3];
  Vec3d_sub(A_SI[1], shl->A_BI.org, shr->A_BI.org);
  Vec3d_normalizeSelf(A_SI[1]);
  Vec3d_crossProduct(A_SI[0], A_SI[1], Vec3d_ez());
  Vec3d_normalizeSelf(A_SI[0]);
  Vec3d_crossProduct(A_SI[2], A_SI[0], A_SI[1]);

  REXEC(1)
  {
    Mat3d_printCommentDigits("A_SI", A_SI, 5);
  }

  // Right upper arm
  double dirVec[3];
  Vec3d_sub(dirVec, elr->A_BI.org, shr->A_BI.org);
  Vec3d_normalizeSelf(dirVec);
  Vec3d_rotateSelf(dirVec, A_SI);
  upperArmRightDir = std::vector<double>(dirVec, dirVec +3);

  // Right forearm
  Vec3d_sub(dirVec, hr->A_BI.org, elr->A_BI.org);
  Vec3d_normalizeSelf(dirVec);
  Vec3d_rotateSelf(dirVec, A_SI);
  foreArmRightDir = std::vector<double>(dirVec, dirVec + 3);

  // Right hand rotation matrix
  handRightOri.resize(9);
  double tmp[3][3];
  Mat3d_copy(tmp, MAT3D_CAST hr->A_BI.rot);
  //Mat3d_postMulSelf(tmp, A_SI);
  Vec3d_rotateSelf(tmp[0], A_SI);
  Vec3d_rotateSelf(tmp[1], A_SI);
  Vec3d_rotateSelf(tmp[2], A_SI);
  Mat3d_toArray(handRightOri.data(), tmp);// MAT3D_CAST hr->A_BI.rot);


  // Left upper arm
  Vec3d_sub(dirVec, ell->A_BI.org, shl->A_BI.org);
  Vec3d_normalizeSelf(dirVec);
  Vec3d_rotateSelf(dirVec, A_SI);
  upperArmLeftDir = std::vector<double>(dirVec, dirVec + 3);

  // Left forearm
  Vec3d_sub(dirVec, hl->A_BI.org, ell->A_BI.org);
  Vec3d_normalizeSelf(dirVec);
  Vec3d_rotateSelf(dirVec, A_SI);
  foreArmLeftDir = std::vector<double>(dirVec, dirVec + 3);

  // Left hand rotation matrix
  handLeftOri.resize(9);
  Mat3d_copy(tmp, MAT3D_CAST hl->A_BI.rot);
  //Mat3d_postMulSelf(tmp, A_SI);
  Vec3d_rotateSelf(tmp[0], A_SI);
  Vec3d_rotateSelf(tmp[1], A_SI);
  Vec3d_rotateSelf(tmp[2], A_SI);
  Mat3d_toArray(handLeftOri.data(), tmp);// MAT3D_CAST hl->A_BI.rot);

  // Head rotation matrix
  headOri.resize(9);
  Mat3d_copy(tmp, MAT3D_CAST head->A_BI.rot);
  //Mat3d_postMulSelf(tmp, A_SI);
  Vec3d_rotateSelf(tmp[0], A_SI);
  Vec3d_rotateSelf(tmp[1], A_SI);
  Vec3d_rotateSelf(tmp[2], A_SI);
  Mat3d_toArray(headOri.data(), tmp);// MAT3D_CAST hl->A_BI.rot);

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

void PolicySkeletonComponent::onUpdateScene(RcsGraph* desired, RcsGraph* current, ActionScene* scene)
{
  nlohmann::json command;

  std::vector<double> upperArmRightDir, foreArmRightDir, handRightOri,
      upperArmLeftDir, foreArmLeftDir, handLeftOri, headOri;
  bool success = getActionHumanPostureRetarget(desired, scene,
                                               upperArmRightDir,
                                               foreArmRightDir,
                                               handRightOri,
                                               upperArmLeftDir,
                                               foreArmLeftDir,
                                               handLeftOri,
                                               headOri);

  if (success)
  {
    command["input_type"] = IKTeleOp::ActionType::PolarRetarget;
    command["right_upperarm_ori"] = upperArmRightDir;
    command["right_forearm_ori"] = foreArmRightDir;
    command["left_upperarm_ori"] = upperArmLeftDir;
    command["left_forearm_ori"] = foreArmLeftDir;

    const Rcs::Task* task = ctrlPtr->getTask("right_hand_ori");

    if (task)
    {
      if (task->getClassName()=="POLAR")
      {
        command[task->getName()] = std::vector<double>(handRightOri.begin() + 6,
                                                       handRightOri.begin() + 9);
      }
      else if (task->getClassName()=="ABC")
      {
        command[task->getName()] = std::vector<double>(handRightOri.begin(),
                                                       handRightOri.begin() + 6);
      }
      else
      {
        RLOG_CPP(1, "Task " << task->getName() << " is not of type POLAR or ABC");
      }
    }
    else
    {
      RLOG_CPP(1, "Task right_hand_ori not found!");
    }

    task = ctrlPtr->getTask("left_hand_ori");
    if (task)
    {
      if (task->getClassName()=="POLAR")
      {
        command[task->getName()] = std::vector<double>(handLeftOri.begin() + 6,
                                                       handLeftOri.begin() + 9);
      }
      else if (task->getClassName()=="ABC")
      {
        command[task->getName()] = std::vector<double>(handLeftOri.begin(),
                                                       handLeftOri.begin() + 6);
      }
      else
      {
        RLOG_CPP(1, "Task " << task->getName() << " is not of type POLAR or ABC");
      }
    }
    else
    {
      RLOG_CPP(1, "Task left_hand_ori not found!");
    }

    task = ctrlPtr->getTask("head_ori");
    if (task)
    {
      if (task->getClassName()=="POLAR")
      {
        command[task->getName()] = std::vector<double>(headOri.begin()+6, headOri.begin()+9);
      }
      else if (task->getClassName()=="ABC")
      {
        command[task->getName()] = std::vector<double>(headOri.begin(), headOri.begin()+6);
      }
      else
      {
        RLOG_CPP(1, "Task " << task->getName() << " is not of type POLAR or ABC");
      }
    }
    else
    {
      RLOG_CPP(1, "Task head_ori not found!");
    }

  }   // if (success)


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
