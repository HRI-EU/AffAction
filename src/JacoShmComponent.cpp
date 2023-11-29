/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "JacoShmComponent.h"

#if !defined (_MSC_VER)

#include "JacoShm.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_math.h>
#include <Rcs_cmdLine.h>

#include <cmath>
#include <pwd.h>
#include <grp.h>
#include <sys/stat.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <map>




namespace aff
{




JacoShmComponent::JacoShmComponent(const RcsGraph* graph, JacoType roboType) :
  jType(roboType)
{
  this->shm = new JacoShm(JACO_SHM_NAME);

  // Joints naming convention
  switch (this->jType)
  {
    case Jaco7_right:
      jntNames.push_back("j2s7s300_joint_1_right");
      jntNames.push_back("j2s7s300_joint_2_right");
      jntNames.push_back("j2s7s300_joint_3_right");
      jntNames.push_back("j2s7s300_joint_4_right");
      jntNames.push_back("j2s7s300_joint_5_right");
      jntNames.push_back("j2s7s300_joint_6_right");
      jntNames.push_back("j2s7s300_joint_7_right");
      jntNames.push_back("j2s7s300_joint_finger_1_right");
      jntNames.push_back("j2s7s300_joint_finger_2_right");
      jntNames.push_back("j2s7s300_joint_finger_3_right");
      break;

    case Jaco7_left:
      jntNames.push_back("j2s7s300_joint_1_left");
      jntNames.push_back("j2s7s300_joint_2_left");
      jntNames.push_back("j2s7s300_joint_3_left");
      jntNames.push_back("j2s7s300_joint_4_left");
      jntNames.push_back("j2s7s300_joint_5_left");
      jntNames.push_back("j2s7s300_joint_6_left");
      jntNames.push_back("j2s7s300_joint_7_left");
      jntNames.push_back("j2s7s300_joint_finger_1_left");
      jntNames.push_back("j2s7s300_joint_finger_2_left");
      jntNames.push_back("j2s7s300_joint_finger_3_left");
      break;

    default:
      RFATAL("Unknown JacoType: %d", jType);
  }


  for (size_t i=0; i<jntNames.size(); ++i)
  {
    jntMap[jntNames[i]] = JointData(i);
    const RcsJoint* ji = RcsGraph_getJointByName(graph, jntNames[i].c_str());
    RCHECK_MSG(ji, "Joint with name \"%s\" not found", jntNames[i].c_str());
    jntMap[jntNames[i]].q0 = graph->q->ele[ji->jointIndex];
    RLOG(0, "%s: q0=%.3f deg", jntNames[i].c_str(), RCS_RAD2DEG(jntMap[jntNames[i]].q0));
  }


  switch (this->jType)
  {
    case Jaco7_right:
      jntMap["j2s7s300_joint_1_right"].limitlessJoint = true;
      jntMap["j2s7s300_joint_3_right"].limitlessJoint = true;
      jntMap["j2s7s300_joint_5_right"].limitlessJoint = true;
      jntMap["j2s7s300_joint_7_right"].limitlessJoint = true;
      break;

    case Jaco7_left:
      jntMap["j2s7s300_joint_1_left"].limitlessJoint = true;
      jntMap["j2s7s300_joint_3_left"].limitlessJoint = true;
      jntMap["j2s7s300_joint_5_left"].limitlessJoint = true;
      jntMap["j2s7s300_joint_7_left"].limitlessJoint = true;
      break;

    default:
      RFATAL("Unknown JacoType: %d", jType);
  }


  // Initialize joint indices
  int nFound = 0, nExpected = 10;
  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    std::map<std::string,JointData>::iterator it = jntMap.find(JNT->name);
    if (it != jntMap.end())
    {
      it->second.jointIdx = JNT->jointIndex;
      nFound++;
    }
  }

  RCHECK(nFound==nExpected);
}

JacoShmComponent::~JacoShmComponent()
{
  delete this->shm;
}

void JacoShmComponent::updateSensors(RcsGraph* graph)
{
  KinovaShm data;

  // Update sensors from shared memory
  shm->lock();
  memcpy(&data, shm->data, sizeof(KinovaShm));
  shm->unlock();

  // Update named map
  const double* currPos;
  const double* currVel;

  switch (this->jType)
  {
    case Jaco7_right:
    {
      currPos = data.right.currPos;
      currVel = data.right.currVel;
    }
    break;

    case Jaco7_left:
    {
      currPos = data.left.currPos;
      currVel = data.left.currVel;
    }
    break;

    default:
      RFATAL("Unknown JacoType: %d", jType);
  }

  // Compute initial flips
  if (jntOffset.empty())
  {
    jntOffset.resize(jntNames.size(), 0.0);

    for (size_t i=0; i<jntNames.size(); ++i)
    {
      // We only consider offsets for limitless joints, the others will bump into limits.
      if (!jntMap[jntNames[i]].limitlessJoint)
      {
        continue;
      }

      double q_current = currPos[i];
      double q0 = jntMap[jntNames[i]].q0;

      if (q_current-q0>M_PI)
      {
        jntOffset[i] = -2.0*M_PI;
        RLOG(0, "%s: Substracting 2 PI: %.3f %.3f deg",
             jntNames[i].c_str(), RCS_RAD2DEG(q_current), RCS_RAD2DEG(q0));
      }
      else if (q_current-q0<-M_PI)
      {
        jntOffset[i] = 2.0*M_PI;
        RLOG(0, "%s: Adding 2 PI: %.3f %.3f deg",
             jntNames[i].c_str(), RCS_RAD2DEG(q_current), RCS_RAD2DEG(q0));
      }
      else
      {
        jntOffset[i] = 0.0;
        RLOG(0, "%s: No offset: %.3f %.3f deg",
             jntNames[i].c_str(), RCS_RAD2DEG(q_current), RCS_RAD2DEG(q0));
      }


    }

  }

  // for (size_t i=0; i<jntNames.size(); ++i)
  // {
  //   RLOG(0, "%s: q_curr = %.3f deg", jntNames[i].c_str(), RCS_RAD2DEG(currPos[i]));
  // }


  for (size_t i=0; i<jntNames.size(); ++i)
  {
    jntMap[jntNames[i]].setCurrent(currPos[i]+jntOffset[i], currVel[i]);
  }

  // Update graph from map
  std::map<std::string,JointData>::iterator it = jntMap.begin();

  while (it != jntMap.end())
  {
    const int jidx = it->second.jointIdx;

    if (jidx!=-1)
    {
      graph->q->ele[jidx] = it->second.getCurrentJointAngle();
      graph->q_dot->ele[jidx] = it->second.getCurrentJointVelocity();
    }
    it++;
  }

}

void JacoShmComponent::setCommand(const MatNd* q_des)
{
  std::vector<double> q_cmd(jntNames.size());
  std::map<std::string,JointData>::iterator it = jntMap.begin();

  while (it != jntMap.end())
  {
    const int jidx = it->second.jointIdx;

    if (jidx!=-1)
    {
      q_cmd[it->second.arrayIdx] = q_des->ele[jidx]-jntOffset[it->second.arrayIdx];
    }
    it++;
  }

  // REXEC(0)
  // {
  //   //VecNd_printComment("q_cmd", q_cmd.data(), jntNames.size());

  //   for (size_t i=0; i<jntNames.size(); ++i)
  //   {
  //     RLOG(0, "%s: q_cmd = %.3f deg", jntNames[i].c_str(), RCS_RAD2DEG(q_cmd[i]));
  //   }
  // }


  // Write comand to shared memory
  switch (this->jType)
  {
    case Jaco7_right:
      shm->lock();
      VecNd_copy(shm->data->right.desPos, q_cmd.data(), q_cmd.size());
      shm->unlock();
      break;

    case Jaco7_left:
      shm->lock();
      VecNd_copy(shm->data->left.desPos, q_cmd.data(), q_cmd.size());
      shm->unlock();
      break;

    default:
      RFATAL("Unknown JacoType: %d", jType);
  }

}







/*******************************************************************************
 *

  if (argP.hasArgument("-jacoShm7r", "Start with Jaco7 Shm right") && (!dryRun))
  {
    hwc.push_back(RoboJacoShmComponent::create(&entity, graph, Rcs::JacoShmComponent::Jaco7_right));
  }

  if (argP.hasArgument("-jacoShm7l", "Start with Jaco7 Shm left") && (!dryRun))
  {
    hwc.push_back(RoboJacoShmComponent::create(&entity, graph, Rcs::JacoShmComponent::Jaco7_left));
  }



 ******************************************************************************/
RoboJacoShmComponent* RoboJacoShmComponent::create(EntityBase* parent,
                                                   const RcsGraph* graph,
                                                   JacoShmComponent::JacoType jt)
{
  RCHECK(graph);
  return new RoboJacoShmComponent(parent, graph, jt);
}

RoboJacoShmComponent::RoboJacoShmComponent(EntityBase* parent,
                                           const RcsGraph* graph,
                                           JacoShmComponent::JacoType jt) :
  ComponentBase(parent), JacoShmComponent(graph, jt), eStop(false), enableCommands(false)
{
  RLOG(0, "Creating RoboJacoShmComponent");
  subscribe("UpdateGraph", &RoboJacoShmComponent::onUpdateGraph);
  subscribe("SetJointCommand", &RoboJacoShmComponent::onSetJointPosition);
  subscribe("InitFromState", &RoboJacoShmComponent::onInitFromState);
  subscribe("EmergencyStop", &RoboJacoShmComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &RoboJacoShmComponent::onEmergencyRecover);
  subscribe("EnableCommands", &RoboJacoShmComponent::onEnableCommands);
}

void RoboJacoShmComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(0, "RoboJacoComponent::onInitFromState()");
  onSetJointPosition(target->q);
}

void RoboJacoShmComponent::onEmergencyStop()
{
  if (this->eStop == false)
  {
    RLOG(0, "RoboJacoComponent::EmergencyStop");
  }

  this->eStop = true;
  enableCommands = false;
}

void RoboJacoShmComponent::onEmergencyRecover()
{
  RLOG(0, "RoboJacoComponent::EmergencyRecover");
  this->eStop = false;
  enableCommands = true;
}

void RoboJacoShmComponent::onSetJointPosition(const MatNd* q_des)
{
  if ((enableCommands) && (!eStop))
  {
    JacoShmComponent::setCommand(q_des);
  }
}

void RoboJacoShmComponent::onUpdateGraph(RcsGraph* graph)
{
  JacoShmComponent::updateSensors(graph);
}

void RoboJacoShmComponent::onEnableCommands()
{
  enableCommands = true;
}




}   // namespace aff

#endif   //  !defined (_MSC_VER)
