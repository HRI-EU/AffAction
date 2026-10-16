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

#include "LeRobotComponent.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_resourcePath.h>


namespace aff
{

LeRobotComponent::LeRobotComponent(EntityBase* parent,
                                   const std::string& dataCollectionXmlFileName) :
  ComponentBase(parent)
{
  subscribe("SetPolicyCommand", &LeRobotComponent::onIncomingCommand);
  subscribe("InitFromState", &LeRobotComponent::onInitFromState);

  if (!Rcs_fileInResourcePath(dataCollectionXmlFileName.c_str()))
  {
    RFATAL("Data collection file \"%s\" not found!", dataCollectionXmlFileName.c_str());
  }

  lerobot_ctrl_curr = std::make_unique<Rcs::ControllerBase>(dataCollectionXmlFileName.c_str());
  lerobot_ctrl_des = std::make_unique<Rcs::ControllerBase>(dataCollectionXmlFileName.c_str());
}


void LeRobotComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(1, "LeRobotComponent::onInitFromState()");
  RcsGraph_copy(lerobot_ctrl_curr->getGraph(), target);
  RcsGraph_copy(lerobot_ctrl_des->getGraph(), target);
}

void LeRobotComponent::updateData(RcsGraph* desired, RcsGraph* current)
{
  if (desired->dof != lerobot_ctrl_des->getGraph()->dof)
  {
    RLOG_CPP(1, "Mismatch between desired and data collecting graph: dofs " <<
             desired->dof << " != " << lerobot_ctrl_des->getGraph()->dof);
    return;
  }

  if (current->dof != lerobot_ctrl_curr->getGraph()->dof)
  {
    RLOG_CPP(1, "Mismatch between current and data collecting graph: dofs " <<
             current->dof << " != " << lerobot_ctrl_curr->getGraph()->dof);
    return;
  }

  std::lock_guard<std::mutex> lock(this->leRobotStateMtx);
  this->lerobot_state_curr = std::vector<double>(current->q->ele, current->q->ele+current->q->size);
  this->lerobot_state_des = std::vector<double>(desired->q->ele, desired->q->ele+desired->q->size);
}


std::map<std::string, double> LeRobotComponent::computeLeRobotData(bool current)
{
  Rcs::ControllerBase* controller = current ? lerobot_ctrl_curr.get() : lerobot_ctrl_des.get();
  const double* q_ptr = current ? lerobot_state_curr.data() : lerobot_state_des.data();


  // Short copy inside mutex. We assume all state is represented in the q-vector
  {
    std::lock_guard<std::mutex> lock(this->leRobotStateMtx);
    MatNd_fromArray(controller->getGraph()->q, q_ptr, controller->getGraph()->dof);
  }


  // Compute kinematics and update task state
  std::vector<double> taskVec(controller->getTaskDim());

  RcsGraph_setState(controller->getGraph(),
                    controller->getGraph()->q,
                    controller->getGraph()->q_dot);

  MatNd x_task = MatNd_fromPtr(taskVec.size(), 1, taskVec.data());
  controller->computeX(&x_task);

  // Assign to collected data
  std::map<std::string, double> data;

  // Joint state
  // data["joint_left_shoulder_joint"] -> double
  for (size_t i=0; i<controller->getGraph()->dof; ++i)
  {
    const RcsJoint* jnt = &controller->getGraph()->joints[i];
    std::string key = "joint." + std::string(jnt->name);
    data[key] = controller->getGraph()->q->ele[jnt->jointIndex];
  }

  // Task state
  // data["task_left_hand_ori_0"] -> double
  for (size_t i = 0; i < controller->getNumberOfTasks(); i++)
  {
    const Rcs::Task* task = controller->getTask(i);
    const size_t taskArrayIdx = controller->getTaskArrayIndex(task);

    // Truncated rotation matrix for Euler angles (first and second row)
    if (task->getClassName() == "ABC")
    {
      double rm[3][3];
      const double* eulerAngles = &x_task.ele[taskArrayIdx];
      Mat3d_fromEulerAngles(rm, eulerAngles);
      for (size_t j = 0; j < 2; j++)
        for (size_t k = 0; k < 3; k++)
        {
          const size_t idx = j * 3 + k;
          std::string key = "task." + task->getName() + "_" + std::to_string(idx);
          data[key] = rm[j][k];
        }
    }
    // Direction vector for Polar angles
    else if (task->getClassName() == "POLAR")
    {
      double axis[3];
      const double* polarAngles = &x_task.ele[taskArrayIdx];
      Vec3d_getPolarAxis(axis, polarAngles[0], polarAngles[1]);
      for (size_t j = 0; j < 3; j++)
      {
        std::string key = "task." + task->getName() + "_" + std::to_string(j);
        data[key] = axis[j];
      }
    }
    // Element-wise task elements
    else
    {
      for (size_t j = 0; j < task->getDim(); j++)
      {
        std::string key = "task." + task->getName() + "_" + std::to_string(j);
        data[key] = x_task.ele[taskArrayIdx + j];
      }
    }
  }

  // Sensor state
  for (size_t i = 0; i < controller->getGraph()->nSensors; ++i)
  {
    const RcsSensor* s = &controller->getGraph()->sensors[i];

    for (size_t j = 0; j < s->rawData->size; j++)
    {
      std::string key = "sensor." + std::string(s->name) + "_" + std::to_string(j);
      data[key] = s->rawData->ele[j];
    }

  }

  return data;
}

void LeRobotComponent::onIncomingCommand(nlohmann::json command)
{
  std::lock_guard<std::mutex> lock(this->incomingCommandMtx);
  this->incomingCommand = command;
}

nlohmann::json LeRobotComponent::getIncomingCommand() const
{
  std::lock_guard<std::mutex> lock(this->incomingCommandMtx);
  return this->incomingCommand;
}


}   // namespace aff
