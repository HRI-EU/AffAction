/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

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

#include "PtuComponent.h"
#include <Rcs_macros.h>



/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_ROS)

#include "ComponentBase.h"

#include <Rcs_typedef.h>
#include <Rcs_cmdLine.h>
#include <Rcs_timer.h>



namespace aff
{

PtuComponent::PtuComponent(EntityBase* parent, double rosCbDt) :
  ComponentBase(parent), topicNameState("/ptu/state"), topicNameCommand("/ptu/velocity_command"),
  panJointName("ptu_pan_joint"), tiltJointName("ptu_tilt_joint"),
  panJointAngle(0.0), tiltJointAngle(0.0),
  panFilt(NULL), tiltFilt(NULL),
  panJointIdx(-1), tiltJointIdx(-1), desiredValuesInitialized(false),
  rosCallbackDt(rosCbDt)
{
  subscribe("Start", &PtuComponent::onStart);
  subscribe("Stop", &PtuComponent::onStop);
  subscribe("UpdateGraph", &PtuComponent::onUpdateGraph);
  subscribe("SetJointCommand", &PtuComponent::setJointPosition);
}

void PtuComponent::rosCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  bool panUpdated = false, tiltUpdated = false;

  for (size_t i=0; i<msg->name.size(); ++i)
  {

    if (msg->name[i]==panJointName)
    {
      NLOG(0, "Joint %zu is %s %f", i, msg->name[i].c_str(), msg->position[i]);
      std::lock_guard<std::mutex> lock(panTiltMtx);
      panJointAngle = msg->position[i];
      panUpdated = true;
    }
    else if (msg->name[i]==tiltJointName)
    {
      NLOG(0, "Joint %zu is %s %f", i, msg->name[i].c_str(), msg->position[i]);
      std::lock_guard<std::mutex> lock(panTiltMtx);
      tiltJointAngle = msg->position[i];
      tiltUpdated = true;
    }
    else
    {
      RLOG(0, "Joint %zu \"%s\" is not found", i, msg->name[i].c_str());
    }

  }

  // Initialize the desired values after the first ROS joint state message
  if (!this->desiredValuesInitialized)
  {
    if (panUpdated && tiltUpdated)
    {
      this->desiredValuesInitialized = true;
      const double tmc = 0.1;
      const double vmax = RCS_DEG2RAD(30.0);
      this->panFilt = new Rcs::RampFilter1D(panJointAngle, tmc, vmax, this->rosCallbackDt);
      this->tiltFilt = new Rcs::RampFilter1D(tiltJointAngle, tmc, vmax, this->rosCallbackDt);
    }
    return;
  }

  // Send commands only after the desired values have been initialized
  panFilt->iterate();
  tiltFilt->iterate();

  sensor_msgs::JointState cmd;

  if (panJointIdx!=-1)
  {
    const double gain = 0.5;
    double vel = panFilt->getVelocity() + gain*(panFilt->getPosition()-panJointAngle);
    cmd.name.push_back(panJointName);
    cmd.position.push_back(0.0);
    cmd.velocity.push_back(vel);
    RLOG(1, "Pan: %f", RCS_RAD2DEG(vel));
  }

  if (tiltJointIdx!=-1)
  {
    const double gain = 0.5;
    double vel = tiltFilt->getVelocity() + gain*(tiltFilt->getPosition()-tiltJointAngle);
    cmd.name.push_back(tiltJointName);
    cmd.position.push_back(0.0);
    cmd.velocity.push_back(vel);
    RLOG(1, "Tilt: %f", RCS_RAD2DEG(vel));
  }

  if (!cmd.name.empty())
  {
    RLOG(1, "Publishing Pan and Tilt");
    ptu_pub.publish(cmd);
  }

}

void PtuComponent::onUpdateGraph(RcsGraph* graph)
{

  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    if (STREQ(panJointName.c_str(), JNT->name))
    {
      NLOG(0, "q[%zu] = %f", JNT->jointIndex, panJointAngle);
      std::lock_guard<std::mutex> lock(panTiltMtx);
      MatNd_set(graph->q, JNT->jointIndex, 0, panJointAngle);
      panJointIdx = JNT->jointIndex;
    }

    if (STREQ(tiltJointName.c_str(), JNT->name))
    {
      NLOG(0, "q[%zu] = %f", JNT->jointIndex, panJointAngle);
      std::lock_guard<std::mutex> lock(panTiltMtx);
      MatNd_set(graph->q, JNT->jointIndex, 0, tiltJointAngle);
      tiltJointIdx = JNT->jointIndex;
    }

  }

}

void PtuComponent::setJointPosition(const MatNd* q_des)
{
  std::lock_guard<std::mutex> lock(panTiltMtx);

  if ((panJointIdx!=-1) && (panFilt))
  {
    panFilt->setTarget(MatNd_get(q_des, panJointIdx, 0));
  }

  if ((tiltJointIdx!=-1) && (tiltFilt))
  {
    tiltFilt->setTarget(MatNd_get(q_des, tiltJointIdx, 0));
  }

}

void PtuComponent::onStart()
{
  RLOG_CPP(5, "PtuComponent::onStart():: Subscribing to " << topicNameState);
  if (!nh)
  {
    RLOG(0, "Creating node handle");
    nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  }

  RLOG_CPP(5, "Subscribing to topic " << topicNameState);
  this->jntSubscriber = nh->subscribe(topicNameState, 1, &PtuComponent::rosCallback, this);
  RLOG(5, "Done subscribing");

  waitForInitialization();
  RLOG(0, "PTU initialized");

  // From here on, pan and tilt targets are initialized
  RLOG_CPP(0, "Advertising topic " << topicNameCommand);
  this->ptu_pub = nh->advertise<sensor_msgs::JointState>(topicNameCommand, 1);
}

void PtuComponent::waitForInitialization()
{
  while (!desiredValuesInitialized)
  {
    usleep(10000);
    RLOG(0, "Waiting for PTU initialization from ROS");
  }
}

void PtuComponent::onStop()
{
  RLOG(0, "PtuComponent::onStop()");
  if (nh)
  {
    this->jntSubscriber.shutdown();
  }

  this->desiredValuesInitialized = false;
}

}   // namespace

#else   // defined USE_ROS

namespace aff
{

PtuComponent::PtuComponent(EntityBase* parent, double rosCbDt) : ComponentBase(parent)
{
  RFATAL("You are trying to use the PTU, but it has not been compiled in");
}

}   // namespace

#endif   // defined USE_ROS
