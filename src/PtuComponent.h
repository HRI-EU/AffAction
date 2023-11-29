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

#ifndef AFF_PTUCOMPONENT_H
#define AFF_PTUCOMPONENT_H

#include "ComponentBase.h"

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_ROS)


#include <Rcs_filters.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <thread>


namespace aff
{

class PtuComponent : public ComponentBase
{
public:

  PtuComponent(EntityBase* parent, double rosCbDt);

private:

  void rosCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void onUpdateGraph(RcsGraph* graph);
  void setJointPosition(const MatNd* q_des);

public:

  void onStart();

protected:

  virtual void waitForInitialization();
  void onStop();

  std::string topicNameState;
  std::string topicNameCommand;
  std::string panJointName;
  std::string tiltJointName;
  double panJointAngle;
  double tiltJointAngle;
  Rcs::RampFilter1D* panFilt;
  Rcs::RampFilter1D* tiltFilt;
  int panJointIdx;
  int tiltJointIdx;
  ros::Subscriber jntSubscriber;
  ros::Publisher ptu_pub;
  std::unique_ptr<ros::NodeHandle> nh;
  std::mutex panTiltMtx;
  bool desiredValuesInitialized;
  double rosCallbackDt;
};

}   // namespace

#else   // defined USE_ROS

namespace aff
{

class PtuComponent : public ComponentBase
{
public:
  PtuComponent(EntityBase* parent, double rosCbDt);
};

}   // namespace

#endif   // defined USE_ROS

#endif   // AFF_PTUCOMPONENT_H
