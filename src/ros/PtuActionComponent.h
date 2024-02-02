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

#ifndef AFF_PTUACTIONCOMPONENT_H
#define AFF_PTUACTIONCOMPONENT_H

#include "ComponentBase.h"

#include <Rcs_graph.h>

#if defined (USE_ROS)
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hri_scitos_schunk_ptu/PtuGotoAction.h>
#include <sensor_msgs/JointState.h>
#endif


namespace aff
{

class PtuActionComponent : public ComponentBase
{
public:

  PtuActionComponent(EntityBase* parent);

#if defined (USE_ROS)
  virtual ~PtuActionComponent();

private:
  void onStart();
  void onStop();
  void onPtuCommand(double panInRad, double tiltInRad);
  void onLookAtCommand(const std::string& bodyName);
  void onUpdateGraph(RcsGraph* graph);
  void onPostUpdateGraph(RcsGraph* current, RcsGraph* desired);
  void rosCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void getPanTilt(std::string gazeTarget);
  void activeCallback();
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const hri_scitos_schunk_ptu::PtuGotoResultConstPtr& result);

  actionlib::SimpleActionClient<hri_scitos_schunk_ptu::PtuGotoAction> actionClient;
  std::unique_ptr<ros::NodeHandle> nh;
  ros::Subscriber ptuPanTiltSubscriber;
  std::string panJointName;
  std::string tiltJointName;
  std::string lookAtBody;
  double panJointAngle;
  double tiltJointAngle;
  std::mutex panTiltMtx;
  bool panTiltUpdated;
  RcsGraph* copyOfGraph;

  double pan_min_degrees;
  double pan_max_degrees;
  double tilt_min_degrees;
  double tilt_max_degrees;

#endif   // defined USE_ROS
};

}   // namespace


#endif   // AFF_PTUACTIONCOMPONENT_H
