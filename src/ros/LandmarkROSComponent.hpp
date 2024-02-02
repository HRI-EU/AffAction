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

#ifndef LANDMARKROSCOMPONENT_H
#define LANDMARKROSCOMPONENT_H

#include "LandmarkBase.h"
#include "ArucoTracker.h"

#include <ComponentBase.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_utils.h>

#include <ros/ros.h>
#include <std_msgs/String.h>




namespace aff
{

class LandmarkROSComponent : public ComponentBase, public LandmarkBase
{
public:

  LandmarkROSComponent(EntityBase* parent, std::string topic_="/landmarks/camera"):
    ComponentBase(parent), LandmarkBase(), topic(topic_)
  {
    subscribe("Start", &LandmarkROSComponent::onStart);
    subscribe("Stop", &LandmarkROSComponent::onStop);
    subscribe("PostUpdateGraph", &LandmarkBase::onPostUpdateGraph);
    subscribe("FreezePerception", &LandmarkBase::onFreezePerception);
    //subscribe("EstimateCameraPose", &LandmarkROSComponent::onEstimateCameraPose);
  }

  virtual ~LandmarkROSComponent()
  {
  }

  void onEstimateCameraPose(int numFrames)
  {
    for (auto& t : trackers)
    {
      ArucoTracker* at = dynamic_cast<ArucoTracker*>(t.get());

      if (at)
      {
        RLOG(0, "Calibrating Aruco camera");
        at->calibrate(numFrames);
      }
    }
  }

  void onStart()
  {
    RLOG_CPP(0, "LandmarkROSComponent::onStart()");
    if (!nh)
    {
      RLOG(0, "Creating node handle");
      nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

      RLOG_CPP(1, "Subscribing to " << topic);
      this->landmarkSubscriber = nh->subscribe(topic, 1, &LandmarkROSComponent::landmarkCallback, this);
      RLOG(0, "Done subscribing");
    }
  }

  void onStop()
  {
    if (nh)
    {
      this->landmarkSubscriber.shutdown();
    }
  }

  void landmarkCallback(const std_msgs::String::ConstPtr& msg)
  {
    RLOG(1, "landmarkCallback");
    if (!frozen)
    {
      nlohmann::json json = nlohmann::json::parse(msg->data);
      setJsonInput(json);
    }
  }

private:

  std::unique_ptr<ros::NodeHandle> nh;
  ros::Subscriber landmarkSubscriber;
  std::string topic;
};


} // namespace

#endif // LANDMARKROSCOMPONENT_H
