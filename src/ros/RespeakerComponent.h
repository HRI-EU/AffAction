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

#ifndef AFF_RESPEAKERCOMPONENT_H
#define AFF_RESPEAKERCOMPONENT_H

#include "ComponentBase.h"
#include "ActionScene.h"

#if defined (USE_ROS)
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <audio_msgs/TalkFlag.h>
#include <mutex>
#endif


namespace aff
{

class RespeakerComponent : public ComponentBase
{
public:

  RespeakerComponent(EntityBase* parent, const ActionScene* scene);

  virtual ~RespeakerComponent();

  /*! \brief Currently these parameters can be set:
   *         - PublishDialogueWithRaisedHandOnly (default is true)
   *         - GazeAtSpeaker                     (default is true)
   *         - SpeakOutSpeakerListenerText       (default is false)
   */
  bool setParameter(const std::string& parameterName, bool flag);

  /*! \brief Currently these parameters can be set:
   *         - HandAboveHeadThreshold             (default is 10.0 meters, which essentially means that the hand-over-head speaker detection is not active. A good default would be 0.0)
   */
  bool setParameter(const std::string& parameterName, double value);

private:

  void setPublishDialogueWithRaisedHandOnly(bool enable);
  bool getPublishDialogueWithRaisedHandOnly() const;
  void setGazeAtSpeaker(bool enable);
  bool getGazeAtSpeaker() const;
  void setSpeakOutSpeakerListenerText(bool enable);
  bool getSpeakOutSpeakerListenerText() const;
  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);
  void onStart();
  void onStop();
  void onResetLLM();
  void onReplayLog();
  void enableASR(bool enable);
  void enableSoundDirectionEstimation(bool enable);
  void toggleASR();
  void toggleHandRaised();
  const HumanAgent* getSpeaker(const double micPosition[3],
                               const double soundDir[3],
                               const RcsGraph* graph) const;
  const Agent* getListener(const double micPosition[3],
                           const HumanAgent* speaker,
                           const RcsGraph* graph);
  void updateSoundDirection(RcsGraph* graph, const std::string& spoken,
                            double micPos[3], double soundDir[3]);

#if defined (USE_ROS)

  void isSpeakingRosCallback(const std_msgs::Bool::ConstPtr& msg);
  void asrRosCallback(const std_msgs::String::ConstPtr& msg);
  void soundLocalizationRosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  ros::Subscriber asrSubscriber;
  ros::Subscriber soundLocalizationSubscriber;
  ros::Subscriber isSpeakingSubscriber;
  ros::Publisher reset_llm_pub;
  ros::Publisher robot_should_listen_pub;
  ros::Publisher dialogue_pub;
  std::unique_ptr<ros::NodeHandle> nh;

#endif

  const ActionScene* scene;
  std::string receivedTextJson;
  std::string respeakerBdyName;
  std::mutex textLock;
  bool isASREnabled;
  bool isSoundDirectionEstimationEnabled;
  bool isSomebodySpeaking;
  bool isAnyHandRaised;
  bool isAnyHandRaisedOverride;
  bool publishDialogueWithRaisedHandOnly;
  bool gazeAtSpeaker;
  bool speakOutSpeakerListenerText;
  double handAboveHeadThreshold;
  std::vector<double> soundDirection, soundDirectionFilt;
};

}   // namespace

#endif   // AFF_RESPEAKERCOMPONENT_H
