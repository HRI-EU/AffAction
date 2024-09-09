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


/*******************************************************************************
 * Buffer used for sending audio to Google. Larger values increase the audio latency.
 * AUDIO_FRAMES_PER_BUFFER=384
 *
 * Console 1:
 * roscore
 *
 * Console 2:
 * roslaunch --wait respeaker_ros respeaker.launch frames_per_buffer:=384
 *
 * Console 3:
 * roslaunch google_asr google_asr.launch hint_filename:=$(rospack find smile)/config/google_asr/hints.txt with_localization:=true
 *
 * Console 3:
 * bin/TestLLMSim -dir config/xml/Affaction/examples/ -f g_dialogue.xml  -tracking -r_agent 0.5 -respeaker
 *
 * Console 4:
 * ~/localdisk/camera_tracking > ./scripts/azure_tracking_socket.py --visualize --aruco --body
 *
 * Manually toggle talk flag:
 * rostopic pub /robot_should_listen audio_msgs/TalkFlag "active: true
 * start_stamp:
 *   secs: 0
 *   nsecs: 0"
 *
 * rostopic pub /robot_should_listen audio_msgs/TalkFlag "active: false
 * start_stamp:
 *   secs: 0
 *   nsecs: 0"
 *
 ******************************************************************************/

#include "RespeakerComponent.h"
#include "json.hpp"

#include <Rcs_typedef.h>
#include <Rcs_filters.h>
#include <Rcs_cmdLine.h>
#include <Rcs_timer.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>
#include <Rcs_quaternion.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>

#if defined (USE_ROS)
#include <audio_msgs/AudioDataStamped.h>
#include <audio_msgs/TalkFlag.h>
#endif

#define AGENT_DETECTION_CONE_ANGLE (30.0*M_PI/180.0)

namespace aff
{

RespeakerComponent::RespeakerComponent(EntityBase* parent, const ActionScene* scene_) :
  ComponentBase(parent), scene(scene_), respeakerBdyName("respeaker"), isASREnabled(false),
  isSoundDirectionEstimationEnabled(true), isAnyHandRaised(false), isAnyHandRaisedOverride(false),
  publishDialogueWithRaisedHandOnly(true), handAboveHeadThreshold(10.0)
{
  soundDirectionROS.resize(3);
  soundDirectionROS[0] = 0.0;
  soundDirectionROS[1] = 1.0;
  soundDirectionROS[2] = 0.0;
  soundDirectionFilt = soundDirectionROS;
  RCHECK(scene);
  subscribe("Start", &RespeakerComponent::onStart);
  subscribe("Stop", &RespeakerComponent::onStop);
  subscribe("PostUpdateGraph", &RespeakerComponent::onPostUpdateGraph);
  subscribe("EnableASR", &RespeakerComponent::enableASR);
  subscribe("ToggleASR", &RespeakerComponent::toggleASR);
  subscribe("ToggleHandRaised", &RespeakerComponent::toggleHandRaised);
  subscribe("EnableSoundDirection", &RespeakerComponent::enableSoundDirectionEstimation);
  subscribe("AgentChanged", &RespeakerComponent::onAgentChanged);
  onStart();
}

RespeakerComponent::~RespeakerComponent()
{
  enableASR(false);
  enableSoundDirectionEstimation(true);
}

void RespeakerComponent::onStart()
{
#if defined (USE_ROS)

  RLOG_CPP(1, "RespeakerComponent::onStart()");
  if (!nh)
  {
    RLOG(1, "Creating node handle");
    nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

    RLOG_CPP(1, "Subscribing to /sound_localization");
    this->soundLocalizationSubscriber = nh->subscribe("/sound_localization", 10, &RespeakerComponent::soundLocalizationRosCallback, this);
    this->asrSubscriber = nh->subscribe("/word", 10, &RespeakerComponent::asrRosCallback, this);
    this->robotShouldListenSubscriber = nh->subscribe("/robot_should_listen", 10, &RespeakerComponent::talkFlagRosCallback, this);
    RLOG(1, "Done subscribing");

    RLOG_CPP(1, "Advertising topic /robot_should_listen");
    this->robot_should_listen_pub = nh->advertise<audio_msgs::TalkFlag>("/robot_should_listen", 1);
    this->event_pub = nh->advertise<std_msgs::String>("/event_simulation", 10);
    RLOG_CPP(1, "Done RespeakerComponent::onStart()");
  }

#endif
}

bool RespeakerComponent::setParameter(const std::string& parameterName, bool flag)
{
  if (parameterName=="PublishDialogueWithRaisedHandOnly")
  {
    setPublishDialogueWithRaisedHandOnly(flag);
    return true;
  }

  RLOG_CPP(1, "Parameter " << parameterName << " not supported - can't set to " << flag);

  return false;
}

bool RespeakerComponent::setParameter(const std::string& parameterName, double value)
{
  if (parameterName=="HandAboveHeadThreshold")
  {
    handAboveHeadThreshold = value;
    return true;
  }

  RLOG_CPP(1, "Parameter " << parameterName << " not supported - can't set to value "
           << value);

  return false;
}

void RespeakerComponent::setPublishDialogueWithRaisedHandOnly(bool enable)
{
  publishDialogueWithRaisedHandOnly = enable;
}

bool RespeakerComponent::getPublishDialogueWithRaisedHandOnly() const
{
  return publishDialogueWithRaisedHandOnly;
}

void RespeakerComponent::enableSoundDirectionEstimation(bool enable)
{
  RLOG(0, "%s sound direction channel", enable ? "Enable" : "Mute");
  isSoundDirectionEstimationEnabled = enable;
}

void RespeakerComponent::enableASR(bool enable)
{
  if (enable==isASREnabled)
  {
    RLOG(0, "/robot_should_listen is already %s", enable ? "enabled" : "disabled");
    return;
  }

  RLOG(0, "Setting /robot_should_listen flag to %s", enable ? "TRUE" : "FALSE");

#if defined (USE_ROS)// \todo: ROS time
  audio_msgs::TalkFlag talkFlagMsg;
  talkFlagMsg.active = enable;
  talkFlagMsg.start_stamp.sec = 0;
  talkFlagMsg.start_stamp.nsec = 0;
  robot_should_listen_pub.publish(talkFlagMsg);
#endif
}

void RespeakerComponent::toggleASR()
{
  enableASR(!isASREnabled);
}

void RespeakerComponent::toggleHandRaised()
{
  isAnyHandRaisedOverride = !isAnyHandRaisedOverride;
}

void RespeakerComponent::getMicrophonePosition(const RcsGraph* graph, double micPos[3]) const
{
  const RcsBody* respeakerBdy = RcsGraph_getBodyByName(graph, respeakerBdyName.c_str());
  RCHECK_MSG(respeakerBdy, "Not found in graph: %s", respeakerBdyName.c_str());

  Vec3d_copy(micPos, respeakerBdy->A_BI.org);
}

// Use raised hand as sound direction (if any)
// HANDTIP_RIGHT = 16, HANDTIP_LEFT = 9, HEAD = 26
bool RespeakerComponent::isRaisedHand(const RcsGraph* graph, const HTr* A_micI, double soundDir[3]) const
{
  bool handRaised = false;

  for (const auto& agent : scene->agents)
  {
    HumanAgent* human = dynamic_cast<HumanAgent*>(agent);
    if (human && human->isVisible())
    {
      const double* headPos = human->getMarker(26).org;
      const double* lhPos = human->getMarker(9).org;
      const double* rhPos = human->getMarker(16).org;
      const double handAboveHead = 0.2 + handAboveHeadThreshold;
      if ((lhPos[2]>headPos[2]+handAboveHead) || (rhPos[2]>headPos[2]+handAboveHead))
      {
        Vec3d_invTransform(soundDir, A_micI, headPos);   // Transform into respeaker frame
        handRaised = true;
        break;
      }
    }
  }

  return handRaised;
}

void RespeakerComponent::updateSoundDirection(RcsGraph* graph)
{
  // Determine respeaker body to transform sound coordinates into world frame
  const RcsBody* respeakerBdy = RcsGraph_getBodyByName(graph, respeakerBdyName.c_str());
  RCHECK_MSG(respeakerBdy, "Not found in graph: %s", respeakerBdyName.c_str());

  // Transform the ROS sound direction from the respeaker frame into the world frame
  double soundDirectionRaw[3];
  sndDirLock.lock();
  Vec3d_copy(soundDirectionRaw, soundDirectionROS.data());
  //Vec3d_invTransform(soundDirectionRaw, &respeakerBdy->A_BI, soundDirectionROS.data());
  sndDirLock.unlock();

  // If the hand is raised, then the sound direction is overwritten with the
  // hand raising agent's head position.
  bool handRaised = isRaisedHand(graph, &respeakerBdy->A_BI, soundDirectionRaw);
  handRaised |= isAnyHandRaisedOverride;

  // In the mode when publishing with a raised hand, we enable the ASR when the
  // hand is raised, and disable it when no hand is raised.
  if (publishDialogueWithRaisedHandOnly && (handRaised!=isAnyHandRaised))
  {
    enableASR(handRaised);
  }

  isAnyHandRaised = handRaised;

  for (int i=0; i<3; ++i)
  {
    soundDirectionFilt[i] = 0.05*soundDirectionRaw[i] + 0.95*soundDirectionFilt[i];
  }

  // We show a cylinder pointing into the sound direction
  RcsBody* soundBeam = RcsGraph_getBodyByName(graph, "sound_beam");
  RCHECK(soundBeam);
  for (unsigned int i = 0; i < soundBeam->nShapes; ++i)
  {
    RcsShape_setComputeType(&soundBeam->shapes[i], RCSSHAPE_COMPUTE_GRAPHICS, true);
  }
  int jidx = RcsBody_getJointIndex(graph, soundBeam);
  RCHECK(jidx>=0);

  // In world coordinates
  HTr tmp;
  HTr_setIdentity(&tmp);
  Vec3d_copy(tmp.rot[0], soundDirectionFilt.data());   // x-axis is in horizontal plane
  Vec3d_copy(tmp.rot[2], Vec3d_ez());                  // z-axis points up
  Vec3d_crossProduct(tmp.rot[1], tmp.rot[2], tmp.rot[0]);

  // Now transform it into the respeaker frame
  // HTr_invTransformSelf(&tmp, &respeakerBdy->A_BI);

  HTr_to6DVector(&graph->q->ele[jidx], &tmp);
}

void RespeakerComponent::onPostUpdateGraph(RcsGraph* graph, RcsGraph* current)
{
  // This is mutex-protected
  std::string spoken = getAndClearTextFromROS();

  updateSoundDirection(graph);

  // Early return if nothing todo
  if (spoken.empty())
  {
    return;
  }

  double micPos[3];
  getMicrophonePosition(graph, micPos);

  // Check which agent is speaking
  const RcsBody* respeakerBdy = RcsGraph_getBodyByName(graph, respeakerBdyName.c_str());
  double sndDirInWorld[3];
  Vec3d_invTransform(sndDirInWorld, &respeakerBdy->A_BI, soundDirectionFilt.data());

  HumanAgent* speaker = getSpeaker(micPos, sndDirInWorld, graph);
  const Agent* listener = getListener(micPos, speaker, graph);
  if (speaker && listener)
  {
    speaker->setGazeTarget(listener->name);
  }

  nlohmann::json micJson = nlohmann::json::parse(spoken);
  std::string text = micJson["text"];

  if ((publishDialogueWithRaisedHandOnly && isAnyHandRaised) ||
      (!publishDialogueWithRaisedHandOnly))
  {
    nlohmann::json outJson = {};
    outJson["text"] = text;

    if (speaker)
    {
      // nlohmann::json speakerJson;
      // speakerJson["name"] = speaker->name;
      // speakerJson["instance_id"] = speaker->instanceId;
      outJson["sender"] = speaker->name;//speakerJson;
    }

    if (listener)
    {
      // nlohmann::json listenerJson;
      // listenerJson["name"] = listener->name;
      // listenerJson["instance_id"] = listener->instanceId;
      outJson["receiver"] = listener->name;//listenerJson;
    }

#if defined (USE_ROS)
    std_msgs::String dialogueResult;
    dialogueResult.data = outJson.dump();
    event_pub.publish(dialogueResult);
#endif

    RLOG_CPP(0, outJson.dump());
  }

}

void RespeakerComponent::onAgentChanged(const std::string& agentName, bool appear)
{
  std::string appearStr = appear ? " appeared" : " disappeared";
  RLOG_CPP(0, "Agent " << agentName << appearStr);

#if defined (USE_ROS)

  RLOG(0, "RespeakerComponent::onAgentChanged()");
  if (nh)
  {
    std_msgs::String resetLLMString;

    nlohmann::json json;
    json["id"] = "person_changed";

    nlohmann::json assignment;
    assignment["person"] = agentName;
    assignment["present"] = appear;

    json["assignment"] = assignment;
    json["present"] = true;
    json["publish"] = true;

    resetLLMString.data = json.dump();

    RLOG_CPP(0, "JSON: '" << resetLLMString.data << "'");
    event_pub.publish(resetLLMString);
  }

#endif
}

void RespeakerComponent::onStop()
{
#if defined (USE_ROS)

  RLOG(0, "RespeakerComponent::onStop()");
  if (nh)
  {
    this->asrSubscriber.shutdown();
    this->soundLocalizationSubscriber.shutdown();
  }

#endif
}

// Check which agent is speaking
HumanAgent* RespeakerComponent::getSpeaker(const double micPosition[3],
                                           const double soundDir[3],
                                           const RcsGraph* graph) const
{
  HumanAgent* speaker = nullptr;
  double ang = 2.0*M_PI;
  RLOG(1, "Sound direction: %.3f %.3f %.3f", soundDir[0], soundDir[1], soundDir[2]);

  for (const auto& agent : scene->agents)
  {
    HumanAgent* human = dynamic_cast<HumanAgent*>(agent);
    RLOG(1, "Checking speaker %s is %s (%s, %s)",
         agent->name.c_str(),
         human ? " human" : " not human",
         (human && human->isVisible()) ? "visible" : "not visible",
         (human && human->hasMarkers()) ? "markers" : "no markers");

    if (human && human->isVisible() && human->hasHead(graph))
    {
      double headPosition[3];
      bool hasHead = human->getHeadPositionInWorld(headPosition, graph);
      RCHECK(hasHead);
      double humanDir[3];
      Vec3d_sub(humanDir, headPosition, micPosition);
      double ang_i = Vec3d_diffAngle(humanDir, soundDir);
      RLOG(1, "  Human direction: %.3f %.3f %.3f Angle = %.1f",
           humanDir[0], humanDir[1], humanDir[2], RCS_RAD2DEG(ang_i));
      if (ang_i<ang)
      {
        ang = ang_i;
        speaker = human;
      }
    }
  }

  return speaker;
}

const Agent* RespeakerComponent::getListener(const double micPosition[3],
                                             const HumanAgent* speaker,
                                             const RcsGraph* graph)
{
  if (!speaker || !speaker->isVisible() || !speaker->hasHead(graph))
  {
    RLOG(1, "No speaker: Can't determine listener");
    return nullptr;
  }

  // Determine the closest listener of all agents
  const Agent* lookedAt = nullptr;
  double ang = 2.0*M_PI;
  double headPosition[3], gazeDirection[3];
  bool speakerSuccess = speaker->getHeadPositionInWorld(headPosition, graph);
  if (!speakerSuccess)
  {
    RLOG_CPP(0, "Failed to obtain the head position of speaker " << speaker->name);
  }

  speakerSuccess = speaker->getGazeDirectionInWorld(gazeDirection, graph);
  if (!speakerSuccess)
  {
    RLOG_CPP(0, "Failed to obtain the gaze direction of speaker " << speaker->name);
  }

  RLOG_CPP(1, "getListener: searching for listener for speaker " << speaker->name);
  for (auto& other : scene->agents)
  {
    RLOG_CPP(1, "getListener: checking " << other->name);
    HumanAgent* otherHuman = dynamic_cast<HumanAgent*>(other);

    // We inhibit that the listener is the speaker
    if (otherHuman && otherHuman->name==speaker->name)
    {
      RLOG_CPP(1, "getListener: ignoring " << otherHuman->name << " because it is the speaker");
      continue;
    }

    double otherHead[3];
    if (otherHuman)
    {
      bool success = otherHuman->getHeadPositionInWorld(otherHead, graph);
      if (!success)
      {
        RLOG_CPP(0, "Failed to obtain the head position of agent " << otherHuman->name);
      }
    }
    else
    {
      Vec3d_copy(otherHead, micPosition);
    }

    double candidateDir[3];
    Vec3d_sub(candidateDir, otherHead, headPosition);
    double ang_i = Vec3d_diffAngle(gazeDirection, candidateDir);

    if (ang_i<ang)
    {
      lookedAt = other;
      ang = ang_i;
    }

  }

  // Only when the listener is inside the speaker's view cone, we will identify
  // her or him as the listener. Otherwise, we assume that the speaker talks
  // to all.
  if ((!lookedAt) || (ang>AGENT_DETECTION_CONE_ANGLE))
  {
    RLOG(1, "No listener found");
    return lookedAt;
  }

  //speaker->setGazeTarget(lookedAt->name);

  return lookedAt;
}


#if defined (USE_ROS)

void RespeakerComponent::talkFlagRosCallback(const audio_msgs::TalkFlag& msg)
{
  if (isASREnabled != msg.active)
  {
    isASREnabled = msg.active;
    std::string bgColor = isASREnabled ? std::string("OBSIDIAN") : std::string();
    getEntity()->publish<std::string, std::string>("RenderCommand", "BackgroundColor", bgColor);
  }

  RLOG(1, "Received isASREnabled from ROS: %s", isASREnabled ? "ON" : "OFF");
}

/*
{"confidence": 0.857319176197052, "text": "hello I Michael", "end_time": 1698945193.0811708, "frame_id": "respeaker_base", "position": {"x": -0.2756373558169989, "y": -0.961261695938319}}
 */
void RespeakerComponent::asrRosCallback(const std_msgs::String::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(textLock);
  receivedTextJson = msg->data;
}

std::string RespeakerComponent::getAndClearTextFromROS()
{
  std::lock_guard<std::mutex> lock(textLock);
  std::string spoken = receivedTextJson;
  receivedTextJson.clear();

  return spoken;
}

void RespeakerComponent::soundLocalizationRosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (!isSoundDirectionEstimationEnabled)
  {
    return;
  }

  double soundDir[3];   // Vector from mic to sound source
  soundDir[0] = msg->pose.position.x;
  soundDir[1] = msg->pose.position.y;
  soundDir[2] = 0.0 * msg->pose.position.z;
  Vec3d_normalizeSelf(soundDir);

  // Ignore sounds from behind \todo(MG): Improve hardcoded behavior
  if (soundDir[1] < -0.1)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(sndDirLock);
  soundDirectionROS = std::vector<double>(soundDir, soundDir + 3);
}

#endif   // USE_ROS


}   // namespace
