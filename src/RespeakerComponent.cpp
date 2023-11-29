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
 * rostopic pub /talk_flag audio_msgs/TalkFlag "active: true
 * start_stamp:
 *   secs: 0
 *   nsecs: 0"
 *
 * rostopic pub /talk_flag audio_msgs/TalkFlag "active: false
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

#if defined (USE_ROS)
#include <audio_msgs/AudioDataStamped.h>
#include <audio_msgs/TalkFlag.h>
#endif

#define AGENT_DETECTION_CONE_ANGLE (30.0*M_PI/180.0)

namespace aff
{

RespeakerComponent::RespeakerComponent(EntityBase* parent, const ActionScene* scene_) :
  ComponentBase(parent), scene(scene_), respeakerBdyName("respeaker"), isASREnabled(false),
  isSoundDirectionEstimationEnabled(true), isSomebodySpeaking(false), isAnyHandRaised(false),
  publishDialogueWithRaisedHandOnly(true), gazeAtSpeaker(true), speakOutSpeakerListenerText(false)
{
  soundDirection.resize(3);
  soundDirection[0] = 0.0;
  soundDirection[1] = 1.0;
  soundDirection[2] = 0.0;
  soundDirectionFilt = soundDirection;
  RCHECK(scene);
  subscribe("Start", &RespeakerComponent::onStart);
  subscribe("Stop", &RespeakerComponent::onStop);
  subscribe("PostUpdateGraph", &RespeakerComponent::onPostUpdateGraph);
  subscribe("EnableASR", &RespeakerComponent::enableASR);
  subscribe("ToggleASR", &RespeakerComponent::toggleASR);
  subscribe("EnableSoundDirection", &RespeakerComponent::enableSoundDirectionEstimation);
  onStart();
}

RespeakerComponent::~RespeakerComponent()
{
  enableASR(false);
  enableSoundDirectionEstimation(true);
}

void RespeakerComponent::setPublishDialogueWithRaisedHandOnly(bool enable)
{
  publishDialogueWithRaisedHandOnly = enable;
}

bool RespeakerComponent::getPublishDialogueWithRaisedHandOnly() const
{
  return publishDialogueWithRaisedHandOnly;
}

void RespeakerComponent::setGazeAtSpeaker(bool enable)
{
  gazeAtSpeaker = enable;
}

bool RespeakerComponent::getGazeAtSpeaker() const
{
  return gazeAtSpeaker;
}

void RespeakerComponent::setSpeakOutSpeakerListenerText(bool enable)
{
  speakOutSpeakerListenerText = enable;
}

bool RespeakerComponent::getSpeakOutSpeakerListenerText() const
{
  return speakOutSpeakerListenerText;
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
    RLOG(0, "Talk flag is already %s", enable ? "enabled" : "disabled");
    return;
  }

  RLOG(0, "Setting talk flag to %s", enable ? "TRUE" : "FALSE");
  isASREnabled = enable;

#if defined (USE_ROS)
  audio_msgs::TalkFlag talkFlagMsg;
  talkFlagMsg.active = isASREnabled;
  talkFlagMsg.start_stamp.sec = 0;
  talkFlagMsg.start_stamp.nsec = 0;
  //talk_flag_pub.publish(talkFlagMsg);
#endif

  std::string bgColor = enable ? std::string() : std::string("OBSIDIAN");
  getEntity()->publish<std::string, std::string>("RenderCommand", "BackgroundColor", bgColor);
}

void RespeakerComponent::toggleASR()
{
  enableASR(!isASREnabled);
}

void RespeakerComponent::updateSoundDirection(RcsGraph* graph, const std::string& spoken,
                                              double micPos[3], double soundDir[3])
{
  Vec3d_set(soundDir, 0.0, 0.0, 1.0);

  // Determine respeaker body to transform sound coordinates into world frame
  const RcsBody* respeakerBdy = RcsGraph_getBodyByName(graph, respeakerBdyName.c_str());
  RCHECK_MSG(respeakerBdy, "Not found in graph: %s", respeakerBdyName.c_str());

  // Use raised hand as sound direction (if any)
  isAnyHandRaised = false;

  for (const auto& agent : scene->agents)
  {
    HumanAgent* human = dynamic_cast<HumanAgent*>(agent);
    if (human && human->isVisible)
    {
      // HANDTIP_RIGHT = 16, HANDTIP_LEFT = 9, HEAD = 26
      const double* headPos = human->markers[26].org;
      const double* lhPos = human->markers[9].org;
      const double* rhPos = human->markers[16].org;

      //RLOG(0, "*** head: %.3f   lh: %.3f   rh: %.3f", headPos[2], lhPos[2], rhPos[2]);

      const double handBelowHead = -0.2;
      if ((lhPos[2]>headPos[2]-handBelowHead) || (rhPos[2]>headPos[2]-handBelowHead))
      {
        // Transform into respeaker frame
        Vec3d_invTransform(soundDirection.data(), &respeakerBdy->A_BI, headPos);
        isAnyHandRaised = true;
        break;
      }
    }
  }


  // Parse json into text and sound direction
  if (!spoken.empty())
  {
    nlohmann::json micJson = nlohmann::json::parse(spoken);
    //std::string text = micJson["text"];
    //double soundDir[3];   // Vector from mic to sound source
    //soundDir[0] = micJson["position"]["x"];
    //soundDir[1] = micJson["position"]["y"];
    //soundDir[2] = 0.0;
  }

  for (int i=0; i<3; ++i)
  {
    soundDirectionFilt[i] = 0.05*soundDirection[i] + 0.95*soundDirectionFilt[i];
    soundDir[i] = soundDirectionFilt[i];
  }
  //  RLOG(0, "Sound direction in local coordinates: %.3f %.3f %.3f", soundDir[0], soundDir[1], soundDir[2]);

  const double* soundDirLocal = soundDirectionFilt.data();

  // Transform direction vector into world coordinates
  Vec3d_copy(micPos, respeakerBdy->A_BI.org);
  Vec3d_transRotateSelf(soundDir, (double(*)[3])respeakerBdy->A_BI.rot);
  //  RLOG(0, "Sound direction in global coordinates: %.3f %.3f %.3f", soundDir[0], soundDir[1], soundDir[2]);

  // We show a cylinder pointing into the sound direction
  RcsBody* soundBeam = RcsGraph_getBodyByName(graph, "sound_beam");
  RCHECK(soundBeam);
  int jidx = RcsBody_getJointIndex(graph, soundBeam);
  RCHECK(jidx>=0);
  HTr tmp;
  HTr_setIdentity(&tmp);
  Vec3d_copy(tmp.rot[0], soundDirLocal);   // x-axis is in horizontal plane
  Vec3d_copy(tmp.rot[2], Vec3d_ez());                  // z-axis points up
  Vec3d_crossProduct(tmp.rot[1], tmp.rot[2], tmp.rot[0]);
  HTr_to6DVector(&graph->q->ele[jidx], &tmp);
  //VecNd_printComment("Sound beam", &graph->q->ele[jidx], 6);
}

void RespeakerComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  onPostUpdateGraphSpeech(desired, current);
}

void RespeakerComponent::onPostUpdateGraphGaze(RcsGraph* desired, RcsGraph* current)
{
  RcsGraph* graph = desired;

  double micPos[3], soundDir[3];
  updateSoundDirection(graph, std::string(), micPos, soundDir);

  for (const auto& speaker_ : scene->agents)
  {
    HumanAgent* speaker = dynamic_cast<HumanAgent*>(speaker_);
    if (!speaker)
    {
      continue;
    }

    std::string listenerName = getListenerName(micPos, speaker);
    RLOG_CPP(0, "Speaker " << speaker->name << ": " << listenerName << " listens");
  }

}

void RespeakerComponent::onPostUpdateGraphSpeech(RcsGraph* desired, RcsGraph* current)
{
  // A very short mutex section to not block anything
  textLock.lock();
  std::string spoken = receivedTextJson;
  receivedTextJson.clear();
  textLock.unlock();

  RcsGraph* graph = desired;
  double micPos[3], soundDir[3];
  updateSoundDirection(graph, spoken, micPos, soundDir);

  // Early return if nothing todo
  if (spoken.empty())
  {
    return;
  }



  // Check which agent is speaking
  const HumanAgent* speaker = getSpeaker(micPos, soundDir);
  std::string speakerName = speaker ? speaker->name : "Nobody";
  std::string listenerName = getListenerName(micPos, speaker);

  /* Create output json in this format:
  (
  "text":"Hi all",
  "sender":"Bob",
  "receiver":"All"
  )
   */

  nlohmann::json micJson = nlohmann::json::parse(spoken);
  std::string text = micJson["text"];

  if ((publishDialogueWithRaisedHandOnly && isAnyHandRaised) ||
      (!publishDialogueWithRaisedHandOnly))
  {
#if defined (USE_ROS)
    nlohmann::json outJson = {};
    outJson["text"] = text;
    outJson["sender"] = speakerName;
    outJson["receiver"] = listenerName;

    std_msgs::String dialogueResult;
    dialogueResult.data = outJson.dump();
    dialogue_pub.publish(dialogueResult);
#endif

    char xplain[512];
    snprintf(xplain, 512, "%s says to %s: %s ", speakerName.c_str(), listenerName.c_str(), text.c_str());
    RLOG(0, "%s", xplain);

    if (speakOutSpeakerListenerText)
    {
      getEntity()->publish("Speak", std::string(xplain));
    }

    if (gazeAtSpeaker)
    {
      std::string actionCommand = "gaze " + speakerName;
      getEntity()->publish("ActionSequence", actionCommand);
    }
  }
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
    this->isSpeakingSubscriber = nh->subscribe("/is_speaking", 10, &RespeakerComponent::isSpeakingRosCallback, this);
    this->talkFlagSubscriber = nh->subscribe("/talk_flag", 10, &RespeakerComponent::talkFlagRosCallback, this);
    RLOG(1, "Done subscribing");

    RLOG_CPP(1, "Advertising topic /talk_flag");
    this->talk_flag_pub = nh->advertise<audio_msgs::TalkFlag>("/talk_flag", 1);
    this->dialogue_pub = nh->advertise<std_msgs::String>("/event_speech", 10);
    RLOG_CPP(1, "Done RespeakerComponent::onStart()");
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
const HumanAgent* RespeakerComponent::getSpeaker(const double micPosition[3],
                                                 const double soundDir[3]) const
{
  const HumanAgent* speaker = nullptr;
  double ang = 2.0*M_PI;
  RLOG(1, "Sound direction: %.3f %.3f %.3f", soundDir[0], soundDir[1], soundDir[2]);

  for (const auto& agent : scene->agents)
  {
    HumanAgent* human = dynamic_cast<HumanAgent*>(agent);
    RLOG(1, "Checking speaker %s is %s (%s, %s)",
         agent->name.c_str(),
         human ? " human" : " not human",
         (human && human->isVisible) ? "visible" : "not visible",
         (human && !human->markers.empty()) ? "markers" : "no markers");

    if (human && human->isVisible && human->hasHead())
    {
      double headPosition[3];
      bool hasHead = human->getHeadPosition(headPosition);
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

std::string RespeakerComponent::getListenerName(const double micPosition[3],
                                                const HumanAgent* speaker_)
{
  HumanAgent* speaker = (HumanAgent*) speaker_;   // \todo(MG): Move to somewhere where ActionScene is not const

  if (!speaker || !speaker->isVisible || !speaker->hasHead())
  {
    RLOG(1, "No speaker: all are listeners");
    return "All";
  }

  // Determine the closest listener of all agents
  double ang = 2.0*M_PI;
  const Agent* lookedAt = NULL;

  for (auto& other : scene->agents)
  {
    HumanAgent* otherHuman = dynamic_cast<HumanAgent*>(other);

    // We inhibit that the listener is the speaker
    if (otherHuman && otherHuman->name==speaker->name)
    {
      continue;
    }

    double candidateDir[3], otherHead[3];
    if (otherHuman)
    {
      otherHuman->getHeadPosition(otherHead);
    }
    else
    {
      Vec3d_copy(otherHead, micPosition);
    }

    Vec3d_sub(candidateDir, otherHead, speaker->headPosition);
    double ang_i = Vec3d_diffAngle(speaker->gazeDirection, candidateDir);

    if (ang_i<ang)
    {
      lookedAt = other;
      ang = ang_i;
    }

  }

  // Only when the listener is inside the speaker's view cone, we will identify
  // her or him as the listener. Otherwise, we assume that the speaker talks
  // to all.
  RLOG_CPP(1, "Ang: " << RCS_RAD2DEG(ang) << " " << lookedAt->name);

  if ((!lookedAt) || (ang>AGENT_DETECTION_CONE_ANGLE))
  {
    RLOG(1, "No listener found: assuming it is talking to all");
    return "All";
  }

  speaker->gazeTargetPrev = speaker->gazeTarget;
  speaker->gazeTarget = lookedAt->name;

  // if (speaker->gazeTargetPrev != speaker->gazeTarget)
  // {
  //   std::string msg = speaker->name + " is looking at " + speaker->gazeTarget;
  //   getEntity()->publish("Speak", msg);
  // }

  return lookedAt->name;
}





#if defined (USE_ROS)

void RespeakerComponent::isSpeakingRosCallback(const std_msgs::Bool::ConstPtr& msg)
{
  isSomebodySpeaking = msg->data;
  RLOG(1, "isSpeakingRosCallback: %s", isSomebodySpeaking ? "SPEAKING" : "NOT SPEAKING");
}

void RespeakerComponent::talkFlagRosCallback(const audio_msgs::TalkFlag::ConstPtr& msg)
{
  enableASR(msg->active);
  RLOG(0, "talk_flag subscriber: %s", isASREnabled ? "ENABLED" : "DISABLED");
}

/*
{"confidence": 0.857319176197052, "text": "hello I Michael", "end_time": 1698945193.0811708, "frame_id": "respeaker_base", "position": {"x": -0.2756373558169989, "y": -0.961261695938319}}
 */
void RespeakerComponent::asrRosCallback(const std_msgs::String::ConstPtr& msg)
{

  // We reject any ASR result if no hand is lifted. This means that the
  // complete speech acquisition must be performed with a raised hand.
  if ((!isAnyHandRaised) && publishDialogueWithRaisedHandOnly)
  {
    return;
  }

  //if (isASREnabled)
  {
    std::lock_guard<std::mutex> lock(textLock);
    receivedTextJson = msg->data;
  }
  // else
  // {
  //   RLOG_CPP(0, "Ignoring asr data: " << msg->data);
  // }
}

void RespeakerComponent::soundLocalizationRosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if ((!isSoundDirectionEstimationEnabled) || (!isSomebodySpeaking))
  {
    return;
  }

  double soundDir[3];   // Vector from mic to sound source
  soundDir[0] = msg->pose.position.x;
  soundDir[1] = msg->pose.position.y;
  soundDir[2] = 0.0 * msg->pose.position.z;
  Vec3d_normalizeSelf(soundDir);

  //Ignore sounds from behind
  if (soundDir[1] < -0.1)
  {
    return;
  }

  soundDirection = std::vector<double>(soundDir, soundDir + 3);
}

#endif   // USE_ROS


}   // namespace
