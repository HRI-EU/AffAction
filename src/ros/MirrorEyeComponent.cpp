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

#include "MirrorEyeComponent.h"
#include "SceneJsonHelpers.h"
#include "ActionEyeGaze.h"
#include "json.hpp"

#include <Rcs_typedef.h>
#include <Rcs_timer.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>



namespace aff
{

MirrorEyeComponent::MirrorEyeComponent(EntityBase* parent, const ActionScene* scene,
                                       std::string pubTopic,
                                       std::string gazeAtTopic,
                                       std::string camTopic) :
  ComponentBase(parent), scenePtr(scene), loopCount(0),
  pupilCoordsPublisherTopic(pubTopic), gazeTargetSubscriberTopic(gazeAtTopic), cameraSubscriberTopic(camTopic)
{
  subscribe("Start", &MirrorEyeComponent::onStart);
  subscribe("Stop", &MirrorEyeComponent::onStop);
  subscribe("PostUpdateGraph", &MirrorEyeComponent::onPostUpdateGraph);
  onStart();
}

MirrorEyeComponent::~MirrorEyeComponent()
{
}

void MirrorEyeComponent::onStart()
{
#if defined (USE_ROS)

  RLOG_CPP(1, "MirrorEyeComponent::onStart()");
  if (!nh)
  {
    RLOG(1, "Creating node handle");
    nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

    RLOG_CPP(1, "Subscribing to gaze target " << gazeTargetSubscriberTopic
             << " and camera on topic " << cameraSubscriberTopic);
    this->gaze_target_sub = nh->subscribe(gazeTargetSubscriberTopic, 10, &MirrorEyeComponent::gazeTargetNameRosCallback, this);
    this->camera_sub = nh->subscribe(cameraSubscriberTopic, 10, &MirrorEyeComponent::cameraNameRosCallback, this);

    RLOG_CPP(1, "Advertising pupil coordinates on topic " << pupilCoordsPublisherTopic);
    this->gaze_pub = nh->advertise<std_msgs::String>(pupilCoordsPublisherTopic, 10);
  }

  RLOG_CPP(1, "Done MirrorEyeComponent::onStart()");
#endif
}

void MirrorEyeComponent::onStop()
{
#if defined (USE_ROS)

  RLOG(0, "MirrorEyeComponent::onStop()");
  if (nh)
  {
    this->gaze_target_sub.shutdown();
  }

#endif
}

#if defined (USE_ROS)
void MirrorEyeComponent::gazeTargetNameRosCallback(const std_msgs::String::ConstPtr& object)
{
  std::lock_guard<std::mutex> lock(rosLock);
  currentGazeTarget = object->data;
}

void MirrorEyeComponent::cameraNameRosCallback(const std_msgs::String::ConstPtr& camera)
{
  std::lock_guard<std::mutex> lock(rosLock);
  currentCamera = camera->data;
}
#endif

void MirrorEyeComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  // 25Hz
  if ((++loopCount) % 4 != 0)
  {
    return;
  }

  RcsGraph* graph = desired;

  double p_right[3], p_left[3];

  bool success = ActionEyeGaze::computePupilCoordinates(graph, p_right, p_left);

  const RcsBody* screen = RcsGraph_getBodyByName(graph, ActionEyeGaze::getScreenName().c_str());
  const RcsBody* rightPupil = RcsGraph_getBodyByName(graph, ActionEyeGaze::getRightPupilName().c_str());
  const RcsBody* leftPupil = RcsGraph_getBodyByName(graph, ActionEyeGaze::getLeftPupilName().c_str());
  const RcsBody* rightGazePoint = RcsGraph_getBodyByName(graph, ActionEyeGaze::getRightGazePointName().c_str());
  const RcsBody* leftGazePoint = RcsGraph_getBodyByName(graph, ActionEyeGaze::getLeftGazePointName().c_str());
  const RcsBody* gazePoint = RcsGraph_getBodyByName(graph, ActionEyeGaze::getGazePointName().c_str());

  if ((!screen) || (!rightPupil) || (!leftPupil) || (!rightGazePoint) || (!leftGazePoint) || (!gazePoint))
  {
    success = false;
  }

  if (!success)
  {
    RLOG(1, "Couldn't compute gaze screen coordinates");
    return;
  }


  nlohmann::json eyeJsonL;
  eyeJsonL["screen_coordinates"] = std::vector<double>(p_left, p_left+3);
  eyeJsonL["gaze_distance"] = Vec3d_distance(leftPupil->A_BI.org, leftGazePoint->A_BI.org);

  nlohmann::json eyeJsonR;
  eyeJsonR["screen_coordinates"] = std::vector<double>(p_right, p_right+3);
  eyeJsonR["gaze_distance"] = Vec3d_distance(rightPupil->A_BI.org, rightGazePoint->A_BI.org);

  nlohmann::json gazeJson;
  gazeJson["left_eye"] = eyeJsonL;
  gazeJson["right_eye"] = eyeJsonR;
  gazeJson["screen_distance"] = Vec3d_distance(screen->A_BI.org, gazePoint->A_BI.org);

  // Coordinates of bounding box in camera coordinates
  std::string gazedAtObject, gazingCam;
  {
    std::lock_guard<std::mutex> lock(rosLock);
    gazedAtObject = ActionEyeGaze::resolveGazeTargetBodyName(*scenePtr, desired, this->currentGazeTarget);
    //gazedAtObject = this->currentGazeTarget;
    gazingCam = this->currentCamera;
  }

  if (scenePtr && (!gazedAtObject.empty()) && (!gazingCam.empty()))
  {
    nlohmann::json bb = getObjectInCamera(gazedAtObject, gazingCam, scenePtr, graph);
    gazeJson["bounding_box"] = bb;
  }

  std::string gazeString = gazeJson.dump();
  RLOG_CPP(0, "Gaze JSON: '" << gazeString << "'");

#if defined (USE_ROS)
  std_msgs::String gazeMsg;
  gazeMsg.data = gazeString;
  gaze_pub.publish(gazeMsg);
#endif
}

}   // namespace
