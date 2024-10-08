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

#ifndef AFF_MIRROREYECOMPONENT_H
#define AFF_MIRROREYECOMPONENT_H

#include "ComponentBase.h"
#include "ActionScene.h"

#include <Rcs_graph.h>

#if defined (USE_ROS)
#include <ros/ros.h>
#include <std_msgs/String.h>
#endif

/*

  Mirror Eyes ROS interface:

  - Publisher: on topic /mirror_eyes/pupil_coordinates, a std_msgs::String (json) with this
               content is published in approx. 25 Hz:
               {
                "left_eye":  {'screen_coordinates': [0.02, 0.04, 0.09], 'gaze_distance': 0.9},
                "right_eye": {'screen_coordinates': [0.02, 0.04, 0.09], 'gaze_distance': 0.9},
                "screen_distance": 1.0,
                "bounding_box": {'vertex': [[0.2, 0.4, 0.9], [0.2, 0.4, 0.9], [0.2, 0.4, 0.9],
                                           [0.2, 0.4, 0.9],[0.2, 0.4, 0.9], [0.2, 0.4, 0.9],
                                           [0.2, 0.4, 0.9], [0.2, 0.4, 0.9]],
                                'x': 0.2, 'y': 0.4, 'z': 0.9}
               }

               where the screen coordinates have this convention: x points to the left of the
               screen, y points down, z points normal to the screen (should be 0). The
               origin is int he screen's center. The units are in meters. The bounding_box
               json refers to the object and camera that come through the subscriber. If no
               valid object and camera are received, this field will not exist in the json.

  - Subscriber: on topic /mirror_eyes/gaze_target, this message will be received:
                - std_msgs::String object: The object that is currently gazed at
                - std_msgs::String camera: The camera that the object is looked at
                If both object and camera are valid (can be retrieved in the graph), then
                the "bounding_box" string of the publisher will be filled according to the
                documentation in SceneJsonHelpers::getObjectInCamera.

*/

namespace aff
{
constexpr auto MIRROR_EYES_DEFAULT_PUPIL_COORDINATES_TOPIC = "/mirror_eyes/pupil_coordinates";
constexpr auto MIRROR_EYES_DEFAULT_GAZTARGET_TOPIC         = "/mirror_eyes/gaze_target";
constexpr auto MIRROR_EYES_DEFAULT_CAMERA_TOPIC            = "/mirror_eyes/camera";

class MirrorEyeComponent : public ComponentBase
{
public:

  MirrorEyeComponent(EntityBase* parent, const ActionScene* scene,
                     std::string publisherTopic=MIRROR_EYES_DEFAULT_PUPIL_COORDINATES_TOPIC,
                     std::string gazeTopic=MIRROR_EYES_DEFAULT_GAZTARGET_TOPIC,
                     std::string cameraTopic=MIRROR_EYES_DEFAULT_CAMERA_TOPIC);
  virtual ~MirrorEyeComponent();

private:

  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);
  void onStart();
  void onStop();

  const ActionScene* scenePtr;
  size_t loopCount;
  std::string currentGazeTarget, currentCamera;
  std::string pupilCoordsPublisherTopic, gazeTargetSubscriberTopic, cameraSubscriberTopic;
  std::mutex rosLock;

#if defined (USE_ROS)
  void gazeTargetNameRosCallback(const std_msgs::String::ConstPtr& gazeTarget);
  void cameraNameRosCallback(const std_msgs::String::ConstPtr& camera);
  ros::Publisher gaze_pub;
  ros::Subscriber gaze_target_sub;
  ros::Subscriber camera_sub;
  std::unique_ptr<ros::NodeHandle> nh;
#endif

};

}   // namespace

#endif   // AFF_MIRROREYECOMPONENT_H
