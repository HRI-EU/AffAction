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

void bind_components(py::class_<aff::ExampleActionsECS>& cls)
{

  //////////////////////////////////////////////////////////////////////////////
  // Components to be added. The methods must be called before init.
  //////////////////////////////////////////////////////////////////////////////
  cls.def("addWebsocket", [](aff::ExampleActionsECS& ex)
  {
    // Adds a component to connect to a websocket client. The component receives
    // action commands, and sends back the state.
    ex.addComponentArgument("-websocket");
  })
  .def("addJacoLeft", [](aff::ExampleActionsECS& ex)
  {
    // Adds a component to connect to the left Jaco7 Gen2 arm
    ex.addComponentArgument("-jacoShm7l");
  })
  .def("addJacoRight", [](aff::ExampleActionsECS& ex)
  {
    // Adds a component to connect to the right Jaco7 Gen2 arm
    ex.addComponentArgument("-jacoShm7r");
  })
  .def("addRespeaker", [](aff::ExampleActionsECS& ex, bool listenWitHandRaisedOnly)
  {
    // Adds a component to listen to the Respeaker ROS node, and to acquire the
    // sound directions, ASR etc.
    ex.addComponentArgument("-respeaker");
    if (listenWitHandRaisedOnly)
    {
      ex.addComponentArgument("-respeaker_listenWithRaisedHandOnly");
    }
  })
  .def("addRespeaker_usb", [](aff::ExampleActionsECS& ex)
  {
    // Adds a component to listen to the Respeaker direction signal directly
    // from the USB port.
    ex.addComponentArgument("-respeaker_usb");
  })
  .def("addLandmarkZmq", [](aff::ExampleActionsECS& ex,
                            const std::string& connection,
                            const std::string& camera_name,
                            bool withFaceTracking,
                            const std::string& face_name,
                            bool withArucoTracking,
                            const std::string& base_marker,
                            bool withSkeletonTracking,
                            double skeleton_radius)
  {
    ex.addComponentArgument("-landmarks_zmq");
    ex.addComponentArgument("-landmarks_connection " + connection);
    ex.addComponentArgument("-landmarks_camera " + camera_name);

    if (withFaceTracking)
    {
      ex.addComponentArgument("-face_tracking");
      ex.addComponentArgument("-face_bodyName " + face_name);
    }

    if (withArucoTracking)
    {
      ex.addComponentArgument("-aruco_tracking");
      ex.addComponentArgument("-aruco_base " + base_marker);
    }

    if (withSkeletonTracking)
    {
      ex.addComponentArgument("-skeleton_tracking");
      ex.addComponentArgument("-skeleton_radius " + std::to_string(skeleton_radius));
    }
  },
  py::arg("connection") = "tcp://localhost:5555",
  py::arg("camera_name") = "camera_0",
  py::arg("withFaceTracking") = false,
  py::arg("face_name") = "face",
  py::arg("withArucoTracking") = false,
  py::arg("base_marker") = "aruco_base",
  py::arg("withSkeletonTracking") = false,
  py::arg("skeleton_radius") = DBL_MAX
      )
  .def("addLandmarkRouter", [](aff::ExampleActionsECS& ex,
                               const std::string& connection,
                               const std::string& camera_name,
                               bool withFaceTracking,
                               const std::string& face_agent_name,
                               bool withArucoTracking,
                               const std::string& base_marker,
                               bool withSkeletonTracking,
                               double skeleton_radius)
  {
    ex.addComponentArgument("-landmarks_router");
    ex.addComponentArgument("-landmarks_connection " + connection);
    ex.addComponentArgument("-landmarks_camera " + camera_name);

    if (withFaceTracking)
    {
      ex.addComponentArgument("-face_tracking");
      ex.addComponentArgument("-face_gesture");
      //ex.addComponentArgument("-face_bodyName " + face_name);
      ex.addComponentArgument("-face_tracking.agent " + face_agent_name);
    }

    if (withArucoTracking)
    {
      ex.addComponentArgument("-aruco_tracking");
      ex.addComponentArgument("-aruco_base " + base_marker);
    }

    if (withSkeletonTracking)
    {
      ex.addComponentArgument("-skeleton_tracking");
      if (skeleton_radius != DBL_MAX)   // inhibit polluting log
      {
        ex.addComponentArgument("-skeleton_radius " + std::to_string(skeleton_radius));
      }
    }
  },
  py::arg("connection") = "tcp://*:40000",
  py::arg("camera_name") = "camera_0",
  py::arg("withFaceTracking") = false,
  py::arg("face_agent_name") = "",
  py::arg("withArucoTracking") = false,
  py::arg("base_marker") = "aruco_base",
  py::arg("withSkeletonTracking") = false,
  py::arg("skeleton_radius") = DBL_MAX
      )
  .def("addPTU", [](aff::ExampleActionsECS& ex)
  {
    // Adds a component to connect to the PTU action server ROS node, and to being
    // able to send pan / tilt commands to the PTU
    ex.addComponentArgument("-ptu");
  })
  .def("addTrackingControllerPTU", [](aff::ExampleActionsECS& ex)
  {
    // Adds a component to connect to the PW70 CAN bus, and to being able to
    // send pan / tilt commands to the PTU. Please make sure to not have any
    // other PTU process (e.g. ROS ActionServer) running.
    ex.addComponentArgument("-pw70_vel -pw70_control_frequency 50");
    ex.addComponentArgument("-pw70_pan_joint_name ptu_pan_joint");
    ex.addComponentArgument("-pw70_tilt_joint_name ptu_tilt_joint");
  })
  .def("addMirrorEyes", [](aff::ExampleActionsECS& ex, std::string gazeTargetTopic, std::string cameraTopic, std::string pupilCoordsTopic)
  {
    // Adds the ROS interface to the MirrorEye system, and enables the IK-based eye model.
    ex.addComponentArgument("-mirror_eyes");
    ex.addComponentArgument("-mirror_eyes_gaze_target_topic " + gazeTargetTopic);
    ex.addComponentArgument("-mirror_eyes_camera_topic " + cameraTopic);
    ex.addComponentArgument("-mirror_eyes_pupil_coords_topic " + pupilCoordsTopic);
    ex.eyeIkEnabled = true;
  },
  py::arg("gazeTargetTopic") = "/mirror_eyes/gaze_target",
  py::arg("cameraTopic") = "/mirror_eyes/camera",
  py::arg("pupilCoordsTopic") = "/mirror_eyes/pupil_coordinates")
  .def("addLandmarkROS", [](aff::ExampleActionsECS& ex)
  {
    // Adds a component to listen to the landmarks publishers through ROS, which
    // is for instance the Azure Kinect, and later also the Mediapipe components
    ex.addComponentArgument("-respeaker");
  })
  .def("addZmqListener", [](aff::ExampleActionsECS& ex, std::string ip_string)
  {
    // Adds a component to listen to the ZeroMQ publishers, which
    // is for instance the Webcam Tracking or ASR
    ex.addComponentArgument("-zmq_listener");
  },
  py::arg("ip_string") = "tcp://*:5556")
  .def("addTTS", [](aff::ExampleActionsECS& ex, std::string type, std::string voice)
  {
    // Adds a component to connect to enable the text-to-speech functionality.
    // Currently, 2 modes are supported: the Nuance TTS which requires the
    // corresponding ROS node to run, and a native Unix espeak TTS.
    if (type == "nuance")
    {
      ex.addComponentArgument("-nuance_tts");
    }
    else if (type == "native")
    {
      ex.addComponentArgument("-tts");
    }
    else if (type == "piper")
    {
      if (voice == "alan")
      {
        ex.addComponentArgument("-piper_tts_alan");
      }
      else if (voice == "joe")
      {
        ex.addComponentArgument("-piper_tts_joe");
      }
      else if (voice == "kathleen")
      {
        ex.addComponentArgument("-piper_tts_kathleen");
      }
      else
      {
        ex.addComponentArgument("-piper_tts_ryan");
      }

    }
  },
  py::arg("type") = "piper",
  py::arg("voice") = "kathleen")
  .def("addVirtualCamera", [](aff::ExampleActionsECS& ex, int width, int height, bool withGui)
  {
    ex.virtualCameraWidth = width;
    ex.virtualCameraHeight = height;
    ex.virtualCameraEnabled = true;
    ex.virtualCameraWindowEnabled = withGui;
  },
  py::arg("width") = 640,
  py::arg("height") = 480,
  py::arg("withGui") = false)


  ;

}
