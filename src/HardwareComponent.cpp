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

#include "HardwareComponent.h"
#include "JacoShmComponent.h"
#include "TTSComponent.h"
#include "PhysicsComponent.h"
#include "WebsocketActionComponent.h"
#include "LandmarkZmqComponent.h"
#include "CameraViewComponent.h"
#include "FaceGestureComponent.h"
#include "PW70Component.h"
#include "FaceTracker.h"
#include "StringParserTools.hpp"

#if defined USE_ROS
#include "ros/PtuActionComponent.h"
#include "ros/RespeakerComponent.h"
#include "ros/NuanceTTSComponent.h"
#include "ros/LandmarkROSComponent.hpp"
#include "ros/HololensConnection.hpp"
#include "ros/MirrorEyeComponent.h"
#endif

#include <Rcs_typedef.h>
#include <Rcs_cmdLine.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_macros.h>

#include <thread>


#define HWC_DEFAULT_ROS_SPIN_DT (0.02)  // 20 msec = 50Hz



namespace aff
{

static void initROS(double rosDt)
{
#if defined USE_ROS
  static std::mutex rosInitMtx;
  static bool rosInitialized = false;

  std::lock_guard<std::mutex> lock(rosInitMtx);

  if (rosInitialized)
  {
    RLOG(1, "ROS already initialized - doing nothing");
    return;
  }

  rosInitialized = true;

  Rcs::CmdLineParser argP;
  int argc = 0;
  char** argv = NULL;

  argc = argP.getArgs(&argv);

  if (argc==0)
  {
    RCHECK(argv == NULL);
  }

  RMSG("Calling ros::init()");
  ros::init(argc, argv, "RcsROS", ros::init_options::NoSigintHandler);

  std::thread t1([rosDt]
  {
    while (ros::ok())
    {
      //RLOG(1, "ros::spinOnce(): %.3f sec", rosDt);
      ros::spinOnce();
      Timer_waitDT(rosDt);
    }

    RLOG(0, "ROS says good bye");
  });

  t1.detach();
#else
  RMSG("You are trying to initialize ROS, but it has not been compiled in");
#endif
}

std::vector<ComponentBase*> createHardwareComponents(EntityBase& entity,
                                                     const RcsGraph* graph,
                                                     const ActionScene* scene,
                                                     bool dryRun,
                                                     std::string extraArgs)
{
  Rcs::CmdLineParser argP;
  std::vector<ComponentBase*> components;

  auto argvStrVec = argP.copyArgvToVector();
  auto extraArgsVec = Rcs::String_split(extraArgs, " ");
  argvStrVec.insert(argvStrVec.end(), extraArgsVec.begin(), extraArgsVec.end());
  auto argvString = Rcs::String_concatenate(argvStrVec, " ");

  if (argP.hasArgument("-jacoShm7r", "Start with Jaco7 Shm right") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-jacoShm7r"));
  }

  if (argP.hasArgument("-jacoShm7l", "Start with Jaco7 Shm left") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-jacoShm7l"));
  }

  if (dryRun)
  {
    argP.addDescription("-pw70_pos", "Start with Scitos PTU in position mode");
    argP.addDescription("-pw70_pan_joint_name", "Name of PW70 pan joint (default: empty string)");
    argP.addDescription("-pw70_tilt_joint_name", "Name of PW70 pan joint (default: empty string)");
    argP.addDescription("-pw70_control_frequency", "PW70 PTU control frequency (Must be 1, 10, 25, 50 or 100. Default: 50)");
  }
  else if (getKey(argvStrVec, "-pw70_pos"))
  {
    components.push_back(createComponent(entity, graph, scene, "-pw70_pos", argvString));
  }

  if (dryRun)
  {
    argP.addDescription("-pw70_vel", "Start with Scitos PTU in velocity mode");
  }
  else if (getKey(argvStrVec, "-pw70_vel"))
  {
    components.push_back(createComponent(entity, graph, scene, "-pw70_vel", argvString));
  }

#if defined USE_ROS
  if (argP.hasArgument("-ptu", "Start with Scitos PTU") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-ptu"));
  }
#endif

  for (size_t i=0; i< components.size(); ++i)
  {
    RCHECK_MSG(components[i], "Found NULL hardware component at index %zu", i);
  }

  return components;
}

std::vector<ComponentBase*> createComponents(EntityBase& entity,
                                             const RcsGraph* graph,
                                             const ActionScene* scene,
                                             bool dryRun,
                                             std::string extraArgs)
{
  Rcs::CmdLineParser argP;
  std::vector<ComponentBase*> components;

  auto argvStrVec = argP.copyArgvToVector();
  auto extraArgsVec = Rcs::String_split(extraArgs, " ");
  argvStrVec.insert(argvStrVec.end(), extraArgsVec.begin(), extraArgsVec.end());
  auto argvString = Rcs::String_concatenate(argvStrVec, " ");


  if (argP.hasArgument("-tts", "Start with native text-to-speech") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-tts"));
  }

  if (argP.hasArgument("-piper_tts_alan", "Start with piper text-to-speech, Alan's voice") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-piper_tts_alan"));
  }

  if (argP.hasArgument("-piper_tts_joe", "Start with piper text-to-speech, Joe's voice") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-piper_tts_joe"));
  }

  if (argP.hasArgument("-piper_tts_kathleen", "Start with piper text-to-speech, Kathleen's voice") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-piper_tts_kathleen"));
  }

  if (argP.hasArgument("-piper", "Start with piper text-to-speech, Kathleen's voice") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-piper_tts_kathleen"));
  }

  if (dryRun)
  {
    argP.hasArgument("-websocket", "Start with websocket connection on port 35000");
  }
  else if (getKey(argvStrVec, "-websocket"))
  {
    components.push_back(createComponent(entity, graph, scene, "-websocket"));
  }

  // The debug graphics will be handled in initGraphics.
  if (dryRun)
  {
    argP.addDescription("-landmarks_connection", "Connection string, default is tcp://localhost:5555");
    argP.addDescription("-landmarks_zmq", "Start with ZMQ landmarks component");
    argP.addDescription("-landmarks_camera", "For '-landmarks_zmq': Body name of camera in which the landmarks are assumed to be represented. Default: camera_0");
    argP.addDescription("-face_tracking", "For '-landmarks_zmq': Start with Mediapipe face tracking");
    argP.addDescription("-face_bodyName", "For '-face_tracking' and '-face_gesture': Name of the face's RcsBody (Default: face)");
  }
  else if (getKey(argvStrVec, "-landmarks_zmq"))
  {
    components.push_back(createComponent(entity, graph, scene, "-landmarks_zmq", argvString));
  }

  if (dryRun)
  {
    argP.addDescription("-camera_view", "Add camera view component");
    argP.addDescription("-camera_view_body", "For '-camera_view': Body name to which the camera will be attached. Default: face");
  }
  else if (getKey(argvStrVec, "-camera_view"))
  {
    components.push_back(createComponent(entity, graph, scene, "-camera_view", argvString));
  }

  if (dryRun)
  {
    argP.addDescription("-physics", "Start with physics simulation component");
  }
  else if (getKey(argvStrVec, "-physics"))
  {
    components.push_back(createComponent(entity, graph, scene, "-physics", argvString));
  }

  if (argP.hasArgument("-face_gesture", "Add face gesture component") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-face_gesture", argvString));
  }

#if defined USE_ROS

  if (dryRun)
  {
    argP.hasArgument("-respeaker", "Start with Respeaker");
  }
  else if (getKey(argvStrVec, "-respeaker"))
  {
    components.push_back(createComponent(entity, graph, scene, "-respeaker"));
  }

  if (dryRun)
  {
    argP.hasArgument("-nuance_tts", "Start with Nuance ROS text-to-speech");
  }
  else if (getKey(argvStrVec, "-nuance_tts"))
  {
    components.push_back(createComponent(entity, graph, scene, "-nuance_tts"));
  }

  if (dryRun)
  {
    argP.hasArgument("-landmarks_ros", "Start with ROS landmarks component");
  }
  else if (getKey(argvStrVec, "-landmarks_ros"))
  {
    components.push_back(createComponent(entity, graph, scene, "-landmarks_ros"));
  }

  if (dryRun)
  {
    argP.hasArgument("-holo", "Add HoloLens component");
  }
  else if (getKey(argvStrVec, "-holo"))
  {
    components.push_back(createComponent(entity, graph, scene, "-holo", argvString));
  }

  if (dryRun)
  {
    argP.addDescription("-mirror_eyes", "Start with Mirror Eyes component");
    argP.addDescription("-mirror_eyes_gaze_target_topic",
                        "Name of the ROS subscriber topic for the gaze target name (default: %s)",
                        aff::MIRROR_EYES_DEFAULT_GAZTARGET_TOPIC);
    argP.addDescription("-mirror_eyes_camera_topic",
                        "Name of the ROS subscriber topic for the camera name (default: %s)",
                        aff::MIRROR_EYES_DEFAULT_CAMERA_TOPIC);
    argP.addDescription("-mirror_eyes_pupil_coords_topic",
                        "Name of the ROS publisher topic (default: %s)",
                        aff::MIRROR_EYES_DEFAULT_PUPIL_COORDINATES_TOPIC);
  }
  else if (getKey(argvStrVec, "-mirror_eyes"))
  {
    components.push_back(createComponent(entity, graph, scene, "-mirror_eyes", argvString));
  }
#endif



  for (size_t i = 0; i < components.size(); ++i)
  {
    RCHECK_MSG(components[i], "Found NULL component at index %zu", i);
  }

  return components;
}

ComponentBase* createComponent(EntityBase& entity,
                               const RcsGraph* graph,
                               const ActionScene* scene,
                               const std::string& componentName,
                               std::string extraArgs)
{
  if (componentName == "-tts")
  {
    return new TTSComponent(&entity);
  }
  else if (componentName == "-piper_tts_alan")
  {
    auto tts = new TTSComponent(&entity, "piper");
    tts->setPiperVoice("alan");
    return tts;
  }
  else if (componentName == "-piper_tts_joe")
  {
    auto tts = new TTSComponent(&entity, "piper");
    tts->setPiperVoice("joe");
    return tts;
  }
  else if (componentName == "-piper_tts_kathleen")
  {
    auto tts = new TTSComponent(&entity, "piper");
    tts->setPiperVoice("kathleen");
    return tts;
  }
  else if (componentName == "-websocket")
  {
    return new WebsocketActionComponent(&entity);
  }
  else if (componentName == "-landmarks_zmq")
  {
    auto argsVec = Rcs::String_split(extraArgs, " ");
    std::string connection = "tcp://localhost:5555";
    std::string landmarksCamera = "camera_0";
    getKeyValuePair<std::string>(argsVec, "-landmarks_camera", landmarksCamera);
    getKeyValuePair<std::string>(argsVec, "-landmarks_connection", connection);
    RcsBody* cam = RcsGraph_getBodyByName(graph, landmarksCamera.c_str());
    if (!cam)
    {
      RLOG_CPP(0, "Couldn't find camera body '" << landmarksCamera << "' for LandmarkZmqComponent");
      return nullptr;
    }

    RLOG_CPP(0, "Creating LandmarkZmqComponent with camera " << landmarksCamera);
    LandmarkZmqComponent* lmc = new LandmarkZmqComponent(&entity, connection);
    lmc->setScenePtr((RcsGraph*)graph, (ActionScene*)scene);

    if (getKey(argsVec, "-face_tracking"))
    {
      std::string faceBdyName = "face";
      getKeyValuePair<std::string>(argsVec, "-face_bodyName", faceBdyName);
      auto ft = lmc->addFaceTracker(faceBdyName, landmarksCamera);
    }

    // Initialize all tracker camera transforms from the xml file
    lmc->setCameraTransform(&cam->A_BI);

    return lmc;
  }
  else if (componentName == "-camera_view")
  {
    auto argsVec = Rcs::String_split(extraArgs, " ");
    std::string camBdyName = "face";
    getKeyValuePair<std::string>(argsVec, "-camera_view_body", camBdyName);
    RCHECK_MSG(RcsGraph_getBodyByName(graph, camBdyName.c_str()), "Camera attachment for parameter '-camera_view_body' %s not found", camBdyName.c_str());
    return new CameraViewComponent(&entity, camBdyName, false);
  }
  else if (componentName == "-face_gesture")
  {
    auto argsVec = Rcs::String_split(extraArgs, " ");
    std::string faceBdyName = "face";
    getKeyValuePair<std::string>(argsVec, "-face_bodyName", faceBdyName);
    return new aff::FaceGestureComponent(&entity, faceBdyName);
  }
  else if (componentName == "-physics")
  {
    auto argsVec = Rcs::String_split(extraArgs, " ");
    std::string physicsConfig;
    std::string physicsEngine = "Bullet";
    getKeyValuePair<std::string>(argsVec, "-physics", physicsEngine);
    getKeyValuePair<std::string>(argsVec, "-physics_config", physicsConfig);
    RLOG_CPP(5, "Creating physics with engine " << physicsEngine << " and config file '" << physicsConfig << "'");
    RCHECK(graph);
    return new PhysicsComponent(&entity, graph, physicsEngine, physicsConfig);
  }
  else if (componentName.substr(0, 6) == "-pw70_")
  {
    auto argsVec = Rcs::String_split(extraArgs, " ");
    std::string panJointName, tiltJointName;
    int controlFreq = 50;
    getKeyValuePair(argsVec, "-pw70_pan_joint_name", panJointName);
    getKeyValuePair(argsVec, "-pw70_tilt_joint_name", tiltJointName);
    getKeyValuePair(argsVec, "-pw70_control_frequency", controlFreq);
    const RcsJoint* panJnt = RcsGraph_getJointByName(graph, panJointName.c_str());
    const RcsJoint* tiltJnt = RcsGraph_getJointByName(graph, tiltJointName.c_str());
    const int panIdx = panJnt ? panJnt->jointIndex : -1;
    const int tiltIdx = tiltJnt ? tiltJnt->jointIndex : -1;
    RLOG_CPP(0, "Pan joint: " << panJointName << " index=" << panIdx);

    if (componentName == "-pw70_pos")
    {
      auto c = new PW70Component(&entity, panIdx, tiltIdx);
      bool success = c->setControlFrequency(controlFreq);
      if (!success)
      {
        RLOG(0, "Couldn't create PW70Component - wrong controlFrequency: %d", controlFreq);
        delete c;
        c = nullptr;
      }
      return c;
    }
    else if (componentName == "-pw70_vel")
    {
      auto c = new PW70VelocityComponent(&entity, panIdx, tiltIdx);
      bool success = c->setControlFrequency(controlFreq);
      if (!success)
      {
        RLOG(0, "Couldn't create PW70VelocityComponent - wrong controlFrequency: %d", controlFreq);
        delete c;
        c = nullptr;
      }
      return c;
    }
  }

#if defined USE_ROS
  else if (componentName == "-ptu")
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    return new PtuActionComponent(&entity);
  }
  else if (componentName == "-respeaker")
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    RCHECK(scene);
    return new RespeakerComponent(&entity, scene);
  }
  else if (componentName == "-nuance_tts")
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    return new NuanceTTSComponent(&entity);
  }
  else if (componentName == "-landmarks_ros")
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    LandmarkROSComponent* lmc = new LandmarkROSComponent(&entity);
    lmc->setScenePtr((RcsGraph*)graph, (ActionScene*)scene);

    const RcsBody* cam = RcsGraph_getBodyByName(graph, "camera");
    RCHECK(cam);
    lmc->addArucoTracker(cam->name, "aruco_base");

    // Add skeleton tracker and ALL agents in the scene
    double r_agent = DBL_MAX;
    int nSkeletons = lmc->addSkeletonTrackerForAgents(r_agent);
    RLOG(0, "Added skeleton tracker with %d agents", nSkeletons);

    // Initialize all tracker camera transforms from the xml file
    lmc->setCameraTransform(&cam->A_BI);

    return lmc;
  }
  else if (componentName == "-holo")
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    return new HololensConnection(&entity, true);
  }
  else if (componentName == "-mirror_eyes")
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    auto argsVec = Rcs::String_split(extraArgs, " ");
    std::string pubTopic = MIRROR_EYES_DEFAULT_PUPIL_COORDINATES_TOPIC;
    std::string gazeTopic = MIRROR_EYES_DEFAULT_GAZTARGET_TOPIC;
    std::string camTopic = MIRROR_EYES_DEFAULT_CAMERA_TOPIC;
    getKeyValuePair<std::string>(argsVec, "-mirror_eyes_gaze_target_topic", gazeTopic);
    getKeyValuePair<std::string>(argsVec, "-mirror_eyes_camera_topic", camTopic);
    getKeyValuePair<std::string>(argsVec, "-mirror_eyes_pupil_coords_topic", pubTopic);
    return new MirrorEyeComponent(&entity, scene, gazeTopic, camTopic);
  }
#endif   // USE_ROS

#if !defined (_MSC_VER)
  else if (componentName == "-jacoShm7r")
  {
    RCHECK(graph);
    return RoboJacoShmComponent::create(&entity, graph, JacoShmComponent::Jaco7_right);
  }
  else if (componentName == "-jacoShm7l")
  {
    RCHECK(graph);
    return RoboJacoShmComponent::create(&entity, graph, JacoShmComponent::Jaco7_left);
  }
#endif   // _MSC_VER

  return nullptr;
}

}   // namespace
