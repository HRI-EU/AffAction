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
#include "KortexComponent.hpp"
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

static ComponentBase* createLandmarkZmqComponent(EntityBase& entity,
                                                 const RcsGraph* graph,
                                                 const ActionScene* scene,
                                                 std::string extraArgs)
{
  auto argsVec = Rcs::String_split(extraArgs, " ");
  std::string connection = "tcp://localhost:5555";
  std::string landmarksCamera = "camera_0";
  getKeyValuePair<std::string>(argsVec, "-landmarks_camera", landmarksCamera);
  getKeyValuePair<std::string>(argsVec, "-landmarks_connection", connection);
  RcsBody* cam = RcsGraph_getBodyByName(graph, landmarksCamera.c_str());
  if (!cam)
  {
    RLOG_CPP(0, "Couldn't find camera body '" << landmarksCamera
             << "' for LandmarkZmqComponent");
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

  if (getKey(argsVec, "-aruco_tracking"))
  {
    std::string arucoBaseBdyName = "aruco_base";
    getKeyValuePair<std::string>(argsVec, "-aruco_base", arucoBaseBdyName);
    lmc->addArucoTracker(landmarksCamera, arucoBaseBdyName);
  }

  if (getKey(argsVec, "-skeleton_tracking"))
  {
    RLOG(0, "Enabling Azure skeleton tracker");
    double r_agent = DBL_MAX;
    getKeyValuePair<double>(argsVec, "-skeleton_radius", r_agent);

    // Add skeleton tracker and ALL agents in the scene
    int numAgents = lmc->addSkeletonTrackerForAgents(r_agent);
    RLOG(0, "Done adding skeleton tracker with %d agents", numAgents);
  }

  // Initialize all tracker camera transforms from the xml file
  lmc->setCameraTransform(&cam->A_BI);

  return lmc;
}

static ComponentBase* createPW70Component(EntityBase& entity,
                                          const RcsGraph* graph,
                                          const ActionScene* scene,
                                          const std::string& componentName,
                                          std::string extraArgs)
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
    bool success = c->init(controlFreq);
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
    bool success = c->init(controlFreq);
    if (!success)
    {
      RLOG(0, "Couldn't create PW70VelocityComponent - wrong controlFrequency: %d", controlFreq);
      delete c;
      c = nullptr;
    }
    return c;
  }

  return nullptr;
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

#if !defined (_MSC_VER)
  if (dryRun)
  {
    argP.addDescription("-jacoShm7r", "Start with Jaco7 Shm right");
    argP.addDescription("-jacoShm7l", "Start with Jaco7 Shm left");
  }
  else
  {
    if (getKey(argvStrVec, "-jacoShm7r"))
    {
      ComponentBase* c = RoboJacoShmComponent::create(&entity, graph, JacoShmComponent::Jaco7_right);
      components.push_back(c);
    }

    if (getKey(argvStrVec, "-jacoShm7l"))
    {
      ComponentBase* c = RoboJacoShmComponent::create(&entity, graph, JacoShmComponent::Jaco7_left);
      components.push_back(c);
    }
  }
#endif

  if (dryRun)
  {
    argP.addDescription("-pw70_pos", "Start with Scitos PTU in position mode");
    argP.addDescription("-pw70_pan_joint_name", "Name of PW70 pan joint (default: empty string)");
    argP.addDescription("-pw70_tilt_joint_name", "Name of PW70 pan joint (default: empty string)");
    argP.addDescription("-pw70_control_frequency", "PW70 PTU control frequency (Must be 1, 10, 25, 50 or 100. Default: 50)");
    argP.addDescription("-pw70_vel", "Start with Scitos PTU in velocity mode");
  }
  else
  {
    if (getKey(argvStrVec, "-pw70_pos"))
    {
      RCHECK_MSG(!getKey(argvStrVec, "-pw70_vel"), "Can't start PW70 component both in position and velocity mode");
      components.push_back(createPW70Component(entity, graph, scene, "-pw70_pos", argvString));
    }
    else if (getKey(argvStrVec, "-pw70_vel"))
    {
      RCHECK_MSG(!getKey(argvStrVec, "-pw70_pos"), "Can't start PW70 component both in position and velocity mode");
      components.push_back(createPW70Component(entity, graph, scene, "-pw70_vel", argvString));
    }
  }

  if (dryRun)
  {
    argP.addDescription("-kortex", "Start with Kinova Kortex component");
  }
  else if (getKey(argvStrVec, "-kortex"))
  {
    components.push_back(new aff::KortexComponent(&entity));
  }

#if defined USE_ROS
  if (dryRun)
  {
    argP.addDescription("-ptu", "Connect with Scitos PTU action server");
  }
  else if (getKey(argvStrVec, "-ptu"))
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    components.push_back(new PtuActionComponent(&entity));
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

  if (dryRun)
  {
    argP.addDescription("-tts", "Start with native text-to-speech");
    argP.addDescription("-piper_tts_alan", "Start with piper text-to-speech, Alan's voice");
    argP.addDescription("-piper_tts_joe", "Start with piper text-to-speech, Joe's voice");
    argP.addDescription("-piper_tts_kathleen", "Start with piper text-to-speech, Kathleen's voice");
    argP.addDescription("-piper", "Start with piper text-to-speech, Kathleen's voice");
  }
  else if (getKey(argvStrVec, "-tts"))
  {
    components.push_back(new TTSComponent(&entity));
  }
  else if (getKey(argvStrVec, "-piper_tts_alan"))
  {
    auto tts = new TTSComponent(&entity, "piper");
    tts->setPiperVoice("alan");
    components.push_back(tts);
  }
  else if (getKey(argvStrVec, "-piper_tts_joe"))
  {
    auto tts = new TTSComponent(&entity, "piper");
    tts->setPiperVoice("joe");
    components.push_back(tts);
  }
  else if (getKey(argvStrVec, "-piper_tts_kathleen") || getKey(argvStrVec, "-piper"))
  {
    auto tts = new TTSComponent(&entity, "piper");
    tts->setPiperVoice("kathleen");
    components.push_back(tts);
  }

  if (dryRun)
  {
    argP.hasArgument("-websocket", "Start with websocket connection on port 35000");
  }
  else if (getKey(argvStrVec, "-websocket"))
  {
    components.push_back(new WebsocketActionComponent(&entity));
  }

  // The debug graphics will be handled in initGraphics.
  if (dryRun)
  {
    argP.addDescription("-landmarks_connection", "Connection string, default is tcp://localhost:5555");
    argP.addDescription("-landmarks_zmq", "Start with ZMQ landmarks component");
    argP.addDescription("-landmarks_camera", "For '-landmarks_zmq': Body name of camera in which the landmarks are assumed to be represented. Default: camera_0");
    argP.addDescription("-face_tracking", "For '-landmarks_zmq': Start with Mediapipe face tracking");
    argP.addDescription("-face_bodyName", "For '-face_tracking' and '-face_gesture': Name of the face's RcsBody (Default: face)");
    argP.addDescription("-aruco_tracking", "For '-landmarks_zmq': Start with Aruco marker tracking");
    argP.addDescription("-aruco_base", "For '-landmarks_zmq' and '-aruco_tracking': Name of aruco base marker (default: \"aruco_base\")");
    argP.addDescription("-skeleton_tracking", "For '-landmarks_zmq': Start with skeleton tracking");
    argP.addDescription("-skeleton_radius", "For '-landmarks_zmq' and '-skeleton_tracking': Radius of skeleton detections (default: infinity)");
  }
  else if (getKey(argvStrVec, "-landmarks_zmq"))
  {
    components.push_back(createLandmarkZmqComponent(entity, graph, scene, argvString));
  }

  if (dryRun)
  {
    argP.addDescription("-camera_view", "Add camera view component");
    argP.addDescription("-camera_view_body", "For '-camera_view': Body name to which the camera will be attached. Default: face");
  }
  else if (getKey(argvStrVec, "-camera_view"))
  {
    std::string camBdyName = "face";
    getKeyValuePair<std::string>(argvStrVec, "-camera_view_body", camBdyName);
    RCHECK_MSG(RcsGraph_getBodyByName(graph, camBdyName.c_str()), "Camera attachment for parameter '-camera_view_body' %s not found", camBdyName.c_str());
    components.push_back(new CameraViewComponent(&entity, camBdyName, false));
  }

  if (dryRun)
  {
    argP.addDescription("-physics", "Start with physics simulation component");
  }
  else if (getKey(argvStrVec, "-physics"))
  {
    std::string physicsConfig;
    std::string physicsEngine = "Bullet";
    getKeyValuePair<std::string>(argvStrVec, "-physics", physicsEngine);
    getKeyValuePair<std::string>(argvStrVec, "-physics_config", physicsConfig);
    RLOG_CPP(5, "Creating physics with engine " << physicsEngine << " and config file '" << physicsConfig << "'");
    RCHECK(graph);
    components.push_back(new PhysicsComponent(&entity, graph, physicsEngine, physicsConfig));
  }

  if (dryRun)
  {
    argP.addDescription("-face_gesture", "Add face gesture component");
  }
  else if (getKey(argvStrVec, "-face_gesture"))
  {
    std::string faceBdyName = "face";
    getKeyValuePair<std::string>(argvStrVec, "-face_bodyName", faceBdyName);
    components.push_back(new aff::FaceGestureComponent(&entity, faceBdyName));
  }

#if defined USE_ROS

  if (dryRun)
  {
    argP.hasArgument("-respeaker", "Start with Respeaker");
    argP.hasArgument("-respeaker_listenWithRaisedHandOnly", "Dialogue only considered if agent has hand reised");
  }
  else if (getKey(argvStrVec, "-respeaker"))
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    RCHECK(scene);
    aff::ComponentBase* respeaker = new RespeakerComponent(&entity, scene);
    if (getKey(argvStrVec, "-respeaker_listenWithRaisedHandOnly"))
    {
      respeaker->setParameter("PublishDialogueWithRaisedHandOnly", true);
    }
    components.push_back(respeaker);
  }

  if (dryRun)
  {
    argP.hasArgument("-nuance_tts", "Start with Nuance ROS text-to-speech");
  }
  else if (getKey(argvStrVec, "-nuance_tts"))
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    components.push_back(new NuanceTTSComponent(&entity));
  }

  if (dryRun)
  {
    argP.hasArgument("-landmarks_ros", "Start with ROS landmarks component");
  }
  else if (getKey(argvStrVec, "-landmarks_ros"))
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

    components.push_back(lmc);
  }

  if (dryRun)
  {
    argP.hasArgument("-holo", "Add HoloLens component");
  }
  else if (getKey(argvStrVec, "-holo"))
  {
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    components.push_back(new HololensConnection(&entity, true));
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
    initROS(HWC_DEFAULT_ROS_SPIN_DT);
    std::string pubTopic = MIRROR_EYES_DEFAULT_PUPIL_COORDINATES_TOPIC;
    std::string gazeTopic = MIRROR_EYES_DEFAULT_GAZTARGET_TOPIC;
    std::string camTopic = MIRROR_EYES_DEFAULT_CAMERA_TOPIC;
    getKeyValuePair<std::string>(argvStrVec, "-mirror_eyes_gaze_target_topic", gazeTopic);
    getKeyValuePair<std::string>(argvStrVec, "-mirror_eyes_camera_topic", camTopic);
    getKeyValuePair<std::string>(argvStrVec, "-mirror_eyes_pupil_coords_topic", pubTopic);
    components.push_back(new MirrorEyeComponent(&entity, scene, pubTopic, gazeTopic, camTopic));
  }
#endif



  for (size_t i = 0; i < components.size(); ++i)
  {
    RCHECK_MSG(components[i], "Found NULL component at index %zu", i);
  }

  return components;
}

}   // namespace
