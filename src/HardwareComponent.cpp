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
#include "JacoZmqComponent.hpp"
#include "TTSComponent.h"
#include "PhysicsComponent.h"
#include "WebsocketActionComponent.h"
#include "LandmarkZmqComponent.h"
#include "ZmqRouterComponent.h"
#include "CameraViewComponent.h"
#include "FaceGestureComponent.h"
#include "PW70Component.h"
#include "PW70ZmqComponent.hpp"
#include "CubemarsZmqComponent.hpp"
#include "FaceTracker.h"
#include "KortexComponent.hpp"
#include "FrankaComponent.hpp"
#include "AllegroComponent.hpp"
#include "ZmqJsonSubscriber.hpp"
#include "StringParserTools.hpp"
#include "RespeakerSoundDirComponent.h"
#include "AzureSkeletonTracker.h"
#include "AgentWelcomeComponent.hpp"
#include "ImageTracker.h"
#include "EyeModelIKComponent.h"

//#define WITH_WEBSOCKETCLIENTCOMPONENT
#if defined WITH_WEBSOCKETCLIENTCOMPONENT
#include "WebsocketClientComponent.hpp"
#endif


#if defined USE_ROS
#include "ros/PtuActionComponent.h"
#include "ros/RespeakerComponent.h"
#include "ros/NuanceTTSComponent.h"
#include "ros/LandmarkROSComponent.hpp"
#include "ros/HololensConnection.hpp"
#include "ros/MirrorEyeComponent.h"
#define HWC_DEFAULT_ROS_SPIN_DT (0.02)  // 20 msec = 50Hz
#endif

#include <Rcs_typedef.h>
#include <Rcs_cmdLine.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_macros.h>
#include <Rcs_shape.h>

#include <thread>
#include <random>




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

enum class LandmarkParentClass
{
  LandmarkZmqComponent,
  LandmarkROSComponent,
  ZmqRouterComponent
};

static ComponentBase* createLandmarkComponent(EntityBase& entity,
                                              const RcsGraph* graph,
                                              const ActionScene* scene,
                                              std::string extraArgs,
                                              LandmarkParentClass parentClass,
                                              const std::string& suffix="")
{
  auto argsVec = Rcs::String_split(extraArgs, " ");
  std::string connection = "tcp://localhost:5555";
  std::string landmarksCamera = "camera_0";
  getKeyValuePair<std::string>(argsVec, "-landmarks_camera"+suffix, landmarksCamera);
  getKeyValuePair<std::string>(argsVec, "-landmarks_connection"+suffix, connection);
  RcsBody* cam = RcsGraph_getBodyByName(graph, landmarksCamera.c_str());
  if (!cam)
  {
    RLOG_CPP(0, "Couldn't find camera body '" << landmarksCamera
             << "' for LandmarkZmqComponent");
    return nullptr;
  }


  ComponentBase* ret = nullptr;
  {
    LandmarkBase* lmc = nullptr;

    if (parentClass==LandmarkParentClass::LandmarkZmqComponent)
    {
      RLOG_CPP(0, "Creating LandmarkZmqComponent with camera " << landmarksCamera);
      LandmarkZmqComponent* lmcz = new LandmarkZmqComponent(&entity, connection);
      lmc = lmcz;
      ret = lmcz;
    }
    else if (parentClass==LandmarkParentClass::ZmqRouterComponent)
    {
      RLOG_CPP(5, "Creating ZmqRouterComponent with camera "
               << landmarksCamera << " and connection " << connection);
      RLOG_CPP(5, "Extra-args: " << extraArgs);
      ZmqRouterComponent* lmcz = new ZmqRouterComponent(&entity, connection);
      lmc = lmcz;
      ret = lmcz;
    }
#if defined USE_ROS
    else if (parentClass==LandmarkParentClass::LandmarkROSComponent)
    {
      RLOG_CPP(0, "Creating LandmarkZmqComponent with camera " << landmarksCamera);
      LandmarkROSComponent* lmcz = new LandmarkROSComponent(&entity, (RcsGraph*)graph);
      lmc = lmcz;
      ret = lmcz;
    }
#endif

    if (getKey(argsVec, "-yolo_tracking" + suffix))
    {
      lmc->addYoloTracker(landmarksCamera);
    }

    if (getKey(argsVec, "-image_tracking" + suffix))
    {
      auto imgTracker = std::make_unique<ImageTracker>(&entity, landmarksCamera);
      lmc->addTracker(std::move(imgTracker));
    }

    if (getKey(argsVec, "-virtual_image_tracking" + suffix))
    {
      std::string virtual_camera_type = "AzureKinect_WFOV";
      int virtual_camera_width = 640;
      int virtual_camera_height = 480;

      getKeyValuePair<int>(argsVec, "-virtual_image_tracking.width" + suffix, virtual_camera_width);
      getKeyValuePair<int>(argsVec, "-virtual_image_tracking.height" + suffix, virtual_camera_height);
      getKeyValuePair<std::string>(argsVec, "-virtual_image_tracking.camera_type"+suffix, virtual_camera_type);

      auto imgTracker = std::make_unique<VirtualImageTracker>(&entity, landmarksCamera, virtual_camera_type,
                                                              virtual_camera_width, virtual_camera_height);
      lmc->addTracker(std::move(imgTracker));
    }

    if (getKey(argsVec, "-face_tracking" + suffix))
    {
      std::string faceAgentName, faceBdyName;
      getKeyValuePair<std::string>(argsVec, "-face_tracking.agent" + suffix, faceAgentName);
      getKeyValuePair<std::string>(argsVec, "-face_tracking.face_body_name" + suffix, faceBdyName);

      if (faceBdyName.empty())
      {
        faceBdyName = FaceTracker::findFaceOfAgent(scene, graph, faceAgentName);
        RCHECK_MSG(!faceBdyName.empty(), "Couldn't find face body for agent '%s'", faceAgentName.c_str());
      }

      TrackerBase* tr = lmc->addFaceTracker(faceBdyName, landmarksCamera, faceAgentName);
      FaceTracker* ftr = dynamic_cast<FaceTracker*>(tr);
      RCHECK(ftr);
      ftr->registerAgentAppearDisappearCallback([ret](std::string agentName, bool appear)
      {
        std::string appearStr = appear ? "' appeared" : "' disappered";
        RLOG_CPP(1, "Agent '" << agentName << appearStr);
        ret->getEntity()->publish("AgentChanged", agentName, appear);
      });


      entity.subscribe("RenameAgent", &FaceTracker::onRenameAgent, ftr);
    }

    if (getKey(argsVec, "-aruco_tracking" + suffix))
    {
      std::string arucoBaseBdyName = "aruco_base";
      getKeyValuePair<std::string>(argsVec, "-aruco_base" + suffix, arucoBaseBdyName);
      lmc->addArucoTracker(landmarksCamera, arucoBaseBdyName);
    }

    if (getKey(argsVec, "-skeleton_tracking" + suffix))
    {
      RLOG(0, "Enabling Azure skeleton tracker");
      double r_agent = DBL_MAX;
      getKeyValuePair<double>(argsVec, "-skeleton_radius" + suffix, r_agent);

      // Add skeleton tracker and all human agents in the scene
      int numAgents = lmc->addSkeletonTrackerForAgents(scene, r_agent, landmarksCamera);

      auto skeletonTrackers = lmc->getTrackers<AzureSkeletonTracker>();
      if (skeletonTrackers.size()==1)
      {
        RLOG(0, "Registering AgentChanged event");
        skeletonTrackers[0]->registerAgentAppearDisappearCallback([ret](const std::string& agentName, bool appear)
        {
          std::string appearStr = appear ? "' appeared" : "' disappered";
          RLOG_CPP(0, "Agent '" << agentName << appearStr);
          ret->getEntity()->publish("AgentChanged", agentName, appear);
        });

      }


      RLOG(0, "Done adding skeleton tracker with %d agents", numAgents);
    }

  }

  return ret;
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
    argP.addDescription("-jacoShm6", "Start with Jaco6 Shm (right)");
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

    if (getKey(argvStrVec, "-jacoShm6"))
    {
      ComponentBase* c = RoboJacoShmComponent::create(&entity, graph, JacoShmComponent::Jaco6);
      components.push_back(c);
    }
  }

  if (dryRun)
  {
    argP.addDescription("-jacoGen2_7_Zmq_right", "Start with Jaco Gen2 7-dof right");
    argP.addDescription("-jacoGen2_7_Zmq_left", "Start with Jaco7 left");
    argP.addDescription("-jacoGen2_6_Zmq", "Start with Jaco6");
    argP.addDescription("-jacoGen2_7_Zmq_right.ip", "Jaco Gen2 7-dof right ip address (Default: localhost)");
    argP.addDescription("-jacoGen2_7_Zmq_left.ip", "Jaco7 left ip address (Default: localhost)");
    argP.addDescription("-jacoGen2_6_Zmq.ip", "Jaco6 ip address (Default: localhost)");
  }
  else
  {
    if (getKey(argvStrVec, "-jacoGen2_7_Zmq_right"))
    {
      const double dt_commands = 0.02;
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-jacoGen2_7_Zmq_right.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40020";
      std::string otherSend="tcp://" + other_ip + ":40021";
      ComponentBase* c = new JacoZmqComponent(&entity, dt_commands, "Jaco7", "_right", otherRecv, otherSend);
      components.push_back(c);
    }

    if (getKey(argvStrVec, "-jacoGen2_7_Zmq_left"))
    {
      const double dt_commands = 0.02;
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-jacoGen2_7_Zmq_left.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40022";
      std::string otherSend="tcp://" + other_ip + ":40023";
      ComponentBase* c = new JacoZmqComponent(&entity, dt_commands, "Jaco7", "_right", otherRecv, otherSend);
      components.push_back(c);
    }

    if (getKey(argvStrVec, "-jacoGen2_6_Zmq"))
    {
      const double dt_commands = 0.02;
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-jacoGen2_7_Zmq_left.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40018";
      std::string otherSend="tcp://" + other_ip + ":40019";
      ComponentBase* c = new JacoZmqComponent(&entity, dt_commands, "Jaco6", "", otherRecv, otherSend);
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
    argP.addDescription("-pw70_vel", "Start with PW70 PTU in velocity mode");
    argP.addDescription("-pw70_zmq", "Start with PW70 PTU client connecting through zmq");
    argP.addDescription("-pw70_zmq.ip", "For -pw70_zmq option only: PW70 ip address (Default: localhost)");
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
    else if (getKey(argvStrVec, "-pw70_zmq"))
    {
      const double dt_commands = 0.02;
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-pw70_zmq.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40006";
      std::string otherSend="tcp://" + other_ip + ":40007";
      components.push_back(new PW70ZmqComponent(&entity, dt_commands, "", otherRecv, otherSend));
    }
  }

  if (dryRun)
  {
    argP.addDescription("-jacoGen3Zmq_left", "Start with Kinova Kortex component (left arm)");
    argP.addDescription("-jacoGen3Zmq_left.ip", "Kinova Kortex server ip (left arm, default: localhost)");
    argP.addDescription("-jacoGen3Zmq_right", "Start with Kinova Kortex component (right arm)");
    argP.addDescription("-jacoGen3Zmq_right.ip", "Kinova Kortex server ip (right arm, default: localhost)");
    argP.addDescription("-frankaZmq_left", "Start with Franka component (left arm)");
    argP.addDescription("-frankaZmq_left.ip", "Franka server ip (left arm, default: localhost)");
    argP.addDescription("-frankaZmq_right", "Start with Franka component (right arm)");
    argP.addDescription("-frankaZmq_right.ip", "Franka server ip (right arm, default: localhost)");
    argP.addDescription("-allegroZmq_left", "Start with Allegro component (left hand)");
    argP.addDescription("-allegroZmq_left.ip", "Allegro server ip (left hand, default: localhost)");
    argP.addDescription("-allegroZmq_right", "Start with Allegro component (right hand)");
    argP.addDescription("-allegroZmq_right.ip", "Allegro server ip (right hand, default: localhost)");
  }
  else
  {
    if (getKey(argvStrVec, "-jacoGen3Zmq_left"))
    {
      std::string suffix = "_left";
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-jacoGen3Zmq_left.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40004";
      std::string otherSend="tcp://" + other_ip + ":40005";
      const double dt_commands = 0.01;
      components.push_back(new aff::KortexComponent(&entity, dt_commands, suffix,
                                                    otherRecv, otherSend));
    }

    if (getKey(argvStrVec, "-jacoGen3Zmq_right"))
    {
      std::string suffix = "_right";
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-jacoGen3Zmq_right.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40002";
      std::string otherSend="tcp://" + other_ip + ":40003";
      const double dt_commands = 0.01;
      components.push_back(new aff::KortexComponent(&entity, dt_commands, suffix,
                                                    otherRecv, otherSend));
    }

    if (getKey(argvStrVec, "-jacoGen3Zmq"))
    {
      std::string suffix = "";
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-jacoGen3Zmq.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40002";
      std::string otherSend="tcp://" + other_ip + ":40003";
      const double dt_commands = 0.01;
      components.push_back(new aff::KortexComponent(&entity, dt_commands, suffix,
                                                    otherRecv, otherSend));
    }

    if (getKey(argvStrVec, "-frankaZmq_right"))
    {
      std::string suffix = "_right";
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-frankaZmq_right.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40008";
      std::string otherSend="tcp://" + other_ip + ":40009";
      const double dt_commands = 0.01;
      components.push_back(new aff::FrankaComponent(&entity, dt_commands, suffix,
                                                    otherRecv, otherSend));
    }

    if (getKey(argvStrVec, "-frankaZmq_left"))
    {
      std::string suffix = "_left";
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-frankaZmq_left.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40010";
      std::string otherSend="tcp://" + other_ip + ":40011";
      const double dt_commands = 0.01;
      components.push_back(new aff::FrankaComponent(&entity, dt_commands, suffix,
                                                    otherRecv, otherSend));
    }

    if (getKey(argvStrVec, "-allegroZmq_right"))
    {
      std::string suffix = "_right";
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-allegroZmq_right.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40012";
      std::string otherSend="tcp://" + other_ip + ":40013";
      const double dt_commands = 0.02;
      components.push_back(new aff::AllegroComponent(&entity, dt_commands, suffix,
                                                     otherRecv, otherSend));
    }

    if (getKey(argvStrVec, "-allegroZmq_left"))
    {
      std::string suffix = "_left";
      std::string other_ip = "localhost";
      getKeyValuePair<std::string>(argvStrVec, "-allegroZmq_left.ip", other_ip);
      std::string otherRecv="tcp://" + other_ip + ":40014";
      std::string otherSend="tcp://" + other_ip + ":40015";
      const double dt_commands = 0.02;
      components.push_back(new aff::AllegroComponent(&entity, dt_commands, suffix,
                                                     otherRecv, otherSend));
    }

  }


  if (dryRun)
  {
    argP.addDescription("-cubemars", "Start with Cubemars client connecting through zmq");
  }
  else if (getKey(argvStrVec, "-cubemars"))
  {
    const double dt_commands = 0.02;
    std::string other_ip = "localhost";
    std::string otherRecv="tcp://" + other_ip + ":40028";
    std::string otherSend="tcp://" + other_ip + ":40029";
    components.push_back(new CubemarsComponent(&entity, dt_commands, "", otherRecv, otherSend));
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
    argP.hasArgument("-eye_ik", "Start with eye model");
    argP.hasArgument("-eye_ik.camera_name", "Name of camera body for eye model");
  }
  else if (getKey(argvStrVec, "-eye_ik"))
  {
    std::string camera_name = "camera_0";
    getKeyValuePair<std::string>(argvStrVec, "-eye_ik.camera_name", camera_name);
    components.push_back(new EyeModelIKComponent(&entity, graph, camera_name));
  }

  if (dryRun)
  {
    argP.hasArgument("-respeaker_usb", "Start with respeaker USB");
  }
  else if (getKey(argvStrVec, "-respeaker_usb"))
  {
    components.push_back(new RespeakerUSBComponent(&entity));
  }

  if (dryRun)
  {
    argP.hasArgument("-zmq_listener", "Start with face recognition client");
    argP.hasArgument("-zmq_listener_ip", "Server ip adress (default: tcp://*:5556)");
  }
  else if (getKey(argvStrVec, "-zmq_listener"))
  {
    std::string ip_address = "tcp://*:5556";
    getKeyValuePair<std::string>(argvStrVec, "-zmq_listener_ip", ip_address);
    components.push_back(new ZmqJsonSubscriber(&entity, ip_address));
  }

  if (dryRun)
  {
    argP.addDescription("-tts", "Start with native text-to-speech");
    argP.addDescription("-piper_tts_alan", "Start with piper text-to-speech, Alan's voice");
    argP.addDescription("-piper_tts_joe", "Start with piper text-to-speech, Joe's voice");
    argP.addDescription("-piper_tts_kathleen", "Start with piper text-to-speech, Kathleen's voice");
    argP.addDescription("-piper_tts_ryan", "Start with piper text-to-speech, Ryan's voice");
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
  else if (getKey(argvStrVec, "-piper_tts_kathleen"))
  {
    auto tts = new TTSComponent(&entity, "piper");
    tts->setPiperVoice("kathleen");
    components.push_back(tts);
  }
  else if (getKey(argvStrVec, "-piper_tts_ryan") || getKey(argvStrVec, "-piper"))
  {
    auto tts = new TTSComponent(&entity, "piper");
    tts->setPiperVoice("ryan");
    components.push_back(tts);
  }

  if (dryRun)
  {
    argP.hasArgument("-websocket", "Start with websocket connection");
    argP.hasArgument("-websocket_port", "Websocket port (default: 35000)");
    argP.hasArgument("-websocket_eventToPublish", "Name of published event (default: ActionSequence)");
  }
  else if (getKey(argvStrVec, "-websocket"))
  {
    int port = 35000;
    std::string eventToPublish = "ActionSequence";
    getKeyValuePair(argvStrVec, "-websocket_port", port);
    getKeyValuePair(argvStrVec, "-websocket_eventToPublish", eventToPublish);
    components.push_back(new WebsocketActionComponent(&entity, port, eventToPublish));
  }

  // The debug graphics will be handled in initGraphics.
  if (dryRun)
  {
    argP.addDescription("-landmarks_connection", "Connection string, default is tcp://localhost:40000");
    argP.addDescription("-landmarks_zmq", "Start with ZMQ landmarks component");
    argP.addDescription("-landmarks_router", "Start with ZMQ landmarks router-dealer network component");
    argP.addDescription("-landmarks_camera", "For '-landmarks_zmq': Body name of camera in which the landmarks are assumed to be represented. Default: camera_0");
    argP.addDescription("-face_tracking", "For '-landmarks_zmq': Start with Mediapipe face tracking");
    argP.addDescription("-face_bodyName", "For '-face_tracking' and '-face_gesture': Name of the face's RcsBody (Default: face)");
    argP.addDescription("-aruco_tracking", "For '-landmarks_zmq': Start with Aruco marker tracking");
    argP.addDescription("-yolo_tracking", "For '-landmarks_zmq': Start with Yolo tracking");
    argP.addDescription("-aruco_base", "For '-landmarks_zmq' and '-aruco_tracking': Name of aruco base marker (default: \"aruco_base\")");
    argP.addDescription("-skeleton_tracking", "For '-landmarks_zmq': Start with skeleton tracking");
    argP.addDescription("-skeleton_radius", "For '-landmarks_zmq' and '-skeleton_tracking': Radius of skeleton detections (default: infinity)");
    argP.addDescription("-agent_welcome", "For '-landmarks_router' and '-skeleton_tracking': Callback for agent appearing and disappearing");
    argP.addDescription("-agent_welcome.recognize", "For '-landmarks_router' and '-agent_welcome': Recognize face");

    argP.addDescription("-virtual_image_tracking", "For '-landmarks_router': Start with virtual image tracking");
    argP.addDescription("-virtual_image_tracking.width", "For '-landmarks_router' and '-virtual_image_tracking': Width of captured image in pixels (Default: 640)");
    argP.addDescription("-virtual_image_tracking.height", "For '-landmarks_router' and '-virtual_image_tracking': Heigth of captured image in pixels (Default: 480)");
    argP.addDescription("-virtual_image_tracking.camera_type", "For '-landmarks_router' and '-virtual_image_tracking': Type of camera (Default: AzureKinect_WFOV. Choices: Kinect_v2, Logitech_C910, AzureKinect. See VirtualCamera.cpp)");
  }
  else if (getKey(argvStrVec, "-landmarks_zmq"))
  {
    components.push_back(createLandmarkComponent(entity, graph, scene, argvString,
                                                 LandmarkParentClass::LandmarkZmqComponent, ""));
  }
  else if (getKey(argvStrVec, "-landmarks_router"))
  {
    components.push_back(createLandmarkComponent(entity, graph, scene, argvString,
                                                 LandmarkParentClass::ZmqRouterComponent, ""));

    if (getKey(argvStrVec, "-agent_welcome"))
    {
      bool with_fr = getKey(argvStrVec, "-agent_welcome.recognize");
      components.push_back(new AgentWelcomeComponent(&entity, scene, with_fr));
    }
#if defined WITH_WEBSOCKETCLIENTCOMPONENT
    components.push_back(new WebsocketClientComponent(&entity));
#endif
  }

  if (getKey(argvStrVec, "-landmarks_zmq2"))
  {
    components.push_back(createLandmarkComponent(entity, graph, scene, argvString,
                                                 LandmarkParentClass::LandmarkZmqComponent, "2"));
  }

  if (getKey(argvStrVec, "-landmarks_zmq3"))
  {
    components.push_back(createLandmarkComponent(entity, graph, scene, argvString,
                                                 LandmarkParentClass::LandmarkZmqComponent, "3"));
  }

  if (getKey(argvStrVec, "-landmarks_zmq4"))
  {
    components.push_back(createLandmarkComponent(entity, graph, scene, argvString,
                                                 LandmarkParentClass::LandmarkZmqComponent, "4"));
  }

  if (getKey(argvStrVec, "-landmarks_zmq5"))
  {
    components.push_back(createLandmarkComponent(entity, graph, scene, argvString,
                                                 LandmarkParentClass::LandmarkZmqComponent, "5"));
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
    components.push_back(createLandmarkComponent(entity, graph, scene, argvString,
                                                 LandmarkParentClass::LandmarkROSComponent));
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
    components.push_back(new MirrorEyeComponent(&entity, scene, graph, pubTopic, gazeTopic, camTopic));
  }
#endif



  for (size_t i = 0; i < components.size(); ++i)
  {
    RCHECK_MSG(components[i], "Found NULL component at index %zu", i);
  }

  return components;
}

}   // namespace
