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
#include "WebsocketActionComponent.h"

#if defined USE_ROS
#include "ros/PtuActionComponent.h"
#include "ros/RespeakerComponent.h"
#include "ros/NuanceTTSComponent.h"
#include "ros/LandmarkROSComponent.hpp"
#endif

#include <Rcs_typedef.h>
#include <Rcs_cmdLine.h>
#include <Rcs_timer.h>
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

  rosInitMtx.lock();

  if (rosInitialized)
  {
    RLOG(1, "ROS already initialized - doing nothing");
    rosInitMtx.unlock();
    return;
  }

  rosInitialized = true;
  rosInitMtx.unlock();

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
                                                     bool dryRun)
{
  Rcs::CmdLineParser argP;
  std::vector<ComponentBase*> components;

  if (argP.hasArgument("-jacoShm7r", "Start with Jaco7 Shm right") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-jacoShm7r"));
  }

  if (argP.hasArgument("-jacoShm7l", "Start with Jaco7 Shm left") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-jacoShm7l"));
  }

  if (argP.hasArgument("-ptu", "Start with Scitos PTU") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-ptu"));
  }

  for (size_t i=0; i< components.size(); ++i)
  {
    RCHECK_MSG(components[i], "Found NULL hardware component at index %zu", i);
  }

  return components;
}

std::vector<ComponentBase*> createComponents(EntityBase& entity,
                                             const RcsGraph* graph,
                                             const ActionScene* scene,
                                             bool dryRun)
{
  Rcs::CmdLineParser argP;
  std::vector<ComponentBase*> components;

  if (argP.hasArgument("-respeaker", "Start with Respeaker") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-respeaker"));
  }

  if (argP.hasArgument("-tts", "Start with native text-to-speech") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-tts"));
  }

  if (argP.hasArgument("-websocket", "Start with websocket connection on port 35000") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-websocket"));
  }

  if (argP.hasArgument("-nuance_tts", "Start with Nuance ROS text-to-speech") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-nuance_tts"));
  }

  if (argP.hasArgument("-landmarks_ros", "Start with ROS landmarks component") && (!dryRun))
  {
    components.push_back(createComponent(entity, graph, scene, "-landmarks_ros"));
  }

  for (size_t i = 0; i < components.size(); ++i)
  {
    RCHECK_MSG(components[i], "Found NULL component at index %zu", i);
  }

  return components;
}

ComponentBase* createComponent(EntityBase& entity,
                               const RcsGraph* graph,
                               const ActionScene* scene,
                               const std::string& componentName)
{

  if (componentName == "-tts")
  {
    return new TTSComponent(&entity);
  }
  else if (componentName == "-websocket")
  {
    return new WebsocketActionComponent(&entity);
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
