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
#include "PtuActionComponent.h"
#include "RespeakerComponent.h"
#include "NuanceTTSComponent.h"
#include "TTSComponent.h"

#include <Rcs_typedef.h>
#include <Rcs_cmdLine.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>

#include <thread>


namespace aff
{

std::vector<ComponentBase*> getHardwareComponents(EntityBase& entity,
                                                  const RcsGraph* graph,
                                                  const ActionScene* scene,
                                                  bool dryRun)
{
  Rcs::CmdLineParser argP;
  std::vector<ComponentBase*> components;

  if (argP.hasArgument("-jacoShm7r", "Start with Jaco7 Shm right") && (!dryRun))
  {
    components.push_back(getComponent(entity, graph, scene, "-jacoShm7r"));
  }

  if (argP.hasArgument("-jacoShm7l", "Start with Jaco7 Shm left") && (!dryRun))
  {
    components.push_back(getComponent(entity, graph, scene, "-jacoShm7l"));
  }

  if (argP.hasArgument("-ptu", "Start with Scitos PTU") && (!dryRun))
  {
    components.push_back(getComponent(entity, graph, scene, "-ptu"));
  }

  for (size_t i=0; i< components.size(); ++i)
  {
    RCHECK_MSG(components[i], "Found NULL hardware component at index %zu", i);
  }

  return components;
}

std::vector<ComponentBase*> getComponents(EntityBase& entity,
                                          const RcsGraph* graph,
                                          const ActionScene* scene,
                                          bool dryRun)
{
  Rcs::CmdLineParser argP;
  std::vector<ComponentBase*> components;

  if (argP.hasArgument("-respeaker", "Start with Respeaker") && (!dryRun))
  {
    components.push_back(getComponent(entity, graph, scene, "-respeaker"));
  }

  if (argP.hasArgument("-tts", "Start with native text-to-speech") && (!dryRun))
  {
    components.push_back(getComponent(entity, graph, scene, "-tts"));
  }

  if (argP.hasArgument("-nuance_tts", "Start with Nuance ROS text-to-speech") && (!dryRun))
  {
    components.push_back(getComponent(entity, graph, scene, "-nuance_tts"));
  }

  for (size_t i = 0; i < components.size(); ++i)
  {
    RCHECK_MSG(components[i], "Found NULL component at index %zu", i);
  }

  return components;
}

ComponentBase* getComponent(EntityBase& entity,
                            const RcsGraph* graph,
                            const ActionScene* scene,
                            const std::string& componentName)
{

  if (componentName == "-tts")
  {
    return new TTSComponent(&entity);
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

void initROS(double rosDt)
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

}   // namespace
