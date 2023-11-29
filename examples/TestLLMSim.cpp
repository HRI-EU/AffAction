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

#include <ExampleLLMSim.h>
#include <ArucoTracker.h>
#include <AzureSkeletonTracker.h>
#include <LandmarkZmqComponent.hpp>
#include <NuanceTTSComponent.h>
#include <TTSComponent.h>
#include <Rcs_macros.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_typedef.h>

#include <SegFaultHandler.h>

#if !defined(_MSC_VER)
#include <X11/Xlib.h>
#endif

#include <csignal>
#include <fstream>

#define INIT_AGENTS 4

RCS_INSTALL_ERRORHANDLERS

aff::ExampleLLMSim* examplePtr = NULL;


void quit(int /*sig*/)
{
  static int kHit = 0;
  examplePtr->stop();
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit + 1);
  kHit++;

  if (kHit == 2)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}

// Run with tracking skeletons from file:
//   bin/TestLLMSim -dir config/xml/Affaction/examples/ -f g_aruco.xml -tracking -jsonFile config/xml/Affaction/data/skeleton.json
//   bin/TestLLMSim -dir config/xml/SmileSimulator/unittest/ -f g_scenario_unittest_multiple_agents.xml
int main(int argc, char** argv)
{
#if !defined(_MSC_VER)
  // Avoid crashes when running remotely.
  XInitThreads();
#endif

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  Rcs_addResourcePath(RCS_CONFIG_DIR);

  aff::ExampleLLMSim ex(argc, argv);
  examplePtr = &ex;
  bool success = ex.init(argc, argv);

  Rcs::CmdLineParser argP;

  bool withTracking = argP.hasArgument("-tracking", "Use tracking component");
  std::unique_ptr<aff::LandmarkZmqComponent> lmc;

  if (withTracking)
  {
    RLOG(0, "Adding trackers");
    std::string connection="tcp://localhost:5555";
    size_t numSkeletons = 3;
    double r_agent = DBL_MAX;
    argP.getArgument("-jsonFile", &connection,
                     "Json file instead of zmq connection (default: python_landmark_input.json)");
    argP.getArgument("-numSkeletons", &numSkeletons,
                     "Max. number of skeletons to be tracked (default: %zu)", numSkeletons);
    argP.getArgument("-r_agent", &r_agent, "Radius around skeleton default position to start tracking (default: inf)");

    lmc = std::unique_ptr<aff::LandmarkZmqComponent>(new aff::LandmarkZmqComponent(&ex.entity, connection));
    lmc->setScenePtr(ex.controller->getGraph(), ex.actionC->getDomain());

    ex.viewer->setKeyCallback('W', [&ex](char k)
    {
      RLOG(0, "Calibrate camera");
      ex.entity.publish("EstimateCameraPose");
    }, "Calibrate camera");

    const RcsBody* cam = RcsGraph_getBodyByName(ex.controller.get()->getGraph(), "camera");
    RCHECK(cam);
    lmc->addArucoTracker(cam->name, "aruco_base");

#if INIT_AGENTS == 0
    std::vector<HTr> defaultPos(numSkeletons);
    for (auto& trf : defaultPos)
    {
      HTr_setIdentity(&trf);
    }

    const RcsBody* seat_agent_09 = RcsGraph_getBodyByName(ex.controller.get()->getGraph(), "seat_agent_09");
    const RcsBody* seat_agent_12 = RcsGraph_getBodyByName(ex.controller.get()->getGraph(), "seat_agent_12");
    const RcsBody* seat_agent_15 = RcsGraph_getBodyByName(ex.controller.get()->getGraph(), "seat_agent_15");
    std::vector<std::string> agentNames;
    if (seat_agent_09 && seat_agent_12 && seat_agent_15 && defaultPos.size()>=3)
    {
      Vec3d_copy(defaultPos[0].org, seat_agent_09->A_BI.org);
      Vec3d_copy(defaultPos[1].org, seat_agent_12->A_BI.org);
      Vec3d_copy(defaultPos[2].org, seat_agent_15->A_BI.org);
      agentNames.push_back("Bob");
      agentNames.push_back("Michael");
      agentNames.push_back("Anna");
    }

#elif INIT_AGENTS == 1
    std::vector<HTr> defaultPos;
    std::vector<std::string> agentNames;

    aff::ActionScene* scene = ex.actionC->getDomain();
    RCHECK(scene);

    for (auto& agent : scene->agents)
    {
      aff::HumanAgent* human = dynamic_cast<aff::HumanAgent*>(agent);

      if (!human)
      {
        continue;
      }

      HTr place;
      HTr_setIdentity(&place);
      Vec3d_copy(place.org, human->defaultPos);
      defaultPos.push_back(place);
      agentNames.push_back(agent->name);
      RLOG(0, "User %s has default position %.3f %.3f %.3f",
           human->name.c_str(), human->defaultPos[0], human->defaultPos[1], human->defaultPos[2]);
    }

    RLOG(0, "Adding Skeleton tracker with %zu humans", defaultPos.size());

    lmc->addSkeletonTracker(defaultPos, r_agent, ex.viewer.get(), agentNames, scene);

#elif INIT_AGENTS == 2
    // Add skeleton tracker and agents
    aff::AzureSkeletonTracker* sTracker = lmc->addSkeletonTracker(numSkeletons);
    sTracker->addAgent("Bob");
    sTracker->addAgent("Dave");
    sTracker->addAgent("Peter");
    // Initialize the graphics for the skeletons.
    sTracker->initGraphics(ex.viewer.get());

#elif INIT_AGENTS == 3

    // Add skeleton tracker and ALL agents in the scene
    aff::AzureSkeletonTracker* sTracker = lmc->addSkeletonTracker(numSkeletons);
    sTracker->addAgents();
    // Initialize the graphics for the skeletons.
    sTracker->initGraphics(ex.viewer.get());

#elif INIT_AGENTS == 4   // Same as python interface

    // Add skeleton tracker and ALL agents in the scene
    int nSkeletons = lmc->addSkeletonTrackerForAgents(r_agent);
    for (auto& tracker : lmc->getTrackers())
    {
      aff::AzureSkeletonTracker* st = dynamic_cast<aff::AzureSkeletonTracker*>(tracker.get());
      if (st)
      {
        st->initGraphics(ex.viewer.get());
      }
    }

    RLOG(0, "Added skeleton tracker with %d agents", nSkeletons);

#endif

    // Initialize all tracker camera transforms from the xml file
    lmc->setCameraTransform(&cam->A_BI);



    RLOG(0, "Done adding trackers");
  }

  //std::unique_ptr<aff::TTSComponent> tts;
  //if (argP.hasArgument("-tts"))
  //{
  //  tts = std::make_unique<aff::TTSComponent>(&ex.entity);
  //}

  //std::unique_ptr<aff::NuanceTTSComponent> nuanceTTS;
  //if (argP.hasArgument("-nuance_tts"))
  //{
  //  if (tts)
  //  {
  //    RLOG(0, "TTS already running - nuance ignored");
  //  }
  //  else
  //  {
  //    nuanceTTS = std::make_unique<aff::NuanceTTSComponent>(&ex.entity);
  //  }
  //}

  if (success)
  {
    ex.start();  // This will block until "Stop" is published
  }

  std::string starLine(80, '*');
  RMSG_CPP("\n\n" + starLine + "\n* TestLLMSim exits with "
           << ex.getNumFailedActions() << " failed actions\n" + starLine);

  xmlCleanupParser();

  return ex.getNumFailedActions();
}
