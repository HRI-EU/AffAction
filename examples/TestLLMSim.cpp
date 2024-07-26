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

#include <ExampleActionsECS.h>
#include <LandmarkZmqComponent.hpp>
#include <CameraViewComponent.h>
#include <FaceGestureComponent.h>
#include <FaceTracker.h>
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

RCS_INSTALL_ERRORHANDLERS

aff::ExampleActionsECS* examplePtr = NULL;


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
//   bin/TestLLMSim -dir config/xml/Affaction/unittest/ -f g_scenario_unittest_multiple_agents.xml
int main(int argc, char** argv)
{
#if !defined(_MSC_VER)
  // Avoid crashes when running remotely.
  XInitThreads();
#endif

  // Ctrl-C callback handler
  signal(SIGINT, quit);

  Rcs_addResourcePath(RCS_CONFIG_DIR);

  aff::ExampleActionsECS ex(argc, argv);
  examplePtr = &ex;
  bool success = ex.init(argc, argv);

  Rcs::CmdLineParser argP;

  bool testSceneQuery = false;
  argP.getArgument("-testSceneQuery", &testSceneQuery, "Test ConcurrentSceneQuery pool");

  if (testSceneQuery)
  {
    bool success = aff::SceneQueryPool::test(&ex);
    RPAUSE_MSG("aff::SceneQueryPool::test() %s", success ? "succeeded" : "failed");
  }



  bool withAruco = argP.hasArgument("-aruco", "Use aruco tracker");
  bool withAzure = argP.hasArgument("-azure", "Use azure people tracker");
  bool withFace = argP.hasArgument("-face", "Use Mediapipe face tracker");
  bool withTracking = argP.hasArgument("-tracking", "Use tracking component");

  if (withAruco || withAzure || withFace)
  {
    withTracking = true;
  }

  std::unique_ptr<aff::LandmarkZmqComponent> lmc;
  const RcsBody* cam = nullptr;


  if (withTracking)
  {
    RLOG(0, "Adding LandmarkZmqComponent");
    cam = RcsGraph_getBodyByName(ex.getGraph(), "camera_0");
    cam = RcsGraph_getBodyByName(ex.getGraph(), "head_kinect_rgb_link");
    RCHECK(cam);
    std::string connection="tcp://localhost:5555";
    argP.getArgument("-jsonFile", &connection,
                     "Json file instead of zmq connection (default: python_landmark_input.json)");
    lmc = std::unique_ptr<aff::LandmarkZmqComponent>(new aff::LandmarkZmqComponent(&ex.getEntity(), connection));
    lmc->setScenePtr(ex.getGraph(), ex.getScene());
  }

  if (withFace)
  {
    auto ft = lmc->addFaceTracker("Daniel", cam->name);
    ex.addComponent(new aff::CameraViewComponent(&ex.getEntity(), "face", false));
    ex.addComponent(new aff::FaceGestureComponent(&ex.getEntity(), "face", dynamic_cast<aff::FaceTracker*>(ft)->getMesh()));
    RLOG(0, "%s adding face tracker", ft ? "SUCCESS" : "FAILURE");
  }

  if (withAruco)
  {
    RCHECK(cam);
    lmc->addArucoTracker(cam->name, "aruco_base");
    RLOG(0, "Done adding aruco tracker");
  }

  if (withAzure)
  {
    RCHECK(cam);
    size_t numSkeletons = 3;
    double r_agent = DBL_MAX;
    argP.getArgument("-numSkeletons", &numSkeletons,
                     "Max. number of skeletons to be tracked (default: %zu)", numSkeletons);
    argP.getArgument("-r_agent", &r_agent, "Radius around skeleton default position to start tracking (default: inf)");

    // Add skeleton tracker and ALL agents in the scene
    int nSkeletons = lmc->addSkeletonTrackerForAgents(r_agent);
    RLOG(0, "Added skeleton tracker with %d agents", nSkeletons);

    if (ex.getViewer())
    {
      ex.getViewer()->setKeyCallback('W', [&ex](char k)
      {
        RLOG(0, "Calibrate camera");
        ex.getEntity().publish("EstimateCameraPose", 20);
      }, "Calibrate camera");
    }

    RLOG(0, "Done adding skeleton tracker");
  }

  // Enable debug graphics
  if (lmc)
  {
    // Initialize all tracker camera transforms from the xml file
    RCHECK(cam);
    lmc->setCameraTransform(&cam->A_BI);

    if (ex.getViewer())
    {
      lmc->createDebugGraphics(ex.getViewer());
    }
  }


  if (success)
  {
    RLOG(0, "Starting loop");
    ex.start();  // This will block until "Stop" is published
  }

  std::string starLine(80, '*');
  RMSG_CPP("\n\n" + starLine + "\n* TestLLMSim exits with "
           << ex.getNumFailedActions() << " failed actions\n" + starLine);

  xmlCleanupParser();

  return ex.getNumFailedActions();
}
