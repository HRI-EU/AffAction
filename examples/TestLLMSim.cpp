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
#include <LandmarkZmqComponent.h>
#include <HardwareComponent.h>
#include <StringParserTools.hpp>
#include <Rcs_macros.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_typedef.h>
#include <Rcs_timer.h>

#include <SegFaultHandler.h>

#if !defined(_MSC_VER)
#include <X11/Xlib.h>
#endif

#include <csignal>
#include <fstream>
#include <iostream>

RCS_INSTALL_ERRORHANDLERS

aff::ExampleActionsECS* examplePtr = nullptr;


void quit(int /*sig*/)
{
  static int kHit = 0;
  if (examplePtr)
  {
    examplePtr->stop();
  }
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
static int testLLMSim(int argc, char** argv)
{
#if !defined(_MSC_VER)
  // Avoid crashes when running remotely.
  XInitThreads();
#endif

  RLOG(0, "testLLMSim()");
  Rcs_addResourcePath(RCS_CONFIG_DIR);

  aff::ExampleActionsECS ex(argc, argv);
  examplePtr = &ex;
  bool success = ex.init(argc, argv);

  Rcs::CmdLineParser argP;
  bool withAruco = argP.hasArgument("-aruco", "Use aruco tracker");
  bool withAzure = argP.hasArgument("-azure", "Use azure people tracker");
  bool withFace = argP.hasArgument("-face", "Use Mediapipe face tracker");
  bool withTracking = argP.hasArgument("-tracking", "Use tracking component");

  if (withAruco || withAzure || withFace)
  {
    withTracking = true;
  }

  aff::LandmarkZmqComponent* lmc = nullptr;
  std::string lmArgs;

  // Assemble string containing all args
  if (withTracking)
  {
    RLOG(0, "Enabling tracking");
    std::string connection = "tcp://localhost:5555";
    argP.getArgument("-jsonFile", &connection, "Json file instead of zmq connection (default: python_landmark_input.json)");
    lmArgs += "-landmarks_connection " + connection + " -landmarks_camera head_kinect_lens ";
    if (withFace)
    {
      lmArgs += "-face_tracking -face_bodyName face ";
    }

    // Create components
    aff::ComponentBase* c = createComponent(ex.getEntity(), ex.getGraph(), ex.getScene(), "-landmarks_zmq", lmArgs);
    ex.addComponent(c);
    lmc = static_cast<aff::LandmarkZmqComponent*>(c);
    if (withFace)
    {
      ex.addComponent(createComponent(ex.getEntity(), ex.getGraph(), ex.getScene(), "-face_gesture", lmArgs));
      ex.addComponent(createComponent(ex.getEntity(), ex.getGraph(), ex.getScene(), "-camera_view", lmArgs));
    }
  }



  if (withAruco)
  {
    RLOG(0, "Enabling aruco traker");
    lmc->addArucoTracker("camera_0", "aruco_base");
    RLOG(0, "Done adding aruco tracker");
  }

  if (withAzure)
  {
    RLOG(0, "Enabling Azure skeleton tracker");
    //    RCHECK(cam);
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

  // Because initGraphics() has already been called
  if (lmc)
  {
    RLOG(0, "Enabling lmc debug graphics");
    lmc->createDebugGraphics(ex.viewer.get());
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

static int testSceneQuery(int argc, char** argv)
{
  RLOG(0, "testSceneQuery()");
  aff::ExampleActionsECS ex(argc, argv);
  bool success = aff::SceneQueryPool::test(&ex);
  RLOG(0, "aff::SceneQueryPool::test() %s", success ? "succeeded" : "failed");

  return 0;
}

static int testPTU(int argc, char** argv)
{
  RLOG(0, "testPTU()");

  Rcs_addResourcePath(RCS_CONFIG_DIR);

  aff::ExampleActionsECS ex(argc, argv);
  examplePtr = &ex;
  bool success = ex.init(argc, argv);

  int count = 0;
  double t0 = Timer_getSystemTime();
  ex.getEntity().subscribe("SetJointCommand", [&count, &t0](const MatNd* q_des)
  {
    ++count;
    if (count%10==0)
    {
      double t = Timer_getSystemTime()-t0;
      double pan = RCS_DEG2RAD(30.0)*sin(t);
      RLOG(0, "Pan %f: t = %.4f", pan, t);
      examplePtr->getEntity().publish("PtuPanTiltCommand", pan, 0.0);
    }

  });

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

static int testStringParsing()
{
  std::vector<std::string> params = { "key1", "True", "key2", "false", "key3", "42", "key4", "99.99", "key5", "99.99", "key6", "-5" };

  bool val1, val2;
  int val3, val5;
  double val4;
  size_t val6;

  if (aff::getKeyValuePair(params, "key1", val1) == 0)
  {
    std::cout << "key1: " << std::boolalpha << val1 << "\n";  // Should print "key1: true"
  }

  if (aff::getKeyValuePair(params, "key2", val2) == 0)
  {
    std::cout << "key2: " << std::boolalpha << val2 << "\n";  // Should print "key2: false"
  }

  if (aff::getKeyValuePair(params, "key3", val3) == 0)
  {
    std::cout << "key3: " << val3 << "\n";  // Should print "key3: 42"
  }

  if (aff::getKeyValuePair(params, "key4", val4) == 0)
  {
    std::cout << "key4: " << val4 << "\n";  // Should print "key4: 99.99"
  }

  if (aff::getKeyValuePair(params, "key5", val5) == 0)
  {
    std::cout << "key5: " << val5 << "\n";  // Should not print anythong
  }

  if (aff::getKeyValuePair(params, "key6", val6) == 0)
  {
    std::cout << "key6: " << val6 << "\n";  // Should not print anythong
  }

  return 0;
}

int main(int argc, char** argv)
{
  // Ctrl-C callback handler
  signal(SIGINT, quit);

  Rcs::CmdLineParser argP(argc, argv);
  int mode = 0, res = 0;

  argP.getArgument("-m", &mode, "Test mode (default: %d)", mode);

  switch (mode)
  {
    case 0:
      res = testLLMSim(argc, argv);
      break;

    case 1:
      res = testStringParsing();
      break;

    case 2:
      res = testSceneQuery(argc, argv);
      break;

    case 3:
      res = testPTU(argc, argv);
      break;

    default:
      RMSG("No such mode: %d", mode);
  }

  return res;
}
