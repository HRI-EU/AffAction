/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

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

#include "ExampleFlowMatching.h"

#include <EventGui.h>

#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graphParser.h>
#include <Rcs_macros.h>
#include <Rcs_shape.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_utilsCPP.h>
#include <ForceDragger.h>

#include <CmdLineWidget.h>

#include <GraphNode.h>
#include <PhysicsNode.h>

#include <fstream>
#include <iostream>
#include <thread>



namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
static void onSetLogLevel(int dl)
{
  RcsLogLevel = dl;
}

/*******************************************************************************
 *
 ******************************************************************************/
class NamedBodyForceDragger : public Rcs::MouseDragger
{
public:

  NamedBodyForceDragger(Rcs::PhysicsBase* sim_) : Rcs::MouseDragger(), sim(sim_)
  {
  }

  virtual void update()
  {

    double I_mouseTip[3], I_bodyAnchor[3], k_bodyAnchor[3];
    bool leftMouseButtonPressed, rightMouseButtonPressed, leftShiftPressed, leftCtrlPressed;

    const RcsBody* bdy = Rcs::MouseDragger::getDragData(I_mouseTip, I_bodyAnchor, k_bodyAnchor,
                                                        &leftMouseButtonPressed,
                                                        &rightMouseButtonPressed,
                                                        &leftShiftPressed,
                                                        &leftCtrlPressed,
                                                        false);

    if (bdy)
    {
      HTr A_BI = bdy->A_BI;
      A_BI.org[0] = I_mouseTip[0];
      A_BI.org[1] = I_mouseTip[1];
      sim->applyTransform(bdy, &A_BI);
    }

  }

  Rcs::PhysicsBase* sim;
};

/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleFlowMatching, "Actions", "FlowMatching");

ExampleFlowMatching::ExampleFlowMatching() : ExampleFlowMatching(0, NULL)
{
}

ExampleFlowMatching::ExampleFlowMatching(int argc, char** argv) : ExampleBase(argc, argv)
{
  dt = 0.001;
  dt_max = 0.0;
  loopCount = 0;

  updateGraph = NULL;
  computeKinematics = NULL;
  setRenderCommand = NULL;
}

ExampleFlowMatching::~ExampleFlowMatching()
{
  stop();
  Rcs_removeResourcePath(configDirectory.c_str());
}

bool ExampleFlowMatching::initParameters()
{
  xmlFileName = "g_tbar.xml";
  configDirectory = "../src/Smile/src/AffAction/config/xml/flowmatching";

  return true;
}

bool ExampleFlowMatching::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  parser->getArgument("-dt", &dt, "Time step (default is %f)", dt);
  parser->getArgument("-f", &xmlFileName, "Configuration file name "
                      "(default is %s)", xmlFileName.c_str());
  parser->getArgument("-dir", &configDirectory, "Configuration file directory "
                      "(default is %s)", configDirectory.c_str());

  if (parser->hasArgument("-h"))
  {
    std::cout << help() << std::endl;
    return false;
  }

  return true;
}

bool ExampleFlowMatching::initAlgo()
{
  Rcs_addResourcePath(RCS_CONFIG_DIR);
  Rcs_addResourcePath(configDirectory.c_str());

  entity.subscribe("SetLogLevel", &onSetLogLevel);
  entity.subscribe("Quit", &ExampleFlowMatching::onQuit, this);

  entity.setDt(dt);
  updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  setRenderCommand = entity.registerEvent<>("Render");

  RcsGraph* graph = RcsGraph_create(xmlFileName.c_str());
  RCHECK(RcsGraph_check(graph, NULL, NULL));

  // Graph component contains "sensed" graph
  graphC = std::make_unique<aff::GraphComponent>(&entity, graph);
  graphC->setEnableRender(false);
  physicsC = std::make_unique<aff::PhysicsComponent>(&entity, graph, "Bullet", "physics.xml");

  // Initialization sequence to initialize all graphs from the sensory state. This also triggers the
  // "Start" event, starting all component threads.
  entity.initialize(graphC->getGraph());

  RLOG_CPP(1, help());

  RcsGraph_destroy(graph);

  return true;
}

bool ExampleFlowMatching::initGraphics()
{
  viewer = std::make_unique<aff::GraphicsWindow>(&entity, true, true);
  osg::ref_ptr<Rcs::PhysicsNode> pn = new Rcs::PhysicsNode();
  pn->setSimulation(physicsC->getPhysics());
  pn->init(false);   // without force dragger, we use our own
  pn->addChild(new NamedBodyForceDragger(physicsC->getPhysics()));
  viewer->add(pn.get());
  viewer->setTitle("ExampleFlowMatching");

  // Check if there is a body named 'initial_camera_view'.
  double q_cam[6];
  VecNd_set6(q_cam, -2.726404, -2.515165, 3.447799,   -0.390124, 0.543041, 0.795294);

  const RcsBody* camera_body = RcsGraph_getBodyByName(graphC->getGraph(), "default_camera_view");
  if (camera_body)
  {
    RLOG(1, "Setting initial view based on body 'initial_camera_view'.");
    HTr_to6DVector(q_cam, &camera_body->A_BI);
  }

  viewer->setCameraTransform(q_cam[0], q_cam[1], q_cam[2], q_cam[3], q_cam[4], q_cam[5]);

#if defined(_MSC_VER)
  viewer->setWindowSize(12, 36, 640, 480);
#else
  viewer->setWindowSize(0, 0, 640, 480);
#endif

  viewer->setKeyCallback('e', [this](char k)
  {
    new aff::EventGui(&entity);
  }, "Launch event gui");

  viewer->setKeyCallback(' ', [this](char k)
  {
    entity.call("TogglePause");

  }, "Toggle pause modus");

  viewer->setKeyCallback('X', [this](char k)
  {
    RcsGraph_writeDotFile(graphC->getGraph(), "RcsGraph.dot");
    std::string dottyCommand = "dotty " + std::string("RcsGraph.dot") + "&";
    int err = system(dottyCommand.c_str());
    if (err == -1)
    {
      RLOG(1, "Couldn't start dotty tool");
    }

  }, "Show dot file");


  vcamC = std::make_unique<VirtualCameraComponent>(&entity);
  vcamC->setSceneData(new Rcs::GraphNode(graphC->getGraph()));
  vcamC->setCameraTransform(q_cam);

  return true;
}

void ExampleFlowMatching::run()
{
  // Start all threads of components. This has already been published during
  // the entitie's initialize() method in the initAlgo() method. This Start
  // event takes carea about all components that have been added later.
  entity.publish("Start");
  entity.process();

  while (runLoop)
  {
    step();
  }

  // The runLoop is ended with ExampleBase::stop(). We still need to call each
  // component's stop event.
  entity.publish("Stop");
  entity.process();
}

void ExampleFlowMatching::step()
{
  double dtProcess = Timer_getSystemTime();

  entity.call<RcsGraph*>("UpdateGraph", graphC->getGraph());
  entity.call<RcsGraph*>("ComputeKinematics", graphC->getGraph());
  // updateGraph->call(graphC->getGraph());
  // computeKinematics->call(graphC->getGraph());
  //setRenderCommand->call();

  if (loopCount%10==0)
  {
    entity.call("Render");
  }

  if (loopCount%30==0)
  {
    entity.call("Capture");
  }
  entity.process();
  entity.stepTime();

  dtProcess = Timer_getSystemTime() - dtProcess;

  if (entity.getTime() > 3.0)
  {
    dt_max = std::max(dt_max, dtProcess);
  }


  char timeStr[256];
  snprintf(timeStr, 256, "Time: %.3f   dt: %.1f dt_max: %.1f msec\n"
           "queue: %zu (max: %zu)",
           entity.getTime(), dtProcess * 1.0e3, dt_max * 1.0e3,
           entity.queueSize(), entity.getMaxQueueSize());
  entity.publish("SetTextLine", std::string(timeStr), 0);

  Timer_waitDT(entity.getDt() - dtProcess);

  loopCount++;
  RLOG_CPP(6, "Loop end: queue size is " << entity.queueSize());
}

void ExampleFlowMatching::onQuit()
{
  entity.publish("Stop");
  runLoop = false;
}

}   // namespace aff
