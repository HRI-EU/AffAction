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

#include "ExampleJointControl.h"
#include "HardwareComponent.h"
#include "SceneJsonHelpers.h"
#include "ComponentFactory.h"

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
#include <Rcs_graphicsUtils.h>

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
ExampleJointControl::ExampleJointControl() : ExampleJointControl(0, NULL)
{
}

ExampleJointControl::ExampleJointControl(int argc, char** argv) : ExampleBase(argc, argv), entity()
{
}

ExampleJointControl::~ExampleJointControl()
{
  stop();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    RLOG_CPP(0*5, "Deleting hardware component " << i);
    delete hwc[i];
  }

  for (size_t i = 0; i < components.size(); ++i)
  {
    RLOG_CPP(0*5, "Deleting component " << i << ": " << components[i]->getName());
    delete components[i];
  }

  Rcs_removeResourcePath(configDirectory.c_str());
  RLOG_CPP(5, "Done deleting ExampleJointControl");
}

bool ExampleJointControl::initParameters()
{
  xmlFileName = "g_attentive_support.xml";
  configDirectory = "config/xml/examples";

  return true;
}

bool ExampleJointControl::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  parser->getArgument("-dt", &dt, "Time step (default is %f)", dt);
  parser->getArgument("-f", &xmlFileName, "Configuration file name "
                      "(default is %s)", xmlFileName.c_str());
  parser->getArgument("-dir", &configDirectory, "Configuration file directory "
                      "(default is %s)", configDirectory.c_str());

  // This is just for pupulating the parsed command line arguments for the help
  // functions / help window.
  const bool dryRun = true;
  createHardwareComponents(entity, NULL, NULL, dryRun, componentArgs);
  createComponents(entity, NULL, NULL, dryRun, componentArgs);

  if (parser->hasArgument("-h"))
  {
    std::cout << help() << std::endl;
    return false;
  }

  return true;
}

bool ExampleJointControl::initAlgo()
{
  Rcs_addResourcePath(RCS_CONFIG_DIR);
  Rcs_addResourcePath(configDirectory.c_str());

  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("SetLogLevel", &onSetLogLevel);
  entity.subscribe("Quit", &ExampleJointControl::onQuit, this);
  entity.subscribe("Print", &ExampleJointControl::onPrint, this);

  entity.setDt(dt);
  updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  setRenderCommand = entity.registerEvent<>("Render");
  postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");

  RcsGraph* graph = RcsGraph_create(xmlFileName.c_str());
  RCHECK(RcsGraph_check(graph, NULL, NULL));
  controller = std::make_unique<Rcs::ControllerBase>(graph);

  // Graph component contains "sensed" graph
  graphC = new aff::GraphComponent(&entity, getGraph());
  graphC->setEnableRender(false);
  components.push_back(graphC);

  // Initialize robot components from command line and componentArgs
  auto cTmp = createHardwareComponents(entity, getGraph(), nullptr, false, componentArgs);
  this->hwc.insert(hwc.end(), cTmp.begin(), cTmp.end());
  cTmp = createComponents(entity, getGraph(), nullptr, false, componentArgs);
  this->components.insert(components.end(), cTmp.begin(), cTmp.end());

  RLOG(0, "Finished initialize");
  return true;
}



bool ExampleJointControl::initGraphics()
{
  auto syncMode = blockingMainThread ? GraphicsWindow::SyncMode::External : GraphicsWindow::SyncMode::Threaded;
  viewer = new GraphicsWindow(&entity, syncMode);
  components.push_back(viewer);

  viewer->setTitle("ExampleJointControl");

  // Apply default camera view, or the transform of a body named 'initial_camera_view'.
  double q_cam[6];
  VecNd_set6(q_cam, -2.7, -2.5, 3.4, -0.4, 0.5, 0.8);

  const RcsBody* camera_body = RcsGraph_getBodyByName(getCurrentGraph(), "default_camera_view");
  if (camera_body)
  {
    RLOG(5, "Setting initial view based on body 'initial_camera_view'.");
    HTr_to6DVector(q_cam, &camera_body->A_BI);
  }

  viewer->setCameraTransform(q_cam[0], q_cam[1], q_cam[2], q_cam[3], q_cam[4], q_cam[5]);

#if defined(_MSC_VER)
  viewer->setWindowSize(12, 36, 1000, 750);
#else
  viewer->setWindowSize(0, 0, 1000, 750);
#endif

  viewer->setKeyCallback('q', [this](char k)
  {
    RLOG(0, "Quitting");
    entity.publish("Quit");
  }, "Quit");

  viewer->setKeyCallback('e', [this](char k)
  {
    if (blockingMainThread)
    {
      auto ew = new aff::EventWidget(&entity);
      ew->show();
    }
    else
    {
      new aff::EventGui(&entity);
    }
  }, "Launch event gui");

  viewer->setKeyCallback('x', [this](char k)
  {
    static int viewMode = 0;
    viewMode++;
    if (viewMode>2)
    {
      viewMode = 0;
    }

    switch (viewMode)
    {
      case 0:
        entity.publish("RenderCommand", std::string("Physics"), std::string("show"));
        entity.publish("RenderCommand", std::string("Gui"), std::string("show"));
        entity.publish("RenderCommand", std::string("Gui"), std::string("setGhostMode"));
        renderStringHUD = "Showing commands (shadow) and sensory state (solid)";
        break;

      case 1:
        entity.publish("RenderCommand", std::string("Physics"), std::string("hide"));
        entity.publish("RenderCommand", std::string("Gui"), std::string("show"));
        entity.publish("RenderCommand", std::string("Gui"), std::string("unsetGhostMode"));
        renderStringHUD = "Showing desired commands";
        break;

      case 2:
        entity.publish("RenderCommand", std::string("Physics"), std::string("show"));
        entity.publish("RenderCommand", std::string("Gui"), std::string("hide"));
        renderStringHUD = "Showing sensory state";
        break;
    }

  }, "Toggle GraphicsWindow display");

  entity.publish("RenderCommand", std::string("ShowLines"), std::string("false"));
  entity.publish("RenderCommand", std::string("Physics"), std::string("hide"));
  entity.publish("RenderCommand", std::string("Gui"), std::string("show"));
  entity.publish("RenderCommand", std::string("Gui"), std::string("unsetGhostMode"));
  entity.process();

  // Show the graph of the GraphComponent (updated from hardware)
  renderStringHUD = "Showing commands (shadow) and sensory state (solid)";
  graphC->setEnableRender(true);
  entity.publish<std::string, const RcsGraph*>("RenderGraph", "Physics", getCurrentGraph());
  entity.publish<std::string, const RcsGraph*>("RenderGraph", "Gui", getGraph());
  entity.process();
  Timer_waitDT(0.5);
  entity.publish("RenderCommand", std::string("Physics"), std::string("show"));
  entity.publish("RenderCommand", std::string("Gui"), std::string("show"));
  entity.publish("RenderCommand", std::string("Gui"), std::string("setGhostMode"));
  entity.process();

  return true;
}

bool ExampleJointControl::initGuis()
{
  jguiC = new JointGuiComponent(&entity, getGraph(), tmc);
  components.push_back(jguiC);
  return true;
}

void ExampleJointControl::run()
{
  // Initialization sequence to initialize all graphs from the sensory state.
  // This also triggers the "Start" event, starting all component threads.
  entity.initialize(getCurrentGraph());

  while (runLoop)
  {
    step();
  }

  // The runLoop is ended with ExampleBase::stop(). We still need to call each
  // component's stop event.
  entity.publish("Stop");
  entity.process();
}

void ExampleJointControl::step()
{
  dtProcess = Timer_getSystemTime();

  updateGraph->call(getCurrentGraph());
  computeKinematics->call(getCurrentGraph());
  postUpdateGraph->call(getGraph(), getCurrentGraph());
  setJointCommand->call(jguiC->getJointCommandPtr());
  setRenderCommand->call();
  entity.process();
  entity.stepTime();

  dtProcess = Timer_getSystemTime() - dtProcess;

  if (entity.getTime() > 3.0)
  {
    dt_max = std::max(dt_max, dtProcess);
  }

  loopCount++;

  char timeStr[256];
  snprintf(timeStr, 256, "Time: %.3f   dt: %.1f dt_max: %.1f msec\n"
           "queue: %zu (max: %zu)\n%s",
           entity.getTime(), dtProcess * 1.0e3, dt_max * 1.0e3,
           entity.queueSize(), entity.getMaxQueueSize(), renderStringHUD.c_str());
  entity.publish("SetTextLine", std::string(timeStr), 0);

  Timer_waitDT(entity.getDt() - dtProcess);

  RLOG(6, "Loop end %d", loopCount - 1);
  RLOG_CPP(6, "Loop end: queue size is " << entity.queueSize());
}

std::string ExampleJointControl::help()
{
  std::stringstream s;

  s << ExampleBase::help();
  s << Rcs::RcsGraph_printUsageToString(xmlFileName);
  s << Rcs::RcsShape_distanceFunctionsToString();
  s << std::endl << "Hardware concurrency: " << std::thread::hardware_concurrency() << std::endl;
  if (getGraph())
  {
    s << "Graph size[bytes]: " << RcsGraph_sizeInBytes(getGraph()) << std::endl;
  }
  s << "Current working directory: " << Rcs::File_getCurrentWorkingDir() << std::endl;
  s << AffordanceEntity::printAffordanceCapabilityMatches();

  s << std::endl << components.size() << " components:" << std::endl;
  for (size_t i = 0; i < components.size(); ++i)
  {
    s << "Component " << i << ": '" << components[i]->getName() << "'" << std::endl;
  }
  s << std::endl << hwc.size() << " hardware components:" << std::endl;
  for (size_t i = 0; i < hwc.size(); ++i)
  {
    s << "Hardware component " << i << ": '" << hwc[i]->getName() << "'" << std::endl;
  }

  return s.str();
}

void ExampleJointControl::onQuit()
{
  entity.publish("Stop");
  runLoop = false;
}

/*******************************************************************************
 * Builds a search tree, finds solutions, and handles events.
 ******************************************************************************/

void ExampleJointControl::onPrint()
{
  std::cout << help();
}

RcsGraph* ExampleJointControl::getGraph()
{
  return controller ? controller->getGraph() : nullptr;
}

const RcsGraph* ExampleJointControl::getGraph() const
{
  return controller ? controller->getGraph() : nullptr;
}

RcsGraph* ExampleJointControl::getCurrentGraph()
{
  return graphC ? graphC->getGraph() : nullptr;
}

const RcsGraph* ExampleJointControl::getCurrentGraph() const
{
  return graphC ? graphC->getGraph() : nullptr;
}

void ExampleJointControl::startThreaded()
{
  std::thread t1([&]
  {
    ExampleBase::start();

    RLOG(0, "ExampleJointControl thread says good bye");
  });
  t1.detach();
}

void ExampleJointControl::updateUI()
{
  if (!viewer)
  {
    return;
  }

  viewer->frame();
  handleKeys();
}

void ExampleJointControl::setSyncMode(std::string syncMode)
{
  ExampleBase::setSyncMode(syncMode);

  if (syncMode=="External")
  {
    blockingMainThread = true;
  }
}

void ExampleJointControl::addComponentArgument(const std::string& arg)
{
  componentArgs += " " + arg;
}


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleAllegroGui : public ExampleJointControl
{
public:

  ExampleAllegroGui(int argc, char** argv) : ExampleJointControl(argc, argv)
  {
    RMSG("Start bin/AllegroDriver -m 1");
  }

  virtual ~ExampleAllegroGui() = default;

  bool initParameters()
  {
    ExampleJointControl::initParameters();
    xmlFileName = "g_robo.xml";
    configDirectory = "config/xml/Allegro";
    addComponentArgument("-allegroZmq_right");
    return true;
  }

};

RCS_REGISTER_EXAMPLE(ExampleAllegroGui, "RoboDrivers", "Allegro right Joint-Gui");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleJacoGen2_6 : public ExampleJointControl
{
public:

  ExampleJacoGen2_6(int argc, char** argv) : ExampleJointControl(argc, argv)
  {
  }

  virtual ~ExampleJacoGen2_6()
  {
  }

  bool initParameters()
  {
    ExampleJointControl::initParameters();
    xmlFileName = "g_kinova_ulw2_6dof.xml";
    configDirectory = "config/xml/TwoArmJaco7";
    addComponentArgument("-jacoGen2_6_Zmq");
    return true;
  }

  std::string help()
  {
    std::string str = "Start bin/JacoEthernetDriver -m 1 -robo_name wasabi -rt\n\n";
    str += ExampleJointControl::help();
    return str;
  }

};

RCS_REGISTER_EXAMPLE(ExampleJacoGen2_6, "RoboDrivers", "Jaco 6 Joint-Gui (wasabi)");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleFrankaRightGui : public ExampleJointControl
{
public:

  ExampleFrankaRightGui(int argc, char** argv) : ExampleJointControl(argc, argv)
  {
  }

  virtual ~ExampleFrankaRightGui()
  {
  }

  bool initParameters()
  {
    ExampleJointControl::initParameters();
    //xmlFileName = "g_robo_tablemount.xml";
    xmlFileName = "g_franka_duomount.xml";
    configDirectory = "config/xml/Franka";
    componentArgs = "-frankaZmq_right ";
    return true;
  }

  std::string help()
  {
    std::string str = "Start bin/FrankaDriver -m 1 -robo_name franka_right\n\n";
    str += ExampleJointControl::help();
    return str;
  }

};

RCS_REGISTER_EXAMPLE(ExampleFrankaRightGui, "RoboDrivers", "Franka right with Joint-Gui");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleFrankaLeftGui : public ExampleJointControl
{
public:

  ExampleFrankaLeftGui(int argc, char** argv) : ExampleJointControl(argc, argv)
  {
  }

  virtual ~ExampleFrankaLeftGui()
  {
  }

  bool initParameters()
  {
    ExampleJointControl::initParameters();
    //xmlFileName = "g_robo_left_tablemount.xml";
    xmlFileName = "g_franka_duomount.xml";
    configDirectory = "config/xml/Franka";
    componentArgs = "-frankaZmq_left ";
    return true;
  }

  std::string help()
  {
    std::string str = "Start bin/FrankaDriver -m 1 -robo_name franka_left\n\n";
    str += ExampleJointControl::help();
    return str;
  }

};

RCS_REGISTER_EXAMPLE(ExampleFrankaLeftGui, "RoboDrivers", "Franka left with Joint-Gui");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleFrankaBimanualGui : public ExampleJointControl
{
public:

  ExampleFrankaBimanualGui(int argc, char** argv) : ExampleJointControl(argc, argv)
  {
  }

  virtual ~ExampleFrankaBimanualGui()
  {
  }

  bool initParameters()
  {
    ExampleJointControl::initParameters();
    //xmlFileName = "g_robo_bimanual_tablemount.xml";
    xmlFileName = "g_franka_duomount.xml";
    configDirectory = "config/xml/Franka";
    componentArgs = "-frankaZmq_left -frankaZmq_right ";
    return true;
  }

  std::string help()
  {
    std::string str = "Start:\n";
    str = "  bin/FrankaDriver -m 1 -robo_name laplace\n";
    str = "  bin/FrankaDriver -m 1 -robo_name riemann\n\n";
    str += ExampleJointControl::help();
    return str;
  }

};

RCS_REGISTER_EXAMPLE(ExampleFrankaBimanualGui, "RoboDrivers", "Franka both arms with Joint-Gui");



}   // namespace aff
