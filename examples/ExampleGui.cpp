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

#include "ExampleGui.h"
#include "HardwareComponent.h"
#include "SceneJsonHelpers.h"
#include "ComponentFactory.h"
#include "JointGuiComponent.h"

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
RCS_REGISTER_EXAMPLE(ExampleGui, "Gui", "Joint Gui");

ExampleGui::ExampleGui() : ExampleGui(0, NULL)
{
}

ExampleGui::ExampleGui(int argc, char** argv) :
  ExampleBase(argc, argv), entity(), graphToInitializeWith(NULL)
{
  dt = 0.01;
  dt_max = 0.0;
  dt_max2 = 0.0;
  speedUp = 1;
  loopCount = 0;
  blockingMainThread = false;
  enableWireframeToggle = true;   // Show wireframe if collisions are deactivated
  enableRealGraphVisualization = false;

  pause = false;
  withRobot = false;
  dtProcess = 0.0;
  dtEvents = 0.0;

  updateGraph = nullptr;
  postUpdateGraph = nullptr;
  computeKinematics = nullptr;
  setJointCommand = nullptr;
  setRenderCommand = nullptr;

  viewer = nullptr;
  graphC = nullptr;
}

ExampleGui::~ExampleGui()
{
  stop();

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    RLOG_CPP(5, "Deleting hardware component " << i);
    delete hwc[i];
  }

  for (size_t i = 0; i < components.size(); ++i)
  {
    RLOG_CPP(5, "Deleting component " << i << ": " << components[i]->getName());
    delete components[i];
  }

  Rcs_removeResourcePath(configDirectory.c_str());
  RcsGraph_destroy(graphToInitializeWith);
  RLOG_CPP(5, "Done deleting ExampleGui");
}

bool ExampleGui::initParameters()
{
  xmlFileName = "g_attentive_support.xml";
  configDirectory = "config/xml/examples";

  return true;
}

bool ExampleGui::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  parser->getArgument("-speedUp", &speedUp, "Speed-up factor (default: %d)", speedUp);
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

bool ExampleGui::initAlgo()
{
  Rcs_addResourcePath(RCS_CONFIG_DIR);
  Rcs_addResourcePath(configDirectory.c_str());

  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("SetLogLevel", &onSetLogLevel);
  entity.subscribe("Quit", &ExampleGui::onQuit, this);
  entity.subscribe("Print", &ExampleGui::onPrint, this);
  entity.subscribe("Process", &ExampleGui::onProcess, this);

  entity.setDt(dt);
  updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  setRenderCommand = entity.registerEvent<>("Render");
  postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");

  if (pause)
  {
    entity.call("TogglePause");
  }

  if (!controller)
  {
    RcsGraph* graph = RcsGraph_create(xmlFileName.c_str());
    RCHECK(RcsGraph_check(graph, NULL, NULL));
    controller = std::make_unique<Rcs::ControllerBase>(graph);
  }

  // Graph component contains "sensed" graph
  graphC = new aff::GraphComponent(&entity, getGraph());
  graphC->setEnableRender(false);
  addComponent(graphC);

  // Remember the state for re-initialization
  graphToInitializeWith = RcsGraph_clone(getGraph());

  // Initialize robot components from command line and componentArgs
  auto cTmp = createHardwareComponents(entity, getGraph(), nullptr, false, componentArgs);
  this->hwc.insert(hwc.end(), cTmp.begin(), cTmp.end());
  cTmp = createComponents(entity, getGraph(), nullptr, false, componentArgs);
  this->components.insert(components.end(), cTmp.begin(), cTmp.end());

  if (!hwc.empty())
  {
    setEnableRobot(true);
  }

  // Initialization sequence to initialize all graphs from the sensory state. This also triggers the
  // "Start" event, starting all component threads.
  entity.initialize(getCurrentGraph());

  return true;
}



bool ExampleGui::initGraphics()
{
  if (viewer)
  {
    RLOG(1, "Graphics already initialized");
    return false;
  }

  auto syncMode = blockingMainThread ? GraphicsWindow::SyncMode::External : GraphicsWindow::SyncMode::Threaded;
  viewer = new GraphicsWindow(&entity, syncMode);
  addComponent(viewer);

  viewer->setTitle("ExampleGui");

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
    getEntity().publish("Quit");
  }, "Quit");

  viewer->setKeyCallback('e', [this](char k)
  {
    auto ew = new aff::EventWidget(&entity);
    ew->show();
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
        RLOG(0, "Showing both (Real is solid)");
        getEntity().publish("RenderCommand", std::string("Physics"),
                            std::string("show"));
        getEntity().publish("RenderCommand", std::string("IK"),
                            std::string("show"));
        getEntity().publish("RenderCommand", std::string("IK"),
                            std::string("setGhostMode"));
        break;

      case 1:
        RLOG(0, "Showing IK");
        getEntity().publish("RenderCommand", std::string("Physics"),
                            std::string("hide"));
        getEntity().publish("RenderCommand", std::string("IK"),
                            std::string("show"));
        getEntity().publish("RenderCommand", std::string("IK"),
                            std::string("unsetGhostMode"));
        break;

      case 2:
        RLOG(0, "Showing Real");
        getEntity().publish("RenderCommand", std::string("Physics"),
                            std::string("show"));
        getEntity().publish("RenderCommand", std::string("IK"),
                            std::string("hide"));
        break;
    }

  }, "Toggle GraphicsWindow display");

  entity.publish("RenderCommand", std::string("ShowLines"), std::string("false"));
  entity.publish("RenderCommand", std::string("Physics"), std::string("hide"));
  entity.publish("RenderCommand", std::string("IK"), std::string("show"));
  entity.publish("RenderCommand", std::string("IK"), std::string("unsetGhostMode"));
  entity.process();

  // Show the graph of the GraphComponent (updated from hardware)
  if (enableRealGraphVisualization)
  {
    graphC->setEnableRender(true);
    entity.publish<std::string, const RcsGraph*>("RenderGraph", "Physics", getCurrentGraph());
    entity.publish<std::string, const RcsGraph*>("RenderGraph", "IK", getGraph());
    entity.process();
    Timer_waitDT(0.5);
    entity.publish("RenderCommand", std::string("Physics"), std::string("show"));
    entity.publish("RenderCommand", std::string("IK"), std::string("show"));
    getEntity().publish("RenderCommand", std::string("IK"), std::string("setGhostMode"));
    entity.process();
  }

  return true;
}

bool ExampleGui::initGuis()
{
  addComponent(new JointGuiComponent(&entity, getGraph(), 0.2));

  return true;
}

void ExampleGui::run()
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

void ExampleGui::step()
{
  dtProcess = Timer_getSystemTime();

  stepMtx.lock();
  updateGraph->call(getCurrentGraph());
  computeKinematics->call(getCurrentGraph());
  postUpdateGraph->call(getGraph(), getCurrentGraph());

  auto jgcs = getComponents<JointGuiComponent>(components);
  RCHECK(jgcs.size()==1);

  REXEC(1)
  {
    MatNd_printCommentDigits("gui", jgcs[0]->getJointCommandPtr(), 5);
  }

  setJointCommand->call(jgcs[0]->getJointCommandPtr());
  RcsGraph_setState(getGraph(), jgcs[0]->getJointCommandPtr(), NULL);

  setRenderCommand->call();
  dtEvents = Timer_getSystemTime() - dtProcess;
  entity.process();
  entity.stepTime();
  stepMtx.unlock();

  dtProcess = Timer_getSystemTime() - dtProcess;


  if (entity.getTime() > 3.0)
  {
    dt_max = std::max(dt_max, dtProcess);
    dt_max2 = std::max(dt_max2, dtEvents);
  }

  loopCount++;

  char timeStr[256];
  snprintf(timeStr, 256, "[Step joints] Time: %.3f   dt: %.1f dt_max: %.1f %.1f msec\n"
           "queue: %zu (max: %zu)",
           entity.getTime(), dtProcess * 1.0e3, dt_max * 1.0e3, dt_max2 * 1.0e3,
           entity.queueSize(), entity.getMaxQueueSize());
  entity.publish("SetTextLine", std::string(timeStr), 0);

  if (loopCount % speedUp == 0)
  {
    Timer_waitDT(entity.getDt() - dtProcess);
  }

  RLOG(6, "Loop end %d", loopCount - 1);
  RLOG_CPP(6, "Loop end: queue size is " << entity.queueSize());
}

std::string ExampleGui::help()
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

void ExampleGui::onQuit()
{
  entity.publish("Stop");
  runLoop = false;
}

/*******************************************************************************
 * Builds a search tree, finds solutions, and handles events.
 ******************************************************************************/

void ExampleGui::onPrint()
{
  std::cout << help();
}


void ExampleGui::setEnableRobot(bool enable)
{
  RMSG("***** Real Robot in the loop - speedUp resetted to 1 *****");
  speedUp = 1;
  withRobot = enable;
}

void ExampleGui::addComponent(ComponentBase* component)
{
  if (component)
  {
    components.push_back(component);
  }
}

bool ExampleGui::eraseComponent(ComponentBase* component)
{
  bool success = false;

  for (auto it = components.begin(); it != components.end(); ++it)
  {
    if (component && ((*it)==component))
    {
      delete *it;
      components.erase(it);
      success = true;
      break;
    }
  }

  return success;
}

void ExampleGui::addHardwareComponent(ComponentBase* component)
{
  if (component)
  {
    hwc.push_back(component);
    setEnableRobot(true);
  }
}

bool ExampleGui::getRobotEnabled() const
{
  return withRobot;
}

RcsGraph* ExampleGui::getGraph()
{
  return controller ? controller->getGraph() : nullptr;
}

const RcsGraph* ExampleGui::getGraph() const
{
  return controller ? controller->getGraph() : nullptr;
}

RcsGraph* ExampleGui::getCurrentGraph()
{
  return graphC ? graphC->getGraph() : nullptr;
}

const RcsGraph* ExampleGui::getCurrentGraph() const
{
  return graphC ? graphC->getGraph() : nullptr;
}

GraphicsWindow* ExampleGui::getViewer()
{
  return viewer;
}

bool ExampleGui::eraseViewer()
{
  bool success = eraseComponent(viewer);
  viewer = nullptr;
  return success;
}

const EntityBase& ExampleGui::getEntity() const
{
  return entity;
}

EntityBase& ExampleGui::getEntity()
{
  return entity;
}

void ExampleGui::startThreaded()
{
  std::thread t1([&]
  {
    ExampleBase::start();

    RLOG(0, "ExampleGui thread says good bye");
  });
  t1.detach();
}

void ExampleGui::onProcess()
{
  entity.process();
}

void ExampleGui::lockStepMtx() const
{
  stepMtx.lock();
}

void ExampleGui::unlockStepMtx() const
{
  stepMtx.unlock();
}

void ExampleGui::addComponentArgument(const std::string& arg)
{
  componentArgs += " " + arg;
}

std::string ExampleGui::getComponentArguments() const
{
  return componentArgs;
}

const std::vector<ComponentBase*>& ExampleGui::getComponentsRef() const
{
  return components;
}

void ExampleGui::updateUI()
{
  if (!getViewer())
  {
    return;
  }

  getViewer()->frame();
  handleKeys();
}

void ExampleGui::setSyncMode(std::string syncMode)
{
  ExampleBase::setSyncMode(syncMode);

  if (syncMode=="External")
  {
    blockingMainThread = true;
  }
}



}   // namespace aff
