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

#include "ExampleTeleOp.h"
#include "HardwareComponent.h"
#include "SceneJsonHelpers.h"
#include "ComponentFactory.h"
#include "LandmarkBase.h"

#include <EventGui.h>

#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graphParser.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_utilsCPP.h>
#include <AABBNode.h>

#include <fstream>
#include <iostream>
#include <thread>


namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
ExampleTeleOp::ExampleTeleOp() : ExampleTeleOp(0, NULL)
{
}

ExampleTeleOp::ExampleTeleOp(int argc, char** argv) : ExampleBase(argc, argv)
{
}

ExampleTeleOp::~ExampleTeleOp()
{
  cleanup();
}

void ExampleTeleOp::cleanup()
{
  stop();

  while (runFunctionRunning)
  {
    Timer_waitDT(0.1);
  }

  RLOG(1, "Run function finished");

  for (size_t i = 0; i < hwc.size(); ++i)
  {
    RLOG_CPP(1, "Deleting hardware component " << i);
    delete hwc[i];
  }
  hwc.clear();

  for (size_t i = 0; i < components.size(); ++i)
  {
    RLOG_CPP(1, "Deleting component " << i << ": " << components[i]->getName());
    delete components[i];
  }
  components.clear();

  Rcs_removeResourcePath(configDirectory.c_str());

  this->viewer = nullptr;
  this->graphC = nullptr;
  this->ikc = nullptr;

  RLOG_CPP(1, "Done deleting ExampleActionsECS");
}

bool ExampleTeleOp::initParameters()
{
  xmlFileName = "c_franka_teleop_right.xml";
  configDirectory = "config/xml/Franka";
  HTr_setIdentity(&eeTrf);

  return true;
}

bool ExampleTeleOp::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  parser->getArgument("-dt", &dt, "Time step (default is %f)", dt);
  parser->getArgument("-f", &xmlFileName, "Configuration file name "
                      "(default is %s)", xmlFileName.c_str());
  parser->getArgument("-dir", &configDirectory, "Configuration file directory "
                      "(default is %s)", configDirectory.c_str());
  parser->getArgument("-noGraphics", &noGraphics, "Run without graphics window (default: off)");
  parser->getArgument("-noLimits", &noLimits, "Run without limits(default: off)");
  parser->getArgument("-alpha", &alpha, "Null space scaling (default is %f)", alpha);
  parser->getArgument("-lambda", &lambda, "Regularization (default is %f)", lambda);
  parser->getArgument("-withRobo", &withRobo, "Use robot components");

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

bool ExampleTeleOp::initAlgo()
{
  std::string cfgDir = build_path + std::string(RCS_CONFIG_DIR);
  std::string xmlDir = build_path + configDirectory;
  Rcs_addResourcePath(cfgDir.c_str());
  Rcs_addResourcePath(xmlDir.c_str());

  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &ExampleTeleOp::onQuit, this);
  entity.subscribe("Print", &ExampleTeleOp::onPrint, this);
  entity.subscribe("SetTwist", &ExampleTeleOp::onSetTwist, this);
  entity.subscribe("SetWrench", &ExampleTeleOp::onSetWrench, this);
  entity.subscribe("SetFingerPose", &ExampleTeleOp::onSetFingerPose, this);
  entity.subscribe("SetBiManualPoseCommand", &ExampleTeleOp::onSetBiManualPoseCommand, this);
  entity.subscribe("PostUpdateGraph", &ExampleTeleOp::onPostUpdateGraph, this);
  entity.subscribe("PrintCollectedData", &ExampleTeleOp::printCollectedData, this);


  entity.setDt(dt);
  updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  setTwist = entity.registerEvent<double,double,double,double,double,double,bool>("SetTwist");
  setJointCommand = entity.registerEvent<const MatNd*>("SetJointCommand");
  setRenderCommand = entity.registerEvent<>("Render");
  postUpdateGraph = entity.registerEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph");

  this->controller = std::make_unique<Rcs::ControllerBase>(xmlFileName.c_str());

  // Get name of end effector
  RCHECK(controller->getNumberOfTasks()>0);
  if (controller->getTask(0)->getEffector())
  {
  this->endEffectorName = std::string(controller->getTask(0)->getEffector()->name);
  }

  const RcsBody* ee = RcsGraph_getBodyByName(getGraph(), this->endEffectorName.c_str());
  if (ee)
  {
  HTr_copy(&eeTrf, &ee->A_BI);
  }

  // Extract the collision model
  {
    xmlDocPtr doc = nullptr;
    xmlNodePtr node = parseXMLFile(getGraph()->cfgFile, nullptr, &doc);
    if (node)
    {
      xmlNodePtr child = getXMLChildByName(node, "BroadPhase");
      if (child)
      {
        RcsBroadPhase* bp = RcsBroadPhase_createFromXML(getGraph(), child);
        RcsBroadPhase_updateBoundingVolumes(bp);
        controller->setBroadPhase(bp);
        controller->setNarrowPhase(RcsCollisionModel_create(getGraph()));
      }
      else
      {
        RFATAL("Currently we require a broadphase model - none found in graph configuration file");
      }

      xmlFreeDoc(doc);
    }
    else
    {
      RLOG(0, "Failed to read xml file \"%s\"", xmlFileName.c_str());
    }

  }



  this->modelStates = Rcs::RcsGraph_getModelStates(getGraph());

  // Initialize the scene (for agent model only)
  if (withScene)
  {
    std::string sceneFile = Rcs::getAbsoluteFileName(xmlFileName);
    this->scene = std::make_unique<ActionScene>(sceneFile.c_str());
    this->updateScene = entity.registerEvent<RcsGraph*, RcsGraph*, ActionScene*>("UpdateScene");
    scene->print();
  }

  // Graph component contains "sensed" graph
  graphC = new aff::GraphComponent(&entity, getGraph());
  graphC->setEnableRender(false);
  graphC->setEnableDifferentialKinematics(true);
  components.push_back(graphC);

  // Inverse kinematics controller, no constraints, right inverse
  ikc = new aff::IKTeleOp(&entity, controller.get());
  ikc->setEnableSpeedAccelerationLimit(!noLimits);
  ikc->setSpeedLimitCheck(!noLimits);
  ikc->setJointLimitCheck(!noLimits);
  ikc->setCollisionCheck(!noLimits);
  ikc->setLambda(lambda);
  ikc->setAlpha(alpha);
  components.push_back(ikc);

  // Initialize robot components from command line and componentArgs
  auto cTmp = createHardwareComponents(entity, getGraph(), scene.get(), false, componentArgs);
  this->hwc.insert(hwc.end(), cTmp.begin(), cTmp.end());
  cTmp = createComponents(entity, getGraph(), scene.get(), false, componentArgs);
  this->components.insert(components.end(), cTmp.begin(), cTmp.end());

  // Initialization sequence to initialize all graphs from the sensory state. This also triggers the
  // "Start" event, starting all component threads.
  entity.initialize(getCurrentGraph());

  // Initialize desired wrench with current robot's state
  if (this->inputType=="Wrench")
  {
    MatNd wrenchArr = MatNd_fromPtr(wrench_des.size(), 1, wrench_des.data());
    controller->computeX(&wrenchArr);
  }

  //std::cout << help() << std::endl;
  RcsGraph_fprintJoints(stdout, getCurrentGraph());

  return true;
}



bool ExampleTeleOp::initGraphics()
{
  if (noGraphics)
  {
    RLOG(0, "Skipping graphics");
    return true;
  }

  auto syncMode = blockingMainThread ? GraphicsWindow::SyncMode::External : GraphicsWindow::SyncMode::Threaded;
  viewer = new GraphicsWindow(&entity, syncMode);
  components.push_back(viewer);

  viewer->setTitle("ExampleTeleOp");
  viewer->setDynamicMeshUpdates(false);

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

  viewer->setKeyCallback('a', [this](char k)
  {
    static bool enable = false;
    enable = !enable;
    RLOG(0, "%s retargeting", enable ? "Enabling" : "Disabling");
    entity.publish("EnableRetargetting", enable);
  }, "Toggle speed scaling for retargetting");

  viewer->setKeyCallback('d', [this](char k)
  {
    static bool activateTasks = true;
    activateTasks = !activateTasks;
    RLOG(0, "%s tasks", activateTasks ? "Enabling" : "Disabling");
    entity.publish("EnableTasks", activateTasks);
  }, "Toggle task cativation");

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

  viewer->setKeyCallback('b', [this](char k)
  {
    RLOG(0, "Toggling broadphase in graphics window");
    osg::Switch* bpNode = dynamic_cast<osg::Switch*>(viewer->getNode("BroadPhase"));
    if (bpNode)
    {
      bool visible = bpNode->getValue(0);
      if (visible)
      {
        bpNode->setAllChildrenOff();
      }
      else
      {
        bpNode->setAllChildrenOn();
      }
    }

    RcsBroadPhase_fprint(stdout, controller->getBroadPhase());

  }, "Toggle broadphase in graphics window");

  // Broadphase visualization
  const RcsBroadPhase* bp = controller->getBroadPhase();

  if (!bp)
  {
    RLOG(0, "You forgot to specify a broadphase collision model");
  }
  else
  {
    osg::ref_ptr<osg::Switch> bpNode = new osg::Switch();
    bpNode->setAllChildrenOff();
    bpNode->setName("BroadPhase");
    for (unsigned int i = 0; i < bp->nBodies; ++i)
    {
      osg::ref_ptr<Rcs::AABBNode> sn = new Rcs::AABBNode();
      sn->makeDynamic(bp->bodies[i].aabbMin, bp->bodies[i].aabbMax);
      bpNode->addChild(sn.get());
    }
    for (unsigned int i = 0; i < bp->nTrees; ++i)
    {
      osg::ref_ptr<Rcs::AABBNode> sn = new Rcs::AABBNode();
      sn->makeDynamic(bp->trees[i].aabbMin, bp->trees[i].aabbMax);
      bpNode->addChild(sn.get());
      for (unsigned int j = 0; j < bp->trees[i].nBodies; ++j)
      {
        osg::ref_ptr<Rcs::AABBNode> sn = new Rcs::AABBNode();
        sn->makeDynamic(bp->trees[i].bodies[j].aabbMin,
                        bp->trees[i].bodies[j].aabbMax);
        bpNode->addChild(sn.get());
      }

    }

    viewer->add(bpNode.get());
  }

  // If we have a LandmarkComponent, we initialize its debug graphics
  // here. We have to defer it to this point, since there's no GraphicsWindow
  // before this.
  auto lmbs = getComponents<aff::LandmarkBase>(components);
  for (auto& c : lmbs)
  {
    RLOG_CPP(5, "Adding debug graphics to LandmarkComponent");
    c->createDebugGraphics(viewer, getGraph());
  }

  viewer->start();
  entity.publish("Render");
  entity.process();
  entity.publish("RenderCommand", std::string("ShowLines"), std::string("false"));
  entity.publish("RenderCommand", std::string("Physics"), std::string("hide"));
  entity.publish("RenderCommand", std::string("IK"), std::string("show"));
  entity.publish("RenderCommand", std::string("IK"), std::string("unsetGhostMode"));
  entity.process();

  // Show the graph of the GraphComponent (updated from hardware)
  if (enableRealGraphVisualization)
  {
    renderStringHUD = "Showing commands (shadow) and sensory state (solid)";
    graphC->setEnableRender(true);
    entity.publish<std::string, const RcsGraph*>("RenderGraph", "Physics", getCurrentGraph());
    entity.publish<std::string, const RcsGraph*>("RenderGraph", "IK", ikc->getGraph());
    entity.process();
    Timer_waitDT(0.5);
    entity.publish("RenderCommand", std::string("Physics"), std::string("show"));
    entity.publish("RenderCommand", std::string("IK"), std::string("show"));
    getEntity().publish("RenderCommand", std::string("IK"), std::string("setGhostMode"));
    entity.process();
  }

  return true;
}

bool ExampleTeleOp::initGuis()
{
  new aff::EventGui(&entity);
  return true;
}

void ExampleTeleOp::run()
{
  // Initialization sequence to initialize all graphs from the sensory state.
  // This also triggers the "Start" event, starting all component threads.
  this->runFunctionRunning = true;
  //entity.initialize(getCurrentGraph());
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
  this->runFunctionRunning = false;
  RLOG(1, "Returning from run() after stopping with events");
}

void ExampleTeleOp::step()
{
  dtProcess = Timer_getSystemTime();

  updateGraph->call(getCurrentGraph());
  computeKinematics->call(getCurrentGraph());
  postUpdateGraph->call(getGraph(), getCurrentGraph());

  if (this->inputType=="Retarget Polar")
  {
    updateScene->call(ikc->getGraph(), getCurrentGraph(), scene.get());
    ikc->onRetargetCommand(ikc->getGraph(), getCurrentGraph(), scene.get());
  }
  else if (this->inputType=="Twist")
  {
    ikc->onTwistCommand(this->twist_des, this->twist_in_world);
  }
  else if (this->inputType=="Wrench")
  {
    ikc->onWrenchCommand(this->wrench_des, this->wrench_in_world);
  }
  else if (this->inputType=="BiManualPose")
  {
    BiManualPoseCommand p;
    {
      std::lock_guard<std::mutex> lock(this->biManualPoseCommandMtx);
      p = this->biManualPoseCommand;
    }

    ikc->computeBiManualPoseCommand(p.leftHandPose, p.rightHandPose,
                                    p.rightFingersPose0, p.rightFingersPose1, p.s_right_01,
                                    p.leftFingersPose0, p.leftFingersPose1, p.s_left_01);
  }
  else
  {
    RLOG_CPP(1, "Unknown input type: " << this->inputType);
  }

  setJointCommand->call(ikc->getJointCommandPtr());
  setRenderCommand->call();
  entity.process();
  entity.stepTime();

  const RcsBody* ee = RcsGraph_getBodyByName(getGraph(), this->endEffectorName.c_str());
  if (ee)
  {
    std::lock_guard<std::mutex> lock(eeMtx);
    HTr_copy(&eeTrf, &ee->A_BI);
  }
  else
  {
    RLOG_CPP(1, "End effector for wrench not found");
    RCHECK_MSG(this->inputType!="Wrench", "For input type wrench, no end effector was found");
  }

  dtProcess = Timer_getSystemTime() - dtProcess;

  if (entity.getTime() > 3.0)
  {
    dt_max = std::max(dt_max, dtProcess);
  }

  loopCount++;

  char timeStr[256];
  snprintf(timeStr, 256, "Time: %.3f   dt: %.1f dt_max: %.1f msec\n"
           "queue: %zu (max: %zu)\n%s"
           "Joint speeds: %.0f %%   Tasks: %s",
           entity.getTime(), dtProcess * 1.0e3, dt_max * 1.0e3,
           entity.queueSize(), entity.getMaxQueueSize(), renderStringHUD.c_str(),
           100.0*ikc->getJointSpeedScaling(), ikc->getTasksActive() ? "on" : "off");
  entity.publish("SetTextLine", std::string(timeStr), 0);

  Timer_waitDT(entity.getDt() - dtProcess);

  RLOG(6, "Loop end %d", loopCount - 1);
  RLOG_CPP(6, "Loop end: queue size is " << entity.queueSize());
}

std::string ExampleTeleOp::help()
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

void ExampleTeleOp::onQuit()
{
  entity.publish("Stop");
  runLoop = false;
}

void ExampleTeleOp::onPrint()
{
  std::cout << help();
}

RcsGraph* ExampleTeleOp::getGraph()
{
  return controller ? controller->getGraph() : nullptr;
}

const RcsGraph* ExampleTeleOp::getGraph() const
{
  return controller ? controller->getGraph() : nullptr;
}

RcsGraph* ExampleTeleOp::getCurrentGraph()
{
  return graphC ? graphC->getGraph() : nullptr;
}

const RcsGraph* ExampleTeleOp::getCurrentGraph() const
{
  return graphC ? graphC->getGraph() : nullptr;
}

const EntityBase& ExampleTeleOp::getEntity() const
{
  return entity;
}

EntityBase& ExampleTeleOp::getEntity()
{
  return entity;
}

std::vector<double> ExampleTeleOp::getEndEffectorWrench() const
{
  std::vector<double> wrench(6, 0.0);

  std::lock_guard<std::mutex> lock(eeMtx);
  HTr_to6DVector(wrench.data(), &eeTrf);

  return wrench;
}

// std::vector<std::vector<double>> ExampleTeleOp::getBiManualPoseData() const
// {
//   std::vector<std::vector<double>> data;
//   std::lock_guard<std::mutex> lock(this->biManualPoseCommand.mtx);

//   data.push_back(biManualPoseCommand.leftHandPose);
//   data.push_back(biManualPoseCommand.rightHandPose);
//   data.push_back(std::vector<double> {biManualPoseCommand.s_left_01});
//   data.push_back(std::vector<double> {biManualPoseCommand.s_right_01});
//   return data;
// }

std::vector<double> ExampleTeleOp::getBodyPose(std::string bodyName,
                                               bool fromCurrentGraph) const
{
  const RcsGraph* graph = fromCurrentGraph ? getCurrentGraph() : getGraph();
  const RcsBody* bdy = RcsGraph_getBodyByName(graph, bodyName.c_str());

  std::vector<double> pose(6);
  HTr_to6DVector(pose.data(), &bdy->A_BI);

  return pose;
}

void ExampleTeleOp::startThreaded()
{
  std::thread t1([&]
  {
    ExampleBase::start();

    RLOG(0, "ExampleTeleOp thread says good bye");
  });
  t1.detach();
}

void ExampleTeleOp::updateUI()
{
  if (!viewer)
  {
    return;
  }

  viewer->frame();
  handleKeys();
}

void ExampleTeleOp::setSyncMode(std::string syncMode)
{
  ExampleBase::setSyncMode(syncMode);

  if (syncMode=="External")
  {
    blockingMainThread = true;
  }
}

void ExampleTeleOp::addComponentArgument(const std::string& arg)
{
  componentArgs += " " + arg;
}

void ExampleTeleOp::onSetTwist(double vel_x, double vel_y, double vel_z,
                               double vel_thx, double vel_thy, double vel_thz,
                               bool inWorldFrame)
{
  this->twist_des[0] = vel_x;
  this->twist_des[1] = vel_y;
  this->twist_des[2] = vel_z;

  // const double linear_vel_limit = 0.3*entity.getDt();
  // Vec3d_constSaturateSelf(&this->twist_des[0], linear_vel_limit);

  this->twist_des[3] = vel_thx;
  this->twist_des[4] = vel_thy;
  this->twist_des[5] = vel_thz;

  // const double angular_vel_limit = RCS_DEG2RAD(60.0)*entity.getDt();
  // Vec3d_constSaturateSelf(&this->twist_des[3], angular_vel_limit);

  this->twist_in_world = inWorldFrame;
}

void ExampleTeleOp::onSetWrench(double pos_x, double pos_y, double pos_z,
                                double eul_thx, double eul_thy, double eul_thz,
                                bool inWorldFrame)
{
  this->wrench_des[0] = pos_x;
  this->wrench_des[1] = pos_y;
  this->wrench_des[2] = pos_z;

  this->wrench_des[3] = eul_thx;
  this->wrench_des[4] = eul_thy;
  this->wrench_des[5] = eul_thz;

  this->wrench_in_world = true;
}

void ExampleTeleOp::onSetBiManualPoseCommand(std::vector<double> leftHandPose,
                                             std::vector<double> rightHandPose,
                                             std::string rightFingersPose0,
                                             std::string rightFingersPose1,
                                             double s_right_01,
                                             std::string leftFingersPose0,
                                             std::string leftFingersPose1,
                                             double s_left_01)
{
  RCHECK(leftHandPose.size()==6);
  RCHECK(rightHandPose.size()==6);
  std::lock_guard<std::mutex> lock(this->biManualPoseCommandMtx);
  biManualPoseCommand.leftHandPose = leftHandPose;
  biManualPoseCommand.rightHandPose = rightHandPose;
  biManualPoseCommand.rightFingersPose0 = rightFingersPose0;
  biManualPoseCommand.rightFingersPose1 = rightFingersPose1;
  biManualPoseCommand.s_right_01 = s_right_01;
  biManualPoseCommand.leftFingersPose0 = leftFingersPose0;
  biManualPoseCommand.leftFingersPose1 = leftFingersPose1;
  biManualPoseCommand.s_left_01 = s_left_01;
}


void ExampleTeleOp::onSetFingerPose(std::string modelStateName)
{
  if (fingerPoseName==modelStateName)
  {
    return;
  }

  fingerPoseName = modelStateName;

  RLOG_CPP(0, "Setting finger pose to  " << modelStateName);

  auto it = modelStates.find(modelStateName);

  if (it != modelStates.end())
  {
    auto& vec = it->second; // std::vector<std::pair<int,double>>&
    if (vec.size() == fingers_des.size())
    {
      for (std::size_t i = 0; i < vec.size(); ++i)
      {
        fingers_des[i] = vec[i].second;
        MatNd_set(getGraph()->q, vec[i].first, 0, vec[i].second);
      }

    }
    else
    {
      RLOG_CPP(0, "Size mismatch: model state should have 16 elements, but has " << vec.size());
    }
  }
  else
  {
    RLOG_CPP(0, "Model state " << modelStateName << " unknown");
  }
}

void ExampleTeleOp::setBuildPath(const std::string& path)
{
  this->build_path = path + "/";
}

void ExampleTeleOp::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  const RcsJoint* fingerDrvLeft_des = RcsGraph_getJointByName(desired, "joint_driving_left");
  const RcsJoint* fingerDrvLeft_curr = RcsGraph_getJointByName(current, "joint_driving_left");

  if (fingerDrvLeft_des && fingerDrvLeft_curr)
  {
    current->q->ele[fingerDrvLeft_curr->jointIndex] = desired->q->ele[fingerDrvLeft_des->jointIndex];
  }

  collectData(desired, current);
}

void ExampleTeleOp::collectData(RcsGraph* desired, RcsGraph* current)
{
  const RcsBody* ee = controller->getTask(0)->getEffector();
  RCHECK(ee);
  ee = RcsGraph_getBodyByName(desired, ee->name);
  RCHECK(ee);

  std::vector<double> pos(ee->A_BI.org, ee->A_BI.org+3);
  double* rmPtr = (double*)ee->A_BI.rot[0];
  std::vector<double> rm(rmPtr, rmPtr+9);

  std::vector<double> twist_in_world(6);
  for (size_t i=0; i<3; ++i)
  {
    twist_in_world[i] = ee->x_dot[i];
    twist_in_world[i+3] = ee->omega[i];
  }

  std::vector<double> twist_in_ee(6);
  double k_vel[3], k_om[3];
  Vec3d_rotate(k_vel, (double(*)[3])ee->A_BI.rot, ee->x_dot);
  Vec3d_rotate(k_om, (double(*)[3])ee->A_BI.rot, ee->omega);
  for (size_t i=0; i<3; ++i)
  {
    twist_in_ee[i] = k_vel[i];
    twist_in_ee[i+3] = k_om[i];
  }

  std::vector<double> q_curr(current->q->ele, current->q->ele+current->dof);
  std::vector<double> q_des(desired->q->ele, desired->q->ele+desired->dof);

  std::vector<double> fts_base(6, 0.0), fts_ee(6, 0.0);

  RcsSensor* s = RcsGraph_getSensorByName(current, "fts_base_right");
  if (s && s->type==RCSSENSOR_LOAD_CELL)
  {
    VecNd_copy(fts_base.data(), s->rawData->ele, 6);
  }

  s = RcsGraph_getSensorByName(current, "fts_ee_right");
  if (s && s->type==RCSSENSOR_LOAD_CELL)
  {
    VecNd_copy(fts_ee.data(), s->rawData->ele, 6);
  }

  collectedData.set(pos, rm, twist_in_world, twist_in_ee, q_curr, q_des, fts_base, fts_ee);
}

std::vector<std::vector<double>> ExampleTeleOp::getCollectedData() const
{
  return collectedData.get();
}

void ExampleTeleOp::printCollectedData() const
{
  RcsGraph_fprintJoints(stdout, getCurrentGraph());
  collectedData.print();
}

std::vector<std::pair<std::string,VirtualCamera*>> ExampleTeleOp::getVirtualCameras()
{
  std::vector<std::pair<std::string,VirtualCamera*>> cams;
  for (size_t i=0; i<virtualCameras.size(); ++i)
  {
    std::string cam_i_name = virtualCameras[i].first;
    VirtualCamera* cam_i = virtualCameras[i].second.get();
    cams.push_back(std::make_pair(cam_i_name, cam_i));
  }

  return cams;
}

bool ExampleTeleOp::addVirtualCamera(std::string camera_name, std::string camera_type, int width, int height)
{
  if (!RcsGraph_getBodyByName(getGraph(), camera_name.c_str()))
  {
    RLOG(1, "Camera \"%s\" not found in graph", camera_name.c_str());

    for (unsigned int i=0; i<getGraph()->nBodies; ++i)
    {
      std::cout << "Body " << i << ": " << getGraph()->bodies[i].name << std::endl;
    }
    return false;
  }

  VirtualCamera* camera = new VirtualCamera(camera_type, new Rcs::GraphNode(getGraph()), width, height);
  virtualCameras.push_back(std::make_pair(camera_name, std::unique_ptr<VirtualCamera>(camera)));

  return true;
}


/*******************************************************************************
 *
 ******************************************************************************/
ExampleTeleOpFrankaRight::ExampleTeleOpFrankaRight() : ExampleTeleOpFrankaRight(0, NULL)
{
}

ExampleTeleOpFrankaRight::ExampleTeleOpFrankaRight(int argc, char** argv) : ExampleTeleOp(argc, argv)
{
}

RCS_REGISTER_EXAMPLE(ExampleTeleOpFrankaRight, "A TeleOp", "Franka right TeleOp");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleTeleOpFrankaRightSpacemouse : public ExampleTeleOp
{
public:

  ExampleTeleOpFrankaRightSpacemouse() : ExampleTeleOpFrankaRightSpacemouse(0, NULL)
  {
  }

  ExampleTeleOpFrankaRightSpacemouse(int argc, char** argv) : ExampleTeleOp(argc, argv)
  {
  }

  bool initParameters()
  {
    ExampleTeleOp::initParameters();
    addComponentArgument("-spacemouse");
    return true;
  }

};

RCS_REGISTER_EXAMPLE(ExampleTeleOpFrankaRightSpacemouse, "A TeleOp", "Franka right TeleOp with Spacemouse");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleTeleOpFrankaRightMetaquest : public ExampleTeleOp
{
public:

  ExampleTeleOpFrankaRightMetaquest() : ExampleTeleOpFrankaRightMetaquest(0, NULL)
  {
  }

  ExampleTeleOpFrankaRightMetaquest(int argc, char** argv) : ExampleTeleOp(argc, argv)
  {
  }

  bool initParameters()
  {
    ExampleTeleOp::initParameters();
    withScene = true;
    enableRealGraphVisualization = false;
    inputType = "Retarget Polar";
    noLimits = false;
    xmlFileName = "c_franka_retarget_right.xml";
    configDirectory = "config/xml/Franka";
    //addComponentArgument("-eye_ik -eye_ik.camera_name azure_kinect_rgb_frame ");
    addComponentArgument("-landmarks_router -landmarks_connection tcp://*:40000 -landmarks_camera unity_world -skeleton_tracking -skeleton_radius 1000 ");

    return true;
  }

  bool initAlgo()
  {
    if (withRobo)
    {
      addComponentArgument("-frankaZmq_left -frankaZmq_left.ip 192.168.1.101");
      addComponentArgument("-frankaZmq_right -frankaZmq_right.ip 192.168.1.101");
      addComponentArgument("-allegroZmq_left -allegroZmq_left.ip 192.168.0.101");
      addComponentArgument("-allegroZmq_right -allegroZmq_right.ip 192.168.0.101");
      addComponentArgument("-allegroZmq_wrongThumb");
      addComponentArgument("-pw70_zmq -pw70_zmq.ip 192.168.0.101");
    }

    return ExampleTeleOp::initAlgo();
  }

};

RCS_REGISTER_EXAMPLE(ExampleTeleOpFrankaRightMetaquest, "A TeleOp", "Franka retargetting with Metaquest Pro");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleTeleOpFrankaRightMetaquestFile : public ExampleTeleOp
{
public:

  ExampleTeleOpFrankaRightMetaquestFile() : ExampleTeleOpFrankaRightMetaquestFile(0, NULL)
  {
  }

  ExampleTeleOpFrankaRightMetaquestFile(int argc, char** argv) : ExampleTeleOp(argc, argv)
  {
  }

  bool initParameters()
  {
    ExampleTeleOp::initParameters();
    withScene = true;
    enableRealGraphVisualization = false;
    inputType = "Retarget Polar";
    noLimits = false;
    xmlFileName = "c_franka_retarget_right.xml";
    configDirectory = "config/xml/Franka";
    addComponentArgument("-landmarks_router -landmarks_connection config/data/meta_tracking_with_fingers.json -landmarks_camera unity_world -skeleton_tracking -skeleton_radius 1000 ");
    //addComponentArgument("-eye_ik -eye_ik.camera_name azure_kinect_rgb_frame ");

    return true;
  }

  bool initAlgo()
  {
    if (withRobo)
    {
      addComponentArgument("-frankaZmq_left -frankaZmq_left.ip 192.168.1.101");
      addComponentArgument("-frankaZmq_right -frankaZmq_right.ip 192.168.1.101");
      addComponentArgument("-allegroZmq_left -allegroZmq_left.ip 192.168.0.101");
      addComponentArgument("-allegroZmq_right -allegroZmq_right.ip 192.168.0.101");
      addComponentArgument("-pw70_zmq -pw70_zmq.ip 192.168.0.101");
    }

    return ExampleTeleOp::initAlgo();
  }

};

RCS_REGISTER_EXAMPLE(ExampleTeleOpFrankaRightMetaquestFile, "A TeleOp", "Franka retargetting with Metaquest (from log file)");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleTeleOpJaco3MetaquestFile : public ExampleTeleOp
{
public:

  ExampleTeleOpJaco3MetaquestFile() : ExampleTeleOpJaco3MetaquestFile(0, NULL)
  {
  }

  ExampleTeleOpJaco3MetaquestFile(int argc, char** argv) : ExampleTeleOp(argc, argv)
  {
  }

  bool initParameters()
  {
    ExampleTeleOp::initParameters();
    withScene = true;
    enableRealGraphVisualization = false;
    inputType = "Retarget Polar";
    noLimits = false;
    xmlFileName = "c_robo.xml";
    configDirectory = "config/xml/JacoGen3";
    addComponentArgument("-landmarks_router -landmarks_connection tracking.json -landmarks_camera unity_world -skeleton_tracking -skeleton_radius 1000 ");
    //addComponentArgument("-eye_ik -eye_ik.camera_name azure_kinect_rgb_frame ");

    return true;
  }

  bool initAlgo()
  {
    if (withRobo)
    {
      // addComponentArgument("-frankaZmq_left -frankaZmq_left.ip 192.168.1.101");
      // addComponentArgument("-frankaZmq_right -frankaZmq_right.ip 192.168.1.101");
      // addComponentArgument("-pw70_zmq -pw70_zmq.ip 192.168.0.101");
    }

    return ExampleTeleOp::initAlgo();
  }

};

RCS_REGISTER_EXAMPLE(ExampleTeleOpJaco3MetaquestFile, "A TeleOp", "Jaco Gen3 retargetting with Metaquest (from log file)");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleTeleOpJaco3Metaquest : public ExampleTeleOp
{
public:

  ExampleTeleOpJaco3Metaquest() : ExampleTeleOpJaco3Metaquest(0, NULL)
  {
  }

  ExampleTeleOpJaco3Metaquest(int argc, char** argv) : ExampleTeleOp(argc, argv)
  {
  }

  bool initParameters()
  {
    ExampleTeleOp::initParameters();
    withScene = true;
    enableRealGraphVisualization = false;
    inputType = "Retarget Polar";
    noLimits = false;
    xmlFileName = "c_robo.xml";
    configDirectory = "config/xml/JacoGen3";
    addComponentArgument("-landmarks_router -landmarks_connection tcp://*:40000 -landmarks_camera unity_world -skeleton_tracking -skeleton_radius 1000 ");
    //addComponentArgument("-eye_ik -eye_ik.camera_name azure_kinect_rgb_frame ");

    return true;
  }

  bool initAlgo()
  {
    if (withRobo)
    {
      // addComponentArgument("-frankaZmq_left -frankaZmq_left.ip 192.168.1.101");
      // addComponentArgument("-frankaZmq_right -frankaZmq_right.ip 192.168.1.101");
      // addComponentArgument("-pw70_zmq -pw70_zmq.ip 192.168.0.101");
    }

    return ExampleTeleOp::initAlgo();
  }

};

RCS_REGISTER_EXAMPLE(ExampleTeleOpJaco3Metaquest, "A TeleOp", "Jaco Gen3 retargetting with Metaquest");




}   // namespace aff
