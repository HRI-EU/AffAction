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

#include "ExampleActionsECS.h"
#include "ActionFactory.h"
#include "ActionSequence.h"
#include "HardwareComponent.h"

#include <EventGui.h>
#include <ConstraintFactory.h>
#include <ActivationSet.h>

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
#include <Rcs_distanceWM5.h>
#include <Rcs_broadphase.h>

#include <CmdLineWidget.h>

#include <BodyPointDragger.h>
#include <MouseDragger.h>
#include <AABBNode.h>
#include <SphereNode.h>
#include <BodyNode.h>

#include <fstream>
#include <iostream>
#include <thread>


void AffActionExampleInfo()
{
  Rcs::ExampleFactory::print();
}


namespace aff
{
// Utility functions for trimming a string
std::string& ltrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
  str.erase(0, str.find_first_not_of(chars));
  return str;
}

std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
  str.erase(str.find_last_not_of(chars) + 1);
  return str;
}

std::string& trim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
  return ltrim(rtrim(str, chars), chars);
}

/*******************************************************************************
 *
 ******************************************************************************/
static RcsBody* getBodyUnderMousePointer(RcsGraph* graph,
                                         const osgGA::GUIEventAdapter& ea,
                                         osgGA::GUIActionAdapter& aa,
                                         double I_pt[3], double k_pt[3])
{
  Rcs::BodyNode* nd = Rcs::getNodeUnderMouse<Rcs::BodyNode*>(ea, aa, I_pt);

  if (nd == NULL)
  {
    return NULL;
  }

  // Rcs::setNodeAlpha(nd, 0.5 + 0.49*sin(Timer_getSystemTime()));

  RcsBody* bdy = RCSBODY_BY_ID(graph, nd->body()->id);

  if (k_pt && I_pt && bdy)
  {
    Vec3d_invTransform(k_pt, &bdy->A_BI, I_pt);
  }

  return bdy;
}

class NamedMouseDragger: public Rcs::BodyPointDragger//MouseDragger
{
public:
  NamedMouseDragger(RcsGraph* graph_) : graph(graph_)
  {
  }

  RcsBody* getBodyUnderMouse(const osgGA::GUIEventAdapter& ea,
                             osgGA::GUIActionAdapter& aa,
                             double I_pt[3] = NULL,
                             double k_pt[3] = NULL,
                             RcsGraph** g = NULL)
  {
    RcsBody* bdy = getBodyUnderMousePointer(graph, ea, aa, I_pt, k_pt);

    if (g)
    {
      *g = graph;
    }

    return bdy;
  }

  const RcsBody* getBodyUnderMouse(const osgGA::GUIEventAdapter& ea,
                                   osgGA::GUIActionAdapter& aa,
                                   double I_pt[3] = NULL,
                                   double k_pt[3] = NULL,
                                   RcsGraph** g = NULL) const
  {
    const RcsBody* bdy = getBodyUnderMousePointer(graph, ea, aa, I_pt, k_pt);

    if (g)
    {
      *g = graph;
    }

    return bdy;
  }

  bool callback(const osgGA::GUIEventAdapter& ea,
                osgGA::GUIActionAdapter& aa)
  {
    double F[3];
    getDragForce(F);
    //RLOG(0, "Drag force is %f", Vec3d_getLength(F));
    return Rcs::BodyPointDragger::callback(ea, aa);
  }

private:

  RcsGraph* graph;
};



/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleActionsECS, "Actions", "ECS");

ExampleActionsECS::ExampleActionsECS(int argc, char** argv) :
  ExampleBase(argc, argv), entity(), graphToInitializeWith(NULL)
{
  if (!Rcs_hasWM5())
  {
    RLOG(0, "WM5 library not linked - some actions will not work properly");
    RPAUSE();
  }

  trajTime = 0.0;
  ikType = IKComponent::IkSolverType::RMR;
  dt = 0.01;
  dt_max = 0.0;
  dt_max2 = 0.0;
  alpha = 0.05;
  lambda = 1.0e-6;
  speedUp = 1;
  loopCount = 0;

  pause = false;
  noSpeedCheck = false;
  noJointCheck = false;
  noCollCheck = false;
  noTrajCheck = false;

  noLimits = false;
  zigzag = false;
  withEventGui = false;
  withTaskGui = false;
  noViewer = false;
  noTextGui = false;
  plot = false;
  valgrind = false;
  unittest = false;
  withRobot = false;
  singleThreaded = false;

  dtProcess = 0.0;
  dtEvents = 0.0;
  failCount = 0;

  updateGraph = NULL;
  postUpdateGraph = NULL;
  computeKinematics = NULL;
  computeTrajectory = NULL;
  setTaskCommand = NULL;
  setJointCommand = NULL;
  setRenderCommand = NULL;
}

ExampleActionsECS::~ExampleActionsECS()
{
  for (size_t i = 0; i < hwc.size(); ++i)
  {
    delete hwc[i];
  }

  for (size_t i = 0; i < components.size(); ++i)
  {
    delete components[i];
  }

  Rcs_removeResourcePath(config_directory.c_str());
  RcsGraph_destroy(graphToInitializeWith);
}

bool ExampleActionsECS::initParameters()
{
  xmlFileName = "g_scenario.xml";
  config_directory = "config/xml/SmileActions";

  return true;
}

bool ExampleActionsECS::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  parser->getArgument("-speedUp", &speedUp, "Speed-up factor (default: %d)",
                      speedUp);
  parser->getArgument("-dt", &dt, "Time step (default is %f)", dt);
  parser->getArgument("-f", &xmlFileName, "Configuration file name "
                      "(default is %s)", xmlFileName.c_str());
  parser->getArgument("-dir", &config_directory, "Configuration file directory "
                      "(default is %s)", config_directory.c_str());
  parser->getArgument("-alpha", &alpha,
                      "Null space scaling factor (default is %f)", alpha);
  parser->getArgument("-lambda", &lambda, "Regularization (default: %f)", lambda);
  parser->getArgument("-pause", &pause, "Pause after each process() call");
  parser->getArgument("-nospeed", &noSpeedCheck, "No speed limit checks");
  parser->getArgument("-nojl", &noJointCheck, "Disable joint limit checks");
  parser->getArgument("-nocoll", &noCollCheck, "Disable collision checks");
  parser->getArgument("-notrajcheck", &noTrajCheck, "Disable trajectory checks");
  parser->getArgument("-noLimits", &noLimits, "Ignore all robot limits");
  parser->getArgument("-zigzag", &zigzag, "Zig-zag trajectory");
  parser->getArgument("-eventGui", &withEventGui, "Launch event gui");
  parser->getArgument("-taskGui", &withTaskGui, "Launch task gui");
  parser->getArgument("-noViewer", &noViewer, "Do not launch viewer");
  parser->getArgument("-noTextGui", &noTextGui, "Do not launch TextGui");
  parser->getArgument("-plot", &plot, "Enable debug plotting");
  parser->getArgument("-valgrind", &valgrind, "Valgrind mode without graphics and Gui");
  parser->getArgument("-unittest", &unittest, "Run unit tests");
  parser->getArgument("-singleThreaded", &singleThreaded, "Run predictions sequentially");
  parser->getArgument("-sequence", &sequenceCommand, "Sequence command to start with");

  // This is just for pupulating the parsed command line arguments for the help
  // functions / help window.
  const bool dryRun = true;
  getHardwareComponents(entity, NULL, NULL, dryRun);
  getComponents(entity, NULL, NULL, dryRun);

  return true;
}

bool ExampleActionsECS::initAlgo()
{
  Rcs_addResourcePath(RCS_CONFIG_DIR);
  Rcs_addResourcePath(config_directory.c_str());

  if (noLimits)
  {
    noSpeedCheck = true;
    noJointCheck = true;
    noCollCheck = true;
  }

  entity.registerEvent<>("EmergencyStop");
  entity.registerEvent<>("EmergencyRecover");
  entity.registerEvent<>("Quit");
  entity.subscribe("Quit", &ExampleActionsECS::onQuit, this);
  entity.subscribe("Print", &ExampleActionsECS::onPrint, this);
  entity.subscribe("TrajectoryMoving", &ExampleActionsECS::onTrajectoryMoving, this);
  entity.subscribe("ActionSequence", &ExampleActionsECS::onActionSequence, this);
  entity.subscribe("TextCommand", &ExampleActionsECS::onTextCommand, this);

  entity.setDt(dt);
  updateGraph = entity.registerEvent<RcsGraph*>("UpdateGraph");
  computeKinematics = entity.registerEvent<RcsGraph*>("ComputeKinematics");
  computeTrajectory = entity.registerEvent<RcsGraph*>("ComputeTrajectory");
  setTaskCommand = entity.registerEvent<const MatNd*, const MatNd*>("SetTaskCommand");
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

  // Extract the collision model
  {
    xmlDocPtr doc = NULL;
    xmlNodePtr node = parseXMLFile(controller->getGraph()->cfgFile, NULL, &doc);
    if (node)
    {
      xmlNodePtr child = getXMLChildByName(node, "CollisionModel");
      if (child)
      {
        RcsCollisionMdl* cMdl = RcsCollisionModel_createFromXML(controller->getGraph(), child);
        controller->setCollisionMdl(cMdl);
      }


      child = getXMLChildByName(node, "BroadPhase");
      if (child)
      {
        RcsBroadPhase* bp = RcsBroadPhase_createFromXML(controller->getGraph(), child);
        RcsBroadPhase_updateBoundingVolumes(bp);
        controller->setBroadPhase(bp);

        if (!controller->getCollisionMdl())
        {
          RcsCollisionMdl* cMdl = RcsCollisionModel_create(controller->getGraph());
          controller->setCollisionMdl(cMdl);
        }
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


  // Also in the simulator, start out with the Jaco's default pose so that we
  // see when something is going wrong during the initial moves.
  //RcsGraph_getModelStateFromXML(graph->q, graph, "JacoDefaultPose", 0);
  //RcsGraph_setState(graph, NULL, NULL);

  actionC = std::make_unique<aff::ActionComponent>(&entity, controller->getGraph(), controller->getBroadPhase());
  actionC->setLimitCheck(!noLimits);
  actionC->setMultiThreaded(!singleThreaded);
  graphC = std::make_unique<aff::GraphComponent>(&entity, controller->getGraph());
  graphC->setEnableRender(false);//\todo MG CHECK
  trajC = std::make_unique<aff::TrajectoryComponent>(&entity, controller.get(), !zigzag, 1.0,
                                                     !noTrajCheck);
  trajC->enableDebugRendering(false);

  // Inverse kinematics controller, no constraints, right inverse
  ikc = std::make_unique<aff::IKComponent>(&entity, controller.get(), ikType);
  ikc->setEnableSpeedAccelerationLimit(!noSpeedCheck);
  ikc->setSpeedLimitCheck(!noSpeedCheck);
  ikc->setJointLimitCheck(!noJointCheck);
  ikc->setCollisionCheck(!noCollCheck);
  ikc->setLambda(lambda);
  ikc->setAlpha(alpha);

  RCHECK(controller.get() == trajC->getTrajectoryController()->getController());

  // Remember the state for re-initialization
  graphToInitializeWith = RcsGraph_clone(controller->getGraph());

  // Initialize robot components from command line
  this->hwc = getHardwareComponents(entity, controller->getGraph(), actionC->getDomain(), false);
  this->components = getComponents(entity, controller->getGraph(), actionC->getDomain(), false);

  if (!hwc.empty())
  {
    setEnableRobot(true);
  }

  // Initialization sequence to initialize all graphs from the sensory state
  entity.initialize(graphC->getGraph());

  return true;
}

bool ExampleActionsECS::initGraphics()
{
  if (viewer)
  {
    RLOG(1, "Graphics already initialized");
    return false;
  }

  // Optional graphics window. We don't use a static instance since this will
  // not be cleaned up after exiting main.
  if (noViewer || valgrind)
  {
    RLOG(0, "Running without graphics");
    return true;
  }

  viewer = std::make_unique<aff::GraphicsWindow>(&entity, false);
  viewer->add(new NamedMouseDragger(controller->getGraph()));
  viewer->setTitle("ExampleActionsECS");
  viewer->setCameraTransform(-3.923553, -3.427215, 3.719906, -0.253100, 0.551730, 0.595891);
#if defined(_MSC_VER)
  viewer->setWindowSize(12, 36, 1000, 750);
#else
  viewer->setWindowSize(0, 0, 1000, 750);
#endif

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

  viewer->setKeyCallback('t', [this](char k)
  {
    RLOG(0, "Increment prediction visualization");
    entity.publish("ToggleFastPrediction");
  }, "Increment prediction visualization");

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

  viewer->setKeyCallback('f', [this](char k)
  {
    RLOG(0, "Loading action sequence from file");
    std::string filename = "action_sequence.txt";
    std::vector<std::string> lines;
    std::ifstream file(filename);

    if (!file.is_open())
    {
      RLOG_CPP(0, "Failed to open file: " << filename);
    }
    else
    {
      std::string line;
      while (std::getline(file, line))
      {
        lines.push_back(line);
      }

      file.close();

      std::string result;
      for (size_t i = 0; i < lines.size(); ++i)
      {
        result += lines[i];
        if (i != lines.size() - 1)
        {
          result += ";";
        }
      }

      entity.publish("ActionSequence", result);
    }

  }, "Load action sequence from file");

  viewer->setKeyCallback('h', [this](char k)
  {
    RLOG(0, "Showing actions and affordances");
    std::string str = ActionFactory::printToString();
    str += "\n\n";
    str += actionC->getDomain()->printAffordancesToString();
    new Rcs::TextGui(str, "Actions and affordances");
  }, "Showing actions and affordances in text window");
  viewer->setKeyCallback('C', [this](char k)
  {
    static bool flag = false;
    flag = !flag;
    std::string flagMsg = flag ? "true" : "false";
    RLOG_CPP(0, "Toggle collision model to " << flagMsg);
    entity.publish("RenderCommand", std::string("ShowLines"), flagMsg);
  }, "Toggle collision model");

  viewer->setKeyCallback('e', [this](char k)
  {
    new aff::EventGui(&entity);
  }, "Launch event gui");

  viewer->setKeyCallback('k', [this](char k)
  {
    auto ts = tropic::ConstraintFactory::create("traj.xml");

    if (!ts)
    {
      RMSG("Failed to load trajectory from file \"traj.xml\"");
    }
    else
    {
      entity.publish("SetTrajectory", ts);
      RMSG("Loading trajectory from file \"traj.xml\"");
      ts->print();
    }
  }, "Load traj.xml and set it without checking");

  viewer->setKeyCallback(' ', [this](char k)
  {
    entity.call("TogglePause");

  }, "Toggle pause modus");

  viewer->setKeyCallback('T', [this](char k)
  {
    trajC->getTrajectoryController()->toXML("RobjectTrajectory.xml");
    trajC->getTrajectoryController()->getController()->toXML("RobotController.xml");

  }, "Write trajectory to file \"RobjectTrajectory.xml\" and controller to \"RobotController.xml\"");

  viewer->setKeyCallback('o', [this](char k)
  {
    RLOG(0, "Launching ControllerGui");

    new Rcs::ControllerGui(controller.get(),
                           (MatNd*) trajC->getActivationPtr(),
                           (MatNd*) trajC->getTaskCommandPtr(),
                           (const MatNd*) trajC->getTaskCommandPtr(),
                           NULL,
                           true);

  }, "Launch ControllerGui (passive)");

  viewer->setKeyCallback('d', [this](char k)
  {
    static bool enable = true;
    entity.publish("SetDebugRendering", enable);
    enable = !enable;

  }, "Toggle predictor debug rendering");

  viewer->setKeyCallback('X', [this](char k)
  {
    RMSG("Writing controller to \"cOut.xml\" and graph to \"RcsGraph.dot\"");
    controller->toXML("cOut.xml");

    RcsGraph_writeDotFile(controller->getGraph(), "RcsGraph.dot");
    std::string dottyCommand = "dotty " + std::string("RcsGraph.dot") + "&";
    int err = system(dottyCommand.c_str());
    if (err == -1)
    {
      RLOG(1, "Couldn't start dotty tool");
    }

  }, "Show dot file");

  viewer->setKeyCallback('m', [this](char k)
  {
    Rcs::BodyNode* bn = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    if (!bn)
    {
      return;
    }

    const ActionScene* scene = actionC->getDomain();
    std::vector<const Manipulator*> om = scene->getOccupiedManipulators(controller->getGraph());
    RLOG_CPP(0, "Found " << om.size() << " free manipulators");

    if (om.empty())
    {
      std::string textCmd = "get " + std::string(bn->body()->name);
      entity.publish("ActionSequence", textCmd);
    }
    else
    {
      std::vector<const AffordanceEntity*> ge = om[0]->getGraspedEntities(*scene, controller->getGraph());
      RCHECK(!ge.empty());

      std::string textCmd = "put " + ge[0]->name + " " + std::string(bn->body()->name);
      entity.publish("ActionSequence", textCmd);
    }

  }, "Get body under mouse");


  viewer->start();

  entity.publish("RenderCommand", std::string("ShowLines"), std::string("false"));
  entity.publish("RenderCommand", std::string("Physics"), std::string("hide"));
  entity.publish("RenderCommand", std::string("IK"), std::string("show"));
  entity.publish("RenderCommand", std::string("IK"), std::string("unsetGhostMode"));
  //entity.publish("RenderCommand", std::string("IK"), std::string("hide"));
  entity.process();

  return true;
}

bool ExampleActionsECS::initGuis()
{
  if (valgrind)
  {
    return true;
  }

  // Optional task gui to inspect task-level coordinates. We don't use a static
  // instance since this will not be cleaned up after exiting main.
  if (withTaskGui)
  {
    taskGui = std::make_unique<TaskGuiComponent>(&entity, controller.get());
    taskGui->setPassive(true);
  }

  // Gui for manually triggering events with basic data types.
  if (withEventGui)
  {
    new aff::EventGui(&entity);
  }

  if (!noTextGui)
  {
    textGui = std::make_unique<aff::TextEditComponent>(&entity);
  }

  return true;
}

void ExampleActionsECS::run()
{
  // Start all threads of components. We also publish a sequence command. If it
  // has not been set, it is an empty string that immediately returns.
  entity.publish("Start");
  entity.process();

  if (!sequenceCommand.empty())
  {
    entity.publish("ActionSequence", sequenceCommand);
  }

  while (runLoop)
  {
    step();
  }

  // The runLoop is ended with ExampleBase::stop(). We still need to call each
  // component's stop event.
  entity.publish("Stop");
  entity.process();
}

void ExampleActionsECS::step()
{
  dtProcess = Timer_getSystemTime();

  stepMtx.lock();
  updateGraph->call(graphC->getGraph());
  computeKinematics->call(graphC->getGraph());
  postUpdateGraph->call(ikc->getGraph(), graphC->getGraph());
  computeTrajectory->call(ikc->getGraph());
  setTaskCommand->call(trajC->getActivationPtr(), trajC->getTaskCommandPtr());
  setJointCommand->call(ikc->getJointCommandPtr());
  setRenderCommand->call();
  dtEvents = Timer_getSystemTime() - dtProcess;

  if (withTaskGui)
  {
    MatNd_copy(graphC->getGraph()->q, ikc->getGraph()->q);
  }

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
  double endTime = trajC->getMotionEndTime();
  snprintf(timeStr, 256, "Time: %.3f   dt: %.1f dt_max: %.1f %.1f msec\n"
           "failCount: %zu queue: %zu (max: %zu)  End time: %.3f %.3f",
           entity.getTime(), dtProcess * 1.0e3, dt_max * 1.0e3, dt_max2 * 1.0e3,
           failCount, entity.queueSize(), entity.getMaxQueueSize(),
           endTime, trajTime);
  entity.publish("SetTextLine", std::string(timeStr), 0);

  if (endTime > 0.0)
  {
    trajTime += entity.getDt();
  }

  if (loopCount % speedUp == 0)
  {
    Timer_waitDT(entity.getDt() - dtProcess);
  }

  RLOG(6, "Loop end %d", loopCount - 1);
  RLOG_CPP(6, "Loop end: queue size is " << entity.queueSize());
}

std::string ExampleActionsECS::help()
{
  std::stringstream s;

  s << ExampleBase::help();
  s << ActionFactory::printToString();
  s << Rcs::RcsGraph_printUsageToString(xmlFileName);
  s << Rcs::RcsShape_distanceFunctionsToString();

  return s.str();
}

void ExampleActionsECS::onQuit()
{
  entity.publish("Stop");
  runLoop = false;
}

void ExampleActionsECS::onActionSequence(std::string text)
{
  RMSG_CPP("RECEIVED: " << text);
  clearCompletedActionStack();

  // Early exit in case we receive an empty string
  if (text.empty())
  {
    std::string explanation = "ERROR REASON: Received empty action command string. SUGGESTION: Check your spelling DEVELOPER: '" +
                              text + "' " + std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
    entity.publish("ActionResult", false, 0.0, explanation);
    return;
  }

  // Step 1: Analyse if we received an ActionSequence to be broken down. This is
  // starting with either 'sequence' or 's'.
  std::vector<std::string> words = Rcs::String_split(text, " ");

  // We first check if a sequence command has been given. This is
  // starting with either 'sequence' or 's'.
  if ((!words.empty()) && (words[0] == "sequence" || words[0] == "s"))
  {
    if (words.size() != 2)
    {
      // In case the sequence command has been given, we expect more
      // characters describing it. If there are none, this is interpreted
      // as an error and we return.
      std::string explanation = "ERROR REASON: Received sequence with " + std::to_string(words.size()) +
                                " parameters (two are expected). SUGGESTION: Check your spelling DEVELOPER: '" +
                                text + "' " + std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
      entity.publish("ActionResult", false, 0.0, explanation);
      return;
    }

    // In case we detected a sequence, we parse the graph's configuration
    // file again to load all available sequences. We do this here in order
    // to allow to modify the file at run-time.
    RLOG(1, "Detected action sequence");
    ActionSequence seq(controller->getGraph()->cfgFile);
    bool sequenceFound = false;
    for (const auto& pair : seq.sequences)
    {
      if (words[1] == pair.first)
      {
        // If a valid sequence name has been found, we expand it into the text string.
        text = pair.second;
        sequenceFound = true;
        RLOG_CPP(0, "Sequence expanded to '" << text << "'");
        break;
      }
    }

    // If we end up here, a sequence with a name has been given, but the
    // sequence was not specified in the configuration file. This is
    // treated as an error.
    if (!sequenceFound)
    {
      std::string explanation = "ERROR: Can't execute action sequence. REASON: Action sequence " +
                                words[1] + " not found in config file DEVELOPER: '" + text + "'";
      entity.publish("ActionResult", false, 0.0, explanation);
      RLOG_CPP(0, explanation);
      return;
    }

    // Check for cascading sequences and expand them
    bool innerSequenceFound = true;
    while (innerSequenceFound)
    {
      // We start with the assumption that no inner sequence has been found.
      // Split the text into its individual commands.
      innerSequenceFound = false;
      std::vector<std::string> textWords = Rcs::String_split(text, ";");

      // Split individual commands into single words and check if they are
      // sequences.
      for (const auto& command : textWords)
      {
        std::vector<std::string> words = Rcs::String_split(command, " ");
        if ((!words.empty()) && (words[0] == "sequence" || words[0] == "s"))
        {
          // If a inner sequence command is found, start the expanding process.
          // Flag that a cascading sequence has been found, which enforces another
          // top level loop to check for new inner sequences.
          innerSequenceFound = true;
          RLOG_CPP(1, "Detected cascading action sequence");

          // Check if the sequence is in fact part of the configuration file.
          bool sequenceFound = false;
          for (const auto& pair : seq.sequences)
          {
            // If we find the sequence, we replace the ALL the occurences
            // with the sequence text as it's defined in the config.
            if (words[1] == pair.first)
            {
              RLOG_CPP(1, "Expanding '" << command << "' into '" << pair.second << "'");

              replaceFirst(text, command, pair.second);

              sequenceFound = true;
              RLOG_CPP(1, "Sequence expanded to '" << text << "'");
              break;
            }
          }

          // TODO (CP) - duplicated code, refactor
          // If we end up here, a sequence with a name has been given, but the
          // sequence was not specified in the configuration file. This is
          // treated as an error.
          if (!sequenceFound)
          {
            std::string explanation = "ERROR: Can't execute action sequence. REASON: Action sequence " +
                                      words[1] + " not found in config file DEVELOPER: '" + text + "'";
            entity.publish("ActionResult", false, 0.0, explanation);
            RLOG_CPP(0, explanation);
            return;
          }
        }
      }
    }
  }

  RLOG_CPP(0, "Sequence expanded to '" << text << "'");

  // From here on, any sequence has been expanded
  actionStack = Rcs::String_split(text, ";");

  // Strip individual actions from white spaces etc.
  for (auto& action : actionStack)
  {
    trim(action);
  }

  entity.publish("TextCommand", actionStack[0]);
}

void ExampleActionsECS::onPrint()
{
  RcsCollisionModel_fprint(stderr, controller->getCollisionMdl());
  ActionFactory::print();
  actionC->getDomain()->print();
}

void ExampleActionsECS::onTrajectoryMoving(bool isMoving)
{
  // This case is true if the trajectory has started. There is nothing to do here.
  if (isMoving)
  {
    return;
  }

  // If the trajectory has finshed, we can report success.
  entity.publish("ActionResult", true, 0.0, std::string("SUCCESS DEVELOPER: ") +
                 std::string(__FILENAME__) + " line " + std::to_string(__LINE__));

  // We unfreeze the scene after the last action has finished
  if (actionStack.empty())
  {
    // In valgrind test mode (no Guis, for memory leak checking), we quit
    // the run loop after the first sequence has ended, since we don't have
    // control over keyboard and mouse.
    runLoop = valgrind ? false : true;

    // If we are running in unittest mode and there are no more actions in the
    // stack we can simply exit the run
    if (unittest)
    {
      RMSG_CPP("Quitting unit tests since actionStack is empty");
      ExampleBase::stop();
    }
  }
  else
  {
    RLOG_CPP(0, "Still " << actionStack.size()
             << " actions to go - publishing next text command: "
             << actionStack[0]);

    if (actionStack.size()>1)
    {
      entity.publish("TextCommand", actionStack[1]);
    }

  }
}

// This only handles the "reset" keyword
void ExampleActionsECS::onTextCommand(std::string text)
{
  if (getRobotEnabled())
  {
    RLOG(0, "Skipping reset during real robot operation");
    return;
  }

  if (STRNEQ(text.c_str(), "reset", 5))
  {
    if (viewer)
    {
      viewer->lock();
    }
    std::vector<std::string> resetCmd = Rcs::String_split(text, " ");

    // Reset from a file. This currently only works if the graph of the file
    // has the same size / topology(?) as the current one.
    if (resetCmd.size()==2)
    {
      RLOG_CPP(0, "Resetting with file " << resetCmd[1]);
      RcsGraph* newGraph = RcsGraph_create(resetCmd[1].c_str());
      if (!newGraph)
      {
        std::string errStr = "ERROR REASON: Reset with xml file '" + resetCmd[1] +
                             "' failed. SUGGESTION: Check if the file name is correct";
        entity.publish("ActionResult", false, 0.0, errStr +
                       std::string(__FILENAME__) + " line " + std::to_string(__LINE__));

        if (viewer)
        {
          viewer->unlock();
        }
        return;
      }

      RLOG_CPP(0, "Reloading ActionScene with file " << resetCmd[1]);
      actionC->getDomain()->reload(resetCmd[1]);
      RLOG_CPP(0, "Done reloading ActionScene with file " << resetCmd[1]);

      RcsGraph_destroy(graphToInitializeWith);
      graphToInitializeWith = newGraph;
      entity.publish("ReloadGraph", std::string("IK"), (const RcsGraph*)graphToInitializeWith);
    }


    RLOG(0, "Received reset command");
    entity.publish("ClearTrajectory");
    entity.publish("InitFromState", (const RcsGraph*)graphToInitializeWith);
    entity.publish("EmergencyRecover");
    entity.setTime(0.0);
    entity.publish("ActionResult", true, 0.0, std::string("SUCCESS DEVELOPER: Reset succeeded ") +
                   std::string(__FILENAME__) + " line " + std::to_string(__LINE__));

    // The ActionScene is reloaded, since otherwise fill levels etc. remain as
    // they are.
    RLOG(0, "Reloading scene from %s", controller->getGraph()->cfgFile);
    actionC->getDomain()->reload(controller->getGraph()->cfgFile);

    if (actionStack.size()>1)
    {
      entity.publish("TextCommand", actionStack[1]);
    }

    if (viewer)
    {
      viewer->unlock();
    }

    return;
  }

}

void ExampleActionsECS::setEnableRobot(bool enable)
{
  RMSG("***** Real Robot in the loop - speedUp resetted to 1 *****");
  speedUp = 1;
  withRobot = enable;
}

void ExampleActionsECS::addComponent(ComponentBase* component)
{
  if (component)
  {
    components.push_back(component);
  }
}

void ExampleActionsECS::addHardwareComponent(ComponentBase* component)
{
  if (component)
  {
    hwc.push_back(component);
    setEnableRobot(true);
  }
}

bool ExampleActionsECS::getRobotEnabled() const
{
  return withRobot;
}

ActionScene* ExampleActionsECS::getScene()
{
  return actionC ? actionC->getDomain() : nullptr;
}

const ActionScene* ExampleActionsECS::getScene() const
{
  return actionC ? actionC->getDomain() : nullptr;
}

void ExampleActionsECS::startThreaded()
{
  std::thread t1([&]
  {
    ExampleBase::start();

    RLOG(0, "ExampleActionsECS thread says good bye");
  });
  t1.detach();
}


std::vector<std::pair<std::string,std::string>> ExampleActionsECS::getCompletedActionStack() const
{
  std::lock_guard<std::mutex> lock(actionStackMtx);
  return completedActionStack;
}

void ExampleActionsECS::clearCompletedActionStack()
{
  std::lock_guard<std::mutex> lock(actionStackMtx);
  completedActionStack.clear();
}

void ExampleActionsECS::addToCompletedActionStack(std::string action, std::string result)
{
  std::lock_guard<std::mutex> lock(actionStackMtx);
  completedActionStack.push_back(std::make_pair(action, result));
}

void ExampleActionsECS::printCompletedActionStack() const
{
  std::lock_guard<std::mutex> lock(actionStackMtx);

  RMSG("completedActionStack has %zu entries", completedActionStack.size());
  for (size_t i=0; i<completedActionStack.size(); ++i)
  {
    RMSG("completedActionStack[%zu]: Action='%s' result='%s'",
         i, completedActionStack[i].first.c_str(), completedActionStack[i].second.c_str());
  }
}

/*******************************************************************************
 * Replace the first occurence of a string in another string.
 * The replacement is aware of word boundaries, i.e. it will not replace
 * "s 1" in "s 11".
 ******************************************************************************/
void replaceFirst(std::string& str, const std::string& search, const std::string& replace)
{
  size_t pos = 0;
  const size_t search_len = search.length();
  const size_t replace_len = replace.length();

  while ((pos = str.find(search, pos)) != std::string::npos)
  {
    // Check if the found occurrence is not part of a bigger sequence
    // E.g. for "s 1" we don't want to replace "s 11"
    if ((pos == 0 || !std::isalnum(str[pos - 1])) &&
        (pos + search_len == str.length() || !std::isalnum(str[pos + search_len])))
    {
      str.replace(pos, search_len, replace);
      pos += replace_len;
      // quit after 1st replacement
      break;
    }
    else
    {
      // Move past the found occurrence without replacing
      pos += search_len;
    }
  }
}

}   // namespace aff
