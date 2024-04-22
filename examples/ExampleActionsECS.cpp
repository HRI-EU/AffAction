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
#include "AnimationSequence.h"
#include "HardwareComponent.h"
#include "SceneJsonHelpers.h"

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


#define NUM_SCENEQUERIES (5)


void AffActionExampleInfo()
{
  Rcs::ExampleFactory::print();
}

namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
void onSetLogLevel(int dl)
{
  RcsLogLevel = dl;
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
  lookaheadCount = 0;

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
  verbose = true;
  processingAction = false;
  lookahead = false;
  turbo = true;
  earlyExit = false;
  depthFirst = false;

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
  xmlFileName = "g_group_6.xml";
  config_directory = "config/xml/AffAction/xml/examples";
  speedUp = 3;

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
  parser->getArgument("-verbose", &verbose, "Print debug information to console");
  parser->getArgument("-unittest", &unittest, "Run unit tests");
  parser->getArgument("-singleThreaded", &singleThreaded, "Run predictions sequentially");
  parser->getArgument("-sequence", &sequenceCommand, "Sequence command to start with");
  parser->getArgument("-lookahead", &lookahead, "Perform lookahead for whole sequences");
  parser->getArgument("-lookaheadCount", &lookaheadCount, "Number of actions to lookahead");
  parser->getArgument("-turbo", &turbo, "Compute action duration to be as fast as possible");
  parser->getArgument("-earlyExit", &earlyExit, "Quit on first encountered solution when expanding DFS tree");
  parser->getArgument("-depthFirst", &depthFirst, "Use depth-dirst tree search");

  // This is just for pupulating the parsed command line arguments for the help
  // functions / help window.
  const bool dryRun = true;
  createHardwareComponents(entity, NULL, NULL, dryRun);
  createComponents(entity, NULL, NULL, dryRun);

  if (parser->hasArgument("-h"))
  {
    std::cout << help() << std::endl;
    return false;
  }

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
  entity.subscribe("SetLogLevel", &onSetLogLevel);
  entity.subscribe("Quit", &ExampleActionsECS::onQuit, this);
  entity.subscribe("Print", &ExampleActionsECS::onPrint, this);
  entity.subscribe("TrajectoryMoving", &ExampleActionsECS::onTrajectoryMoving, this);
  entity.subscribe("ActionSequence", &ExampleActionsECS::onActionSequence, this);
  entity.subscribe<bool, double, std::string>("ActionResult", &ExampleActionsECS::onActionResult, this);
  entity.subscribe("PlanActionSequence", &ExampleActionsECS::onPlanActionSequence, this);
  entity.subscribe("PlanDFS", &ExampleActionsECS::onPlanActionSequenceDFS, this);
  entity.subscribe("PlanDFSEE", &ExampleActionsECS::onPlanActionSequenceDFSEE, this);
  entity.subscribe("TextCommand", &ExampleActionsECS::onTextCommand, this);
  entity.subscribe("FreezePerception", &ExampleActionsECS::onChangeBackgroundColorFreeze, this);
  entity.subscribe("Process", &ExampleActionsECS::onProcess, this);

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
        RcsBroadPhase* bp = NULL;
        if (noCollCheck)
        {
          bp = RcsBroadPhase_create(controller->getGraph(), 0.0);
        }
        else
        {
          bp = RcsBroadPhase_createFromXML(controller->getGraph(), child);
        }
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
  actionC->setEarlyExitPrediction(true);

#if 1
  // Misuse contacts shape flag for Collision trajectory constraint
  for (size_t i=0; i<actionC->getDomain()->entities.size(); ++i)
  {
    RcsBody* ntt = actionC->getDomain()->entities[i].body(controller->getGraph());

    // If we find one or more shapes that have the distance flag active, we
    // set this flag to true, and set all shapes to be resizeable (except for
    // meshes), so that they can be visualized as wireframe when not collideable.
    bool hasToggleShape = false;
    for (unsigned int i=0; i<ntt->nShapes; ++i)
    {
      RcsShape* sh = &ntt->shapes[i];
      if (RcsShape_isOfComputeType(sh, RCSSHAPE_COMPUTE_DISTANCE))
      {
        RcsShape_setComputeType(sh, RCSSHAPE_COMPUTE_CONTACT, true);
        hasToggleShape = true;
      }
    }

    if (hasToggleShape)
    {
      for (unsigned int i=0; i<ntt->nShapes; ++i)
      {
        RcsShape* sh = &ntt->shapes[i];
        if (sh->type == RCSSHAPE_MESH)
        {
          continue;
        }
        RcsShape_setComputeType(sh, RCSSHAPE_COMPUTE_RESIZEABLE, true);
      }
    }

  }
#endif

  // Graph component contains "sensed" graph
  graphC = std::make_unique<aff::GraphComponent>(&entity, controller->getGraph());
  graphC->setEnableRender(false);//\todo MG CHECK
  trajC = std::make_unique<aff::TrajectoryComponent>(&entity, controller.get(), !zigzag, 1.0,
                                                     !noTrajCheck);

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
  this->hwc = createHardwareComponents(entity, controller->getGraph(), actionC->getDomain(), false);
  this->components = createComponents(entity, controller->getGraph(), actionC->getDomain(), false);

  addComponent(new AnimationSequence(&entity, controller->getGraph()));
  if (!hwc.empty())
  {
    setEnableRobot(true);
    RCHECK_MSG(noCollCheck==false, "You are running the real system without collision detection");
  }

  // Initialization sequence to initialize all graphs from the sensory state
  entity.initialize(graphC->getGraph());

  this->sceneQuery = std::make_unique<SceneQueryPool>(this, NUM_SCENEQUERIES);

  ActionBase::setTurboMode(turbo);

  RLOG_CPP(0, help());

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
    RLOG(1, "Running without graphics");
    return true;
  }

  viewer = std::make_unique<aff::GraphicsWindow>(&entity, false);
  viewer->add(new NamedMouseDragger(controller->getGraph()));
  viewer->setTitle("ExampleActionsECS");

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

  viewer->setKeyCallback('T', [this](char k)
  {
    RLOG(0, "Zap visualization channel");
    entity.publish("ZapAnimation");
  }, "Zap through animation channels");

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
    RLOG(0, "Test occlusions");
    nlohmann::json json = getObjectOccludersForAgent("Daniel", "fanta_bottle", actionC->getDomain(),
                                                     controller->getGraph());
    RLOG_CPP(0, "getOccludersForAgent(Daniel, fanta_bottle):\n" << json.dump(4));

    json = getOccludedObjectsForAgent("Daniel", actionC->getDomain(), controller->getGraph());
    RLOG_CPP(0, "getOccludedObjectsForAgent(Daniel):\n" << json.dump(4));

  }, "Test occlusions");

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

  //viewer->setKeyCallback('T', [this](char k)
  //{
  //  trajC->getTrajectoryController()->toXML("RobjectTrajectory.xml");
  //  trajC->getTrajectoryController()->getController()->toXML("RobotController.xml");

  //}, "Write trajectory to file \"RobjectTrajectory.xml\" and controller to \"RobotController.xml\"");

  viewer->setKeyCallback('o', [this](char k)
  {
    RLOG(0, "Launching ControllerGui");
    controller->toXML("onPressedButtonO.xml");
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

  viewer->setKeyCallback('E', [this](char k)
  {
    Rcs::BodyNode* bn = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    if (!bn)
    {
      return;
    }

    auto ntt = getScene()->getAffordanceEntity(bn->body()->name);
    if (ntt)
    {
      RMSG_CPP("Entity: " << ntt->name);
    }
    else
    {
      auto agent = getScene()->getAgent(bn->body()->name);
      if (agent)
      {
        RMSG_CPP("Agent: " << agent->name);
      }
      else
      {
        RMSG_CPP("No entity or agent for: " << bn->body()->name);
      }
    }

  }, "Get body under mouse");

  viewer->setKeyCallback('m', [this](char k)
  {
    Rcs::BodyNode* bn = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    if (!bn)
    {
      return;
    }

    const ActionScene* scene = actionC->getDomain();
    std::vector<const Manipulator*> om = scene->getOccupiedManipulators(controller->getGraph());
    RLOG_CPP(0, "Found " << om.size() << " occupied manipulators");

    if (om.empty())
    {
      std::string textCmd = "get " + std::string(bn->body()->name);
      entity.publish("ActionSequence", textCmd);
    }
    else
    {
      std::vector<const AffordanceEntity*> ge = om[0]->getGraspedEntities(*scene, controller->getGraph());
      RCHECK_MSG(!ge.empty(), "For manipulator: '%s'", om[0]->name.c_str());

      std::string textCmd = "put " + ge[0]->name + " " + std::string(bn->body()->name);
      entity.publish("ActionSequence", textCmd);
    }

  }, "Get body under mouse");

  viewer->setKeyCallback('a', [this](char k)
  {
    RLOG(0, "Toggling talk flag");
    //entity.publish("ToggleASR");
    entity.publish("ToggleHandRaised");

  }, "Toggle talk flag");

  viewer->setKeyCallback('l', [this](char k)
  {
    RLOG(0, "Resetting LLM memory");
    entity.publish("ResetLLM");

  }, "Resetting LLM memory");

  viewer->setKeyCallback('p', [this](char k)
  {
    RLOG(0, "Replaying log");
    entity.publish("ReplayLog");

  }, "Replaying log");

  viewer->setKeyCallback('A', [this](char k)
  {
    RLOG(0, "Test pan tilt angle calculation");

    double t_calc = Timer_getSystemTime();
    std::vector<double> panTilt = sceneQuery->instance()->getPanTilt("robot", "glass_blue");


    // Agent* robo_ = getScene()->getAgent("robot");
    // RCHECK(robo_);

    // double panTilt[2], err[2];
    // size_t maxIter = 100;
    // double eps = 1.0e-8;
    // RobotAgent* robo = dynamic_cast<RobotAgent*>(robo_);

    // double t_calc = Timer_getSystemTime();
    // int iter = robo->getPanTilt(controller->getGraph(), "table", panTilt, maxIter, eps, err);
    t_calc = Timer_getSystemTime() - t_calc;

    // RLOG(0, "pan=%.1f tilt=%.1f err=%f %f took %d iterations and %.1f msec",
    //      RCS_RAD2DEG(panTilt[0]), RCS_RAD2DEG(panTilt[1]), err[0], err[1], iter, 1.0e3*t_calc);
    if (!panTilt.empty())
      RLOG(0, "pan=%.1f tilt=%.1f took %.1f msec",
           RCS_RAD2DEG(panTilt[0]), RCS_RAD2DEG(panTilt[1]), 1.0e3 * t_calc);

  }, "Test pan tilt angle calculation");


  viewer->start();

  entity.publish("RenderCommand", std::string("ShowLines"), std::string("false"));
  entity.publish("RenderCommand", std::string("Physics"), std::string("hide"));
  entity.publish("RenderCommand", std::string("IK"), std::string("show"));
  entity.publish("RenderCommand", std::string("IK"), std::string("unsetGhostMode"));

  if (!hwc.empty())
  {
    entity.publish("BackgroundColor", std::string("PEWTER"));
  }

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

  if (valgrind)
  {
    // get fanta_bottle;put fanta_bottle lego_box;pose default
    std::vector<std::string> seqCmd;
    seqCmd.push_back("get bottle_of_cola");
    seqCmd.push_back("pour bottle_of_cola glass_blue");
    seqCmd.push_back("put bottle_of_cola");

    std::string errMsg;
    auto tree = PredictionTree::planActionTreeDFT(*getScene(), getGraph(), getBroadPhase(),
                                                  seqCmd, seqCmd.size(), 5, entity.getDt(),
                                                  false, errMsg);

    tree->toDotFile("PredictionTree.dot");
    // auto res = sceneQuery->instance()->planActionSequence(seqCmd, seqCmd.size());
    // std::string newCmd;
    // for (size_t i = 0; i < res.size(); ++i)
    // {
    //   newCmd += res[i];
    //   newCmd += ";";
    // }

    // RLOG_CPP(0, "Command : " << newCmd);
    runLoop = false;
  }

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
           "failCount: %zu queue: %zu (max: %zu)  End time: %.3f %.3f"
           "\nFinal pose: %s",
           entity.getTime(), dtProcess * 1.0e3, dt_max * 1.0e3, dt_max2 * 1.0e3,
           failCount, entity.queueSize(), entity.getMaxQueueSize(),
           endTime, trajTime, isFinalPoseRunning() ? "YES" : "NO");
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
  s << std::endl << "Hardware concurrency: " << std::thread::hardware_concurrency() << std::endl;
  s << "Turbo mode: " << (turbo ? "ON" : "OFF") << std::endl;
  s << "Graph size[bytes]: " << RcsGraph_sizeInBytes(controller->getGraph()) << std::endl;

  return s.str();
}

void ExampleActionsECS::onQuit()
{
  entity.publish("Stop");
  runLoop = false;
}

static void _planActionSequenceThreaded(aff::ExampleActionsECS* ex,
                                        std::string sequenceCommand,
                                        size_t maxNumThreads,
                                        bool depthFirst,
                                        bool earlyExit)
{
  std::vector<std::string> seq = Rcs::String_split(sequenceCommand, ";");

  std::unique_ptr<PredictionTree> tree;

  if (depthFirst)
  {
    tree = ex->getQuery()->planActionTreeDFT(seq, maxNumThreads, earlyExit);
  }
  else
  {
    tree = ex->getQuery()->planActionTree(seq, maxNumThreads);
  }

  std::vector<std::string> predictedActions;

  if (tree)
  {
    auto sln = tree->findSolutionPath();
    for (const auto& nd : sln)
    {
      predictedActions.push_back(nd->actionCommand());
      RLOG_CPP(0, "Action: " << nd->actionCommand() << " cost: " << nd->cost);
    }
  }

  // Animation of all valid solution paths. We only show the first 3 so that we
  // don't copy around huge amounts of memory.
  if (tree)
  {
    size_t numSolutions = std::min(tree->getNumValidPaths(), (size_t)3);
    std::vector<TrajectoryPredictor::PredictionResult> predictions(numSolutions);

    for (size_t i = 0; i < predictions.size(); ++i)
    {
      predictions[i].success = true;
      auto path = tree->findSolutionPath(i);
      for (const auto& nd : path)
      {
        predictions[i].bodyTransforms.insert(predictions[i].bodyTransforms.end(),
                                             nd->bodyTransforms.begin(),
                                             nd->bodyTransforms.end());
      }

    }

    ex->entity.publish("AnimateSequence", predictions, 0);
  }

  if (predictedActions.empty())
  {
    RLOG_CPP(0, "Could not find solution");
    ex->processingAction = false;
    ex->clearCompletedActionStack();
    return;
  }

  RLOG_CPP(0, "Sequence has " << predictedActions.size() << " steps");

  std::string newCmd;
  for (size_t i = 0; i < predictedActions.size(); ++i)
  {
    newCmd += predictedActions[i];
    newCmd += ";";
  }

  RLOG_CPP(0, "Command : " << newCmd);
  ex->entity.publish("ActionSequence", newCmd);
}

void ExampleActionsECS::onPlanActionSequence(std::string sequenceCommand)
{
  processingAction = true;
  const size_t maxNumthreads = 0;   // 0 means auto-select
  bool dfs = false;
  sequenceCommand = ActionSequence::resolve(controller->getGraph()->cfgFile, sequenceCommand);
  std::thread t1(_planActionSequenceThreaded, this, sequenceCommand, maxNumthreads, dfs, false);
  t1.detach();
}

void ExampleActionsECS::onPlanActionSequenceDFS(std::string sequenceCommand)
{
  processingAction = true;
  const size_t maxNumthreads = 0;   // 0 means auto-select
  bool dfs = true;
  sequenceCommand = ActionSequence::resolve(controller->getGraph()->cfgFile, sequenceCommand);
  std::thread t1(_planActionSequenceThreaded, this, sequenceCommand, maxNumthreads, dfs, false);
  t1.detach();
}

void ExampleActionsECS::onPlanActionSequenceDFSEE(std::string sequenceCommand)
{
  processingAction = true;
  const size_t maxNumthreads = 0;   // 0 means auto-select
  bool dfs = true;
  sequenceCommand = ActionSequence::resolve(controller->getGraph()->cfgFile, sequenceCommand);
  std::thread t1(_planActionSequenceThreaded, this, sequenceCommand, maxNumthreads, dfs, true);
  t1.detach();
}

void ExampleActionsECS::onActionSequence(std::string text)
{
  processingAction = true;

  if (verbose)
  {
    RMSG_CPP("RECEIVED: " << text);
  }
  clearCompletedActionStack();

  // Early exit in case we receive an empty string
  if (text.empty())
  {
    std::string explanation = "ERROR REASON: Received empty action command string. SUGGESTION: Check your spelling DEVELOPER: '" +
                              text + "' " + std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
    entity.publish("ActionResult", false, 0.0, explanation);
    return;
  }

  text = ActionSequence::resolve(getGraph()->cfgFile, text);

  // From here on, any sequence has been expanded
  RLOG_CPP(1, "Sequence expanded to '" << text << "'");

  // // From here on, any sequence has been expanded
  actionStack = Rcs::String_split(text, ";");

  // Strip individual actions from white spaces etc.
  for (auto& action : actionStack)
  {
    Rcs::String_trim(action);
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
      if (verbose)
      {
        RMSG_CPP("Quitting unit tests since actionStack is empty");
      }
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
  if (text=="get_state")
  {
    std::thread feedbackThread([this]()
    {
      std::string fb = sceneQuery->instance()->getSceneState().dump();
      entity.publish("SendWebsocket", fb);
    });
    feedbackThread.detach();
  }

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
      getScene()->reload(resetCmd[1]);
      getScene()->initializeKinematics(newGraph);   // Reach and shoulder joints
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
    getScene()->reload(controller->getGraph()->cfgFile);
    getScene()->initializeKinematics(controller->getGraph());   // Reach and shoulder joints

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

void ExampleActionsECS::onActionResult(bool success, double quality, std::string resMsg)
{
  if (verbose)
  {
    RMSG_CPP(resMsg);
  }
  entity.publish("SetTextLine", resMsg, 1);

  lastResultMsg = resMsg;

  // This needs to be done independent of failure or success
  if (!actionStack.empty())
  {
    addToCompletedActionStack(actionStack[0], resMsg);

    REXEC(1)
    {
      printCompletedActionStack();
    }

    actionStack.erase(actionStack.begin());
  }

  if (!success)
  {
    failCount++;

    if (unittest)
    {
      if (!actionStack.empty())
      {
        // \todo: Check why this works
        //RPAUSE_MSG("Calling reset");
        entity.publish("TextCommand", std::string("reset"));
        actionStack.insert(actionStack.begin(), "reset");
      }
    }
    else
    {
      actionStack.clear();
    }
    RLOG_CPP(0, "Clearing action stack, number of failed actions: " << failCount);
  }

  // After the last action command of a sequence has been finished (the
  // trajectory has ended), or we face a failure, we "unfreeze" the perception
  // and accept updates from the perceived scene.
  if (actionStack.empty())
  {
    RLOG(0, "ActionStack is empty, unfreezing perception");
    entity.publish("FreezePerception", false);
    processingAction = false;
    actionC->setFinalPoseRunning(false);
  }

  if ((actionStack.size()==1) && STRNEQ(actionStack[0].c_str(), "pose", 4))
  {
    actionC->setFinalPoseRunning(true);
  }

  // In valgrind mode, we only perform one action, since this might run with valgrind
  if (valgrind || (unittest && actionStack.empty()))
  {
    ExampleBase::stop();
  }
  else
  {
    entity.publish("SendWebsocket", resMsg);
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

RcsGraph* ExampleActionsECS::getGraph()
{
  return controller ? controller->getGraph() : nullptr;
}

const RcsGraph* ExampleActionsECS::getGraph() const
{
  return controller ? controller->getGraph() : nullptr;
}

RcsBroadPhase* ExampleActionsECS::getBroadPhase()
{
  return controller ? controller->getBroadPhase() : nullptr;
}

const RcsBroadPhase* ExampleActionsECS::getBroadPhase() const
{
  return controller ? controller->getBroadPhase() : nullptr;
}

std::shared_ptr<ConcurrentSceneQuery> ExampleActionsECS::getQuery()
{
  return sceneQuery ? sceneQuery->instance() : nullptr;
}

GraphicsWindow* ExampleActionsECS::getViewer()
{
  return viewer.get();
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

void ExampleActionsECS::onChangeBackgroundColorFreeze(bool freeze)
{
  std::string color = freeze ? "BLACK" : "";
  entity.publish<std::string, std::string>("RenderCommand", "BackgroundColor", color);
}

void ExampleActionsECS::onProcess()
{
  entity.process();
}

bool ExampleActionsECS::isFinalPoseRunning() const
{
  RCHECK(actionC);
  return actionC->isFinalPoseRunning();
}

size_t ExampleActionsECS::getNumFailedActions() const
{
  return failCount;
}

void ExampleActionsECS::lockStepMtx() const
{
  stepMtx.lock();
}

void ExampleActionsECS::unlockStepMtx() const
{
  stepMtx.unlock();
}

}   // namespace aff
