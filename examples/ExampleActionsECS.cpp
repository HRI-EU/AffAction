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
#include "LandmarkZmqComponent.h"
#include "PhysicsComponent.h"
#include "VirtualCameraWindow.h"
#include "ActionEyeGaze.h"

#include <EventGui.h>
#include <ConstraintFactory.h>
#include <ActivationSet.h>

#include <ForceDragger.h>
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
#include <Rcs_graphicsUtils.h>

#include <CmdLineWidget.h>

#include <BodyPointDragger.h>
#include <MouseDragger.h>
#include <AABBNode.h>
#include <SphereNode.h>
#include <BodyNode.h>
#include <GraphNode.h>
#include <PhysicsNode.h>

#include <fstream>
#include <iostream>
#include <thread>


#define NUM_SCENEQUERIES (5)
#define FIRST_N_SOLUTIONS (4)


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
class NamedBodyForceDragger : public Rcs::ForceDragger
{
public:

  NamedBodyForceDragger(Rcs::PhysicsBase* sim) : Rcs::ForceDragger(sim)
  {
  }

  virtual void update()
  {
    double f[3];
    Vec3d_sub(f, _I_mouseTip, _I_anchor);
    Vec3d_constMulSelf(f, getForceScaling() * (_leftControlPressed ? 10.0 : 1.0));

    RcsBody* simBdy = NULL;
    if (_draggedBody)
    {
      simBdy = RcsGraph_getBodyByName(physics->getGraph(), _draggedBody->name);
      NLOG(0, "Graph addy: 0x%x", physics->getGraph());
      NLOG(0, "Dragging %s to: [%f, %f, %f]", simBdy->name, f[0], f[1], f[2]);
    }
    physics->applyForce(simBdy, f, _k_anchor);

  }
};



/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleActionsECS, "Actions", "Attentive Support");

ExampleActionsECS::ExampleActionsECS() : ExampleActionsECS(0, NULL)
{
}

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
  lambda = 1.0e-4;
  virtualCameraWidth = 640;
  virtualCameraHeight = 480;
  virtualCameraEnabled = false;
  virtualCameraWindowEnabled = false;
  speedUp = 1;
  loopCount = 0;
  maxNumThreads = 0;

  pause = false;
  noSpeedCheck = false;
  noJointCheck = false;
  noCollCheck = false;
  noTrajCheck = false;
  hasBeenStopped = false;

  noLimits = false;
  zigzag = false;
  withEventGui = false;
  noViewer = false;
  noTextGui = false;
  plot = false;
  valgrind = false;
  unittest = false;
  withRobot = false;
  singleThreaded = false;
  verbose = true;
  processingAction = false;
  turbo = true;
  earlyExitAction = true;

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

  actionC = nullptr;
  graphC = nullptr;
  trajC = nullptr;
  ikc = nullptr;
  textGui = nullptr;
}

ExampleActionsECS::~ExampleActionsECS()
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
  RLOG_CPP(0, "Done deleting ExampleActionsECS");
}

bool ExampleActionsECS::initParameters()
{
  xmlFileName = "g_attentive_support.xml";
  configDirectory = "config/xml/AffAction/xml/examples";
  speedUp = 3;

  return true;
}

bool ExampleActionsECS::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  parser->getArgument("-speedUp", &speedUp, "Speed-up factor (default: %d)", speedUp);
  parser->getArgument("-dt", &dt, "Time step (default is %f)", dt);
  parser->getArgument("-f", &xmlFileName, "Configuration file name "
                      "(default is %s)", xmlFileName.c_str());
  parser->getArgument("-dir", &configDirectory, "Configuration file directory "
                      "(default is %s)", configDirectory.c_str());
  parser->getArgument("-alpha", &alpha, "Null space scaling factor (default is %f)", alpha);
  parser->getArgument("-lambda", &lambda, "Regularization (default: %f)", lambda);
  parser->getArgument("-pause", &pause, "Pause after each process() call");
  parser->getArgument("-nospeed", &noSpeedCheck, "No speed limit checks");
  parser->getArgument("-nojl", &noJointCheck, "Disable joint limit checks");
  parser->getArgument("-nocoll", &noCollCheck, "Disable collision checks");
  parser->getArgument("-notrajcheck", &noTrajCheck, "Disable trajectory checks");
  parser->getArgument("-noLimits", &noLimits, "Ignore all robot limits");
  parser->getArgument("-zigzag", &zigzag, "Zig-zag trajectory");
  parser->getArgument("-eventGui", &withEventGui, "Launch event gui");
  parser->getArgument("-noViewer", &noViewer, "Do not launch viewer");
  parser->getArgument("-noTextGui", &noTextGui, "Do not launch TextGui");
  parser->getArgument("-plot", &plot, "Enable debug plotting");
  parser->getArgument("-valgrind", &valgrind, "Valgrind mode without graphics and Gui");
  parser->getArgument("-verbose", &verbose, "Print debug information to console");
  parser->getArgument("-unittest", &unittest, "Run unit tests");
  parser->getArgument("-singleThreaded", &singleThreaded, "Run predictions sequentially");
  parser->getArgument("-physics", &physicsEngine, "Physics engine (default: none)");
  parser->getArgument("-enableVirtualCamera", &virtualCameraEnabled, "Enable camera for virtual rendering");
  parser->getArgument("-enableVirtualCamWindow", &virtualCameraWindowEnabled, "Window of the camera for virtual rendering");
  parser->getArgument("-virtualCameraBodyName", &virtualCameraBodyName, "Name of body in graph to which camera is attached");
  parser->getArgument("-sequence", &sequenceCommand, "Sequence command to start with");
  parser->getArgument("-turbo", &turbo, "Compute action duration to be as fast as possible");
  parser->getArgument("-maxNumThreads", &maxNumThreads, "Max. number of threads for planning");
  parser->getArgument("-earlyExitAction", &earlyExitAction, "Early exit with action prediction's first error");

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

bool ExampleActionsECS::initAlgo()
{
  Rcs_addResourcePath(RCS_CONFIG_DIR);
  Rcs_addResourcePath(configDirectory.c_str());

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
  entity.subscribe("SetAnimationMode", &TrajectoryPredictor::setAnimationMode);
  entity.subscribe("Quit", &ExampleActionsECS::onQuit, this);
  entity.subscribe("Print", &ExampleActionsECS::onPrint, this);
  entity.subscribe("TrajectoryMoving", &ExampleActionsECS::onTrajectoryMoving, this);
  entity.subscribe("ActionSequence", &ExampleActionsECS::onActionSequence, this);
  entity.subscribe("ActionResult", &ExampleActionsECS::onActionResult, this);
  entity.subscribe("PlanActionSequence", &ExampleActionsECS::onPlanActionSequenceBFS, this);
  entity.subscribe("PlanDFS", &ExampleActionsECS::onPlanActionSequenceDFS, this);
  entity.subscribe("PlanDFSEE", &ExampleActionsECS::onPlanActionSequenceDFSEE, this);
  entity.subscribe("TextCommand", &ExampleActionsECS::onTextCommand, this);
  entity.subscribe("FreezePerception", &ExampleActionsECS::onChangeBackgroundColorFreeze, this);
  entity.subscribe("Process", &ExampleActionsECS::onProcess, this);
  entity.subscribe("SetTurboMode", &ExampleActionsECS::onSetTurboMode, this);
  entity.subscribe("ClearTrajectory", &ExampleActionsECS::onClearTrajectory, this);
  entity.subscribe("SetPupilSpeedWeight", &ExampleActionsECS::onSetPupilSpeedWeight, this);

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
    xmlNodePtr node = parseXMLFile(getGraph()->cfgFile, NULL, &doc);
    if (node)
    {
      xmlNodePtr child = getXMLChildByName(node, "CollisionModel");
      if (child)
      {
        RcsCollisionMdl* cMdl = RcsCollisionModel_createFromXML(getGraph(), child);
        controller->setCollisionMdl(cMdl);
      }


      child = getXMLChildByName(node, "BroadPhase");
      if (child)
      {
        RcsBroadPhase* bp = NULL;
        if (noCollCheck)
        {
          bp = RcsBroadPhase_create(getGraph(), 0.0);
        }
        else
        {
          bp = RcsBroadPhase_createFromXML(getGraph(), child);
        }
        RcsBroadPhase_updateBoundingVolumes(bp);
        controller->setBroadPhase(bp);

        if (!controller->getCollisionMdl())
        {
          RcsCollisionMdl* cMdl = RcsCollisionModel_create(getGraph());
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

  actionC = new aff::ActionComponent(&entity, getGraph(), controller->getBroadPhase());
  actionC->setLimitCheck(!noLimits);
  actionC->setMultiThreaded(!singleThreaded);
  actionC->setEarlyExitPrediction(true);
  addComponent(actionC);

#if 1
  // Misuse contacts shape flag for Collision trajectory constraint
  for (size_t i=0; i< getScene()->entities.size(); ++i)
  {
    RcsBody* ntt = getScene()->entities[i].body(getGraph());

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
        if (sh->type == RCSSHAPE_MESH || sh->type == RCSSHAPE_REFFRAME)
        {
          continue;
        }
        RcsShape_setComputeType(sh, RCSSHAPE_COMPUTE_RESIZEABLE, true);
      }
    }

  }
#endif

  // Graph component contains "sensed" graph
  graphC = new aff::GraphComponent(&entity, getGraph());
  graphC->setEnableRender(false);
  addComponent(graphC);

  trajC = new aff::TrajectoryComponent(&entity, controller.get(), !zigzag, 1.0,
                                       !noTrajCheck);
  addComponent(trajC);

  // Inverse kinematics controller, no constraints, right inverse
  ikc = new aff::IKComponent(&entity, controller.get(), ikType);
  ikc->setEnableSpeedAccelerationLimit(!noSpeedCheck);
  ikc->setSpeedLimitCheck(!noSpeedCheck);
  ikc->setJointLimitCheck(!noJointCheck);
  ikc->setCollisionCheck(!noCollCheck);
  ikc->setLambda(lambda);
  ikc->setAlpha(alpha);
  addComponent(ikc);

  RCHECK(controller.get() == trajC->getTrajectoryController()->getController());

  // Remember the state for re-initialization
  graphToInitializeWith = RcsGraph_clone(getGraph());

  if (!physicsEngine.empty())
  {
    componentArgs += " -physics " + physicsEngine;
  }

  // Initialize robot components from command line and componentArgs
  auto cTmp = createHardwareComponents(entity, getGraph(), getScene(), false, componentArgs);
  this->hwc.insert(hwc.end(), cTmp.begin(), cTmp.end());
  cTmp = createComponents(entity, getGraph(), getScene(), false, componentArgs);
  this->components.insert(components.end(), cTmp.begin(), cTmp.end());

  addComponent(new AnimationSequence(&entity, getGraph()));
  if (!hwc.empty())
  {
    setEnableRobot(true);
    RCHECK_MSG(noCollCheck==false, "You are running the real system without collision detection");
  }

  // Initialization sequence to initialize all graphs from the sensory state. This also triggers the
  // "Start" event, starting all component threads.
  entity.initialize(getCurrentGraph());

  this->sceneQuery = std::make_unique<SceneQueryPool>(this, NUM_SCENEQUERIES);

  ActionBase::setTurboMode(turbo);

  // Initialization of the virtual camera to be able to render the scene from python
  if (virtualCameraWindowEnabled)
  {
    virtualCameraEnabled = true;
  }

  if (virtualCameraEnabled)
  {
    virtualCamera = std::make_unique<VirtualCamera>(new Rcs::GraphNode(getCurrentGraph()),
                                                    virtualCameraWidth, virtualCameraHeight);
  }

  // Printing the help prompt
  RLOG_CPP(1, help());

  //{
  //  const RcsGraph* graph = getGraph();
  //  for (unsigned int i = 0; i < graph->nBodies; ++i)
  //  {
  //    const RcsBody* bdy = &graph->bodies[i];

  //    for (unsigned int j = 0; j < bdy->nShapes; ++j)
  //    {
  //      const RcsShape* sh = &bdy->shapes[j];
  //      if (RcsShape_isOfComputeType(sh, RCSSHAPE_COMPUTE_RESIZEABLE))
  //      {
  //        RLOG(0, "Found resizeable shape %d in body %s (type: %s)",
  //             j, bdy->name, RcsShape_name(sh->type));
  //      }
  //    }
  //  }
  //}

  return true;
}

bool ExampleActionsECS::initGraphics()
{
  if (viewer)
  {
    RLOG(1, "Graphics already initialized");
    return false;
  }

  if (virtualCameraWindowEnabled && virtualCamera)
  {
    double trf6[6] = {-1.836150, -2.665913, 2.790988, -0.537332, 0.374638, 1.143258};
    HTr A_CI;
    HTr_from6DVector(&A_CI, trf6);

    if (!virtualCameraBodyName.empty())
    {
      const RcsBody* camBdy = RcsGraph_getBodyByName(getCurrentGraph(), virtualCameraBodyName.c_str());
      RCHECK_MSG(camBdy, "Unknown body for camera: %s", virtualCameraBodyName.c_str());
      HTr_copy(&A_CI, &camBdy->A_BI);
    }
    addComponent(new VirtualCameraWindow(&entity, virtualCamera.get(), true, false, &A_CI));
  }

  // Optional graphics window. We don't use a static instance since this will
  // not be cleaned up after exiting main.
  if (noViewer || valgrind)
  {
    RLOG(1, "Running without graphics");
    return true;
  }

  bool viewerStartsWithStartEvent = true;
  viewer = std::make_unique<aff::GraphicsWindow>(&entity, viewerStartsWithStartEvent);

  // Add a physics node if physics is enabled
  auto sims = getComponents<PhysicsComponent>(components);
  if (!sims.empty())
  {
    RCHECK(sims.size()==1);
    osg::ref_ptr<Rcs::PhysicsNode> pn = new Rcs::PhysicsNode();
    pn->setSimulation(sims[0]->getPhysics());
    pn->init(true);   // with force dragger
    viewer->add(pn);
  }
  else
  {
    viewer->add(new NamedMouseDragger(getGraph()));
  }

  viewer->setTitle("ExampleActionsECS");

  // Apply default camera view, or the transform of a body named 'initial_camera_view'.
  double q_cam[6];
  VecNd_set6(q_cam, -2.7, -2.5, 3.4, -0.4, 0.5, 0.8);

  const RcsBody* camera_body = RcsGraph_getBodyByName(getCurrentGraph(), "default_camera_view");
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

  viewer->setKeyCallback('q', [this](char k)
  {
    RLOG(0, "Quitting");
    getEntity().publish("Quit");
  }, "Quit");

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
    nlohmann::json json = getObjectOccludersForAgent("Daniel", "fanta_bottle", getScene(),
                                                     getGraph());
    RLOG_CPP(0, "getOccludersForAgent(Daniel, fanta_bottle):\n" << json.dump(4));

    json = getOccludedObjectsForAgent("Daniel", getScene(), getGraph());
    RLOG_CPP(0, "getOccludedObjectsForAgent(Daniel):\n" << json.dump(4));

  }, "Test occlusions");

  viewer->setKeyCallback('h', [this](char k)
  {
    RLOG(0, "Showing actions and affordances");
    std::string str = ActionFactory::printToString();
    str += "\n\n";
    str += actionC->getScene()->printAffordancesToString();
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

    RcsGraph_writeDotFile(getGraph(), "RcsGraph.dot");
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
    auto bn = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    if (!bn)
    {
      return;
    }

    auto scene = getScene();
    auto om = scene->getOccupiedManipulators(getGraph());
    RLOG_CPP(0, "Found " << om.size() << " occupied manipulators");

    if (om.empty())
    {
      auto textCmd = "get " + std::string(bn->body()->name);
      entity.publish("PlanDFSEE", textCmd);
    }
    else
    {
      auto ge = om[0]->getGraspedEntities(*scene, getGraph());
      RCHECK_MSG(!ge.empty(), "For manipulator: '%s'", om[0]->name.c_str());
      auto textCmd = "put " + ge[0]->name + " " + std::string(bn->body()->name);
      entity.publish("PlanDFSEE", textCmd);
    }

  }, "Get body under mouse");

  viewer->setKeyCallback('l', [this](char k)
  {
    auto bn = viewer->getBodyNodeUnderMouse<Rcs::BodyNode*>();
    if (!bn)
    {
      return;
    }

    auto textCmd = "eye_gaze " + std::string(bn->body()->name);
    entity.publish("PlanDFSEE", textCmd);

  }, "Get body under mouse");

  viewer->setKeyCallback('a', [this](char k)
  {
    RLOG(0, "Toggling talk flag");
    entity.publish("ToggleASR");

  }, "Toggle talk flag");

  viewer->setKeyCallback('p', [this](char k)
  {
    RLOG(0, "Resetting physics rigid bodies");
    entity.publish("ResetRigidBodies");

  }, "Resetting physics rigid bodies");

  viewer->setKeyCallback('A', [this](char k)
  {
    int a = TrajectoryPredictor::toggleAnimationMode();
    RLOG(0, "Setting animation mode to %d", a);
  }, "Toggle animation mode: 0: none, 1: successful predictions, 2: all predictions");

  viewer->setKeyCallback('S', [this](char k)
  {
    RLOG(0, "Interrupting action");
    entity.publish("ClearTrajectory");
  }, "Interrupt action");

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

  if (!hwc.empty())
  {
    entity.publish("BackgroundColor", std::string("PEWTER"));
  }

  entity.process();

  // If we have a LandmarkZmw component, we initialize its debug graphics
  // here. We have to defer it to this point, since there's no GraphicsWindow
  // before this.
  auto lmZmqs = getComponents<LandmarkZmqComponent>(components);
  for (auto& c : lmZmqs)
  {
    c->createDebugGraphics(viewer.get());
  }


  return true;
}

bool ExampleActionsECS::initGuis()
{
  if (valgrind)
  {
    return true;
  }

  // Gui for manually triggering events with basic data types.
  if (withEventGui)
  {
    new aff::EventGui(&entity);
  }

  if (!noTextGui)
  {
    textGui = new aff::TextEditComponent(&entity);
    addComponent(textGui);
  }

  return true;
}

void ExampleActionsECS::run()
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

void ExampleActionsECS::step()
{
  dtProcess = Timer_getSystemTime();

  stepMtx.lock();
  updateGraph->call(getCurrentGraph());
  computeKinematics->call(getCurrentGraph());
  postUpdateGraph->call(ikc->getGraph(), getCurrentGraph());
  computeTrajectory->call(ikc->getGraph());
  setTaskCommand->call(trajC->getActivationPtr(), trajC->getTaskCommandPtr());
  setJointCommand->call(ikc->getJointCommandPtr());
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
  if (controller)
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

void ExampleActionsECS::onQuit()
{
  entity.publish("Stop");
  runLoop = false;
}

/*******************************************************************************
 * Builds a search tree, finds solutions, and handles events.
 ******************************************************************************/
static void _planActionSequenceThreaded(aff::ExampleActionsECS* ex,
                                        std::string actionSequence,
                                        size_t maxNumThreads,
                                        bool depthFirst,
                                        bool earlyExitSearch,
                                        bool earlyExitAction)
{
  ex->setProcessingAction(true);
  actionSequence = ActionSequence::resolve(ex->getGraph()->cfgFile, actionSequence);
  if (ex->verbose)
  {
    RMSG_CPP("_planActionSequenceThreaded received sequence: '" << actionSequence << "'");
  }
  std::vector<std::string> seq = Rcs::String_split(actionSequence, ";");

  auto sType = depthFirst ? PredictionTree::SearchType::DFSMT : PredictionTree::SearchType::BFS;
  auto tree = ex->getQuery()->planActionTree(sType, seq, ex->getEntity().getDt(),
                                             maxNumThreads, earlyExitSearch, earlyExitAction);

  // Early exit in case tree could not be constructed
  if (!tree)
  {
    std::vector<ActionResult> explanation(1);
    explanation[0].error = "No solution found";
    explanation[0].reason = "The action tree is empty";
    explanation[0].suggestion = "Send another command that does the same thing";
    explanation[0].developer = std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
    explanation[0].actionCommand = actionSequence;
    ex->getEntity().publish("ActionResult", false, 0.0, explanation);
    if (ex->verbose)
    {
      RMSG_CPP("_planActionSequenceThreaded: Error creating tree - returning");
    }
    return;
  }

  // From here on, we have a valid tree. But it might not contain a valid solution.
  std::vector<std::string> predictedActions;
  std::vector<TrajectoryPredictor::PredictionResult> animations;

  // We first check if there is any fatal error in the tree. In this case, we
  // only report one failed issue raather than FIRST_N_SOLUTIONS.
  auto slnPath = tree->findSolutionPath(0, false);
  size_t numSols = FIRST_N_SOLUTIONS;
  if (!slnPath.empty() && slnPath.back()->fatalError)
  {
    numSols = 1;
  }

  for (size_t i = 0; i < numSols; ++i)
  {
    TrajectoryPredictor::PredictionResult res;
    auto path = tree->findSolutionPath(i, false);

    if (!path.empty())
    {
      res.success = path.back()->success;
      RLOG_CPP(1, "Solution " << i << " is " << (res.success ? "SUCCESSFUL" : "NOT SUCCESSFUL"));

      for (const auto& nd : path)
      {
        if (i==0 && res.success)
        {
          predictedActions.push_back(nd->actionCommand());
          RLOG_CPP(1, "Action: " << nd->actionCommand() << " cost: " << nd->cost);
        }

        if (!nd->bodyTransforms.empty())
        {
          RLOG_CPP(0, "Adding transforms for : " << i << " " << nd->actionCommand() << " with "
                   << nd->bodyTransforms.size() << " transforms");
          res.bodyTransforms.insert(res.bodyTransforms.end(),
                                    nd->bodyTransforms.begin(),
                                    nd->bodyTransforms.end());
        }
      }
    }
    else
    {
      RLOG_CPP(1, "No path found for solution index " << i);
    }

    auto errMsgs = tree->getSolutionErrorStrings(i);
    RLOG_CPP(1, "Found " << errMsgs.size() << " error messages of solution " << i);
    for (const auto& e : errMsgs)
    {
      RLOG_CPP(0, e.toString());
    }

    if (!res.bodyTransforms.empty())
    {
      animations.push_back(res);
    }
  }

  // Needs to go before the return for failure, otherwise we don't see it in the animation
  if (!animations.empty())
  {
    ex->getEntity().publish("AnimateSequence", animations, 0);
  }

  // We failed: Create a meaningful error and return. We don't need to set the
  // processingAction flag here, since that's handled inside the onActionResult
  // method if success is false.
  //if (predictedActions.empty())
  if (predictedActions.size()!=seq.size())
  {
    RLOG_CPP(0, "Could not find solution: Only " << predictedActions.size()
             << " steps of " << seq.size() << " are valid");
    ex->clearCompletedActionStack();

    // Compose a meaningful error description for each failed action sequence
    std::vector<ActionResult> actionResults;

    for (size_t i = 0; i < numSols; ++i)
    {
      auto path = tree->findSolutionPath(i, false);
      std::string actionSeq;
      for (const auto& nd : path)
      {
        actionSeq += nd->actionCommand() + " ";
      }

      auto finalNode = path.empty() ? tree->root : path.back();
      const ActionResult& errMsg = finalNode->feedbackMsg;

      if (!errMsg.error.empty())
      {
        actionResults.push_back(errMsg);
      }
    }

    ex->getEntity().publish("ActionResult", false, 0.0, actionResults);

    REXEC(0)
    {
      bool success = tree->toDotFile("PredictionTreeDFS.dot");
      RLOG(1, "%s PredictionTreeDFS.dot", success ? "Successfully wrote" : "Failed to write");
    }

    if (ex->verbose)
    {
      RMSG_CPP("_planActionSequenceThreaded: No successful predicted action - returning");
    }

    return;
  }

  REXEC(0)
  {
    bool success = tree->toDotFile("PredictionTreeDFS.dot");
    RLOG(0, "%s PredictionTreeDFS.dot", success ? "Successfully wrote" : "Failed to write");
  }

  REXEC(1)
  {
    tree->printTreeVisual(tree->root, 0);
  }

  // From here on, we succeeded
  std::string detailedActionCommand = Rcs::String_concatenate(predictedActions, ";");
  RLOG_CPP(0, "Sequence has " << predictedActions.size() << " steps: " << detailedActionCommand);
  ex->getEntity().publish("ActionSequence", detailedActionCommand);

  if (ex->verbose)
  {
    RMSG_CPP("_planActionSequenceThreaded: SUCCESS - returning");
  }
}

void ExampleActionsECS::onPlanActionSequenceBFS(std::string actionSequence)
{
  bool dfs = false;
  std::thread t1(_planActionSequenceThreaded, this, actionSequence, maxNumThreads, dfs, false, earlyExitAction);
  t1.detach();
}

void ExampleActionsECS::onPlanActionSequenceDFS(std::string actionSequence)
{
  bool dfs = true;
  std::thread t1(_planActionSequenceThreaded, this, actionSequence, maxNumThreads, dfs, false, earlyExitAction);
  t1.detach();
}

void ExampleActionsECS::onPlanActionSequenceDFSEE(std::string actionSequence)
{
  bool dfs = true;
  std::thread t1(_planActionSequenceThreaded, this, actionSequence, maxNumThreads, dfs, true, earlyExitAction);
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
    std::vector<ActionResult> fbmsg(1);
    fbmsg[0].error = "ERROR";
    fbmsg[0].reason = "Received empty action command string.";
    fbmsg[0].suggestion = "Check your spelling.";
    fbmsg[0].developer = std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
    entity.publish("ActionResult", false, 0.0, fbmsg);
    return;
  }

  text = ActionSequence::resolve(getGraph()->cfgFile, text);

  // From here on, any sequence has been expanded
  RLOG_CPP(1, "Sequence expanded to '" << text << "'");

  // From here on, any sequence has been expanded
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
  getScene()->print();
  std::cout << help();
}

void ExampleActionsECS::onTrajectoryMoving(bool isMoving)
{
  // This case is true if the trajectory has started. There is nothing to do here.
  if (isMoving)
  {
    return;
  }

  RLOG_CPP(0, "TrajectoryMoving event got false");

  // If the trajectory has finshed, we can report success.
  std::vector<ActionResult> fbmsg(1);
  fbmsg[0].error = "SUCCESS";
  fbmsg[0].actionCommand = actionStack.empty() ? std::string() : actionStack[0];
  fbmsg[0].developer = std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
  entity.publish("ActionResult", true, 0.0,fbmsg);

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
  RLOG_CPP(0, "RECEIVED: " << text);

  // All sequences must have been resolved before this point.
  RCHECK_MSG(text.find(';') == std::string::npos,
             "Received string with semicolon: '%s'", text.c_str());

  if (text=="get_state")
  {
    std::thread feedbackThread([this]()
    {
      std::string fb = sceneQuery->instance()->getSceneState().dump();
      entity.publish("SendWebsocket", fb);
    });
    feedbackThread.detach();
  }
  // Here we catch the speak actions. This does not yet support the grammar
  // with parallel actions (concatenated with +)
  else if (STRNEQ(text.c_str(), "speak", 5))
  {
    std::string textToSpeak = text.c_str() + 5;
    RLOG_CPP(0, "Speak action: " << textToSpeak);
    entity.publish("Speak", textToSpeak);

    std::vector<ActionResult> fbmsg(1);
    fbmsg[0].error = "SUCCESS";
    fbmsg[0].developer = "SUCCESS DEVELOPER: speaking '" +
                         textToSpeak + "' " + std::string(__FILENAME__) +
                         " line " + std::to_string(__LINE__);
    entity.publish("ActionResult", true, 0.0, fbmsg);
    return;
  }
  else if (STRNEQ(text.c_str(), "reset", 5) && (!getRobotEnabled()))
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
        std::vector<ActionResult> fbmsg(1);
        fbmsg[0].error = "ERROR when doing a reset command";
        fbmsg[0].reason = "Reset with xml file '" + resetCmd[1] + "' failed.";
        fbmsg[0].suggestion = "Check if the file name is correct.";
        fbmsg[0].developer = std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
        std::string errStr = "ERROR REASON: Reset with xml file '" + resetCmd[1] +
                             "' failed. SUGGESTION: Check if the file name is correct";
        entity.publish("ActionResult", false, 0.0, fbmsg);

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

    std::vector<ActionResult> fbmsg(1);
    fbmsg[0].error = "SUCCESS";
    fbmsg[0].developer = std::string("Reset succeeded ") + std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
    entity.publish("ActionResult", true, 0.0, fbmsg);

    // The ActionScene is reloaded, since otherwise fill levels etc. remain as
    // they are.
    RLOG(0, "Reloading scene from %s", getGraph()->cfgFile);
    getScene()->reload(getGraph()->cfgFile);
    getScene()->initializeKinematics(getGraph());   // Reach and shoulder joints

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
  // reset and get_state are taken care of somewhere else
  else
  {
    std::thread t1(&ActionComponent::actionThread, actionC, text);
    t1.detach();
  }
}

void ExampleActionsECS::onActionResult(bool success, double quality,
                                       std::vector<ActionResult> resMsg)
{
  RCHECK(!resMsg.empty());
  std::string resAsString;
  if (verbose)
  {
    for (const auto& s : resMsg)
    {
      resAsString += s.toString() + " ";
    }

    RMSG_CPP(resAsString);
  }

  lastActionResult = resMsg;
  entity.publish("SetTextLine", lastActionResult[0].error, 1);

  // This needs to be done independent of failure or success
  if (!actionStack.empty())
  {
    addToCompletedActionStack(actionStack[0], resMsg[0].error);

    REXEC(0)
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
  // and accept updates from the perceived scene. The "processingAction" flag
  // must be resetted after assigning the lastActionResult.
  if (actionStack.empty())
  {
    RLOG(0, "ActionStack is empty, unfreezing perception");
    entity.publish("FreezePerception", false);
    processingAction = false;
    actionC->setFinalPoseRunning(false);

    RLOG_CPP(0, "lastActionResult has " << lastActionResult.size() << " entries:");
    for (const auto& fb : lastActionResult)
    {
      RLOG_CPP(0, fb.toString());
    }
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
    entity.publish("SendWebsocket", resAsString);
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
  return actionC ? actionC->getScene() : nullptr;
}

const ActionScene* ExampleActionsECS::getScene() const
{
  return actionC ? actionC->getScene() : nullptr;
}

RcsGraph* ExampleActionsECS::getGraph()
{
  return controller ? controller->getGraph() : nullptr;
}

const RcsGraph* ExampleActionsECS::getGraph() const
{
  return controller ? controller->getGraph() : nullptr;
}

RcsGraph* ExampleActionsECS::getCurrentGraph()
{
  return graphC ? graphC->getGraph() : nullptr;
}

const RcsGraph* ExampleActionsECS::getCurrentGraph() const
{
  return graphC ? graphC->getGraph() : nullptr;
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
  return viewer ? viewer.get() : nullptr;
}

const EntityBase& ExampleActionsECS::getEntity() const
{
  return entity;
}

EntityBase& ExampleActionsECS::getEntity()
{
  return entity;
}

const VirtualCamera* ExampleActionsECS::getVirtualCamera() const
{
  return virtualCamera.get();
}

VirtualCamera* ExampleActionsECS::getVirtualCamera()
{
  return virtualCamera.get();
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
    std::cout << "  completedActionStack[" << i << "]: " << std::endl
              << "    Action = '" << completedActionStack[i].first << "'" << std::endl
              << "    Result = '" << completedActionStack[i].second << "'" << std::endl;
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

void ExampleActionsECS::onSetTurboMode(bool enable)
{
  ActionBase::setTurboMode(enable);
}

void ExampleActionsECS::onClearTrajectory()
{
  RLOG(0, "ExampleActionsECS::clearTrajectory()");
  hasBeenStopped = true;

  std::vector<ActionResult> explanation(1);
  explanation[0].error = "Actions interrupted";
  explanation[0].reason = "Somebody interrupted the actions";
  explanation[0].suggestion = "Wait for the next actions";
  explanation[0].developer = std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
  explanation[0].actionCommand = actionStack.empty() ? std::string() : actionStack[0];
  entity.publish("ActionResult", false, 0.0, explanation);
}

void ExampleActionsECS::onSetPupilSpeedWeight(double weight)
{
  ActionEyeGaze::setPupilSpeedWeight(getGraph(), weight);
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

bool ExampleActionsECS::isProcessingAction() const
{
  return processingAction.load();
}

void ExampleActionsECS::setProcessingAction(bool isProcessing)
{
  processingAction = isProcessing;
}



/*******************************************************************************
 *
 ******************************************************************************/
class ExampleCocktail : public ExampleActionsECS
{
public:

  ExampleCocktail(int argc, char** argv) : ExampleActionsECS(argc, argv)
  {
  }

  virtual ~ExampleCocktail()
  {
  }

  bool initParameters()
  {
    ExampleActionsECS::initParameters();
    xmlFileName = "g_example_curiosity_cocktails.xml";
    return true;
  }
};

RCS_REGISTER_EXAMPLE(ExampleCocktail, "Actions", "Cocktail");



/*******************************************************************************
 *
 ******************************************************************************/
class ExamplePizza : public ExampleActionsECS
{
public:

  ExamplePizza(int argc, char** argv) : ExampleActionsECS(argc, argv)
  {
  }

  virtual ~ExamplePizza()
  {
  }

  bool initParameters()
  {
    ExampleActionsECS::initParameters();
    xmlFileName = "g_example_pizza.xml";
    return true;
  }
};

RCS_REGISTER_EXAMPLE(ExamplePizza, "Actions", "Pizza");


/*******************************************************************************
 *
 ******************************************************************************/
class ExampleCocktailGen3 : public ExampleActionsECS
{
public:

  ExampleCocktailGen3(int argc, char** argv) : ExampleActionsECS(argc, argv)
  {
  }

  virtual ~ExampleCocktailGen3()
  {
  }

  bool initParameters()
  {
    ExampleActionsECS::initParameters();
    xmlFileName = "g_example_curiosity_cocktails_gen3.xml";
    return true;
  }
};

RCS_REGISTER_EXAMPLE(ExampleCocktailGen3, "Actions", "Cocktails Gen3");


/*******************************************************************************
 * bin\Release\TestLLMSim.exe
 *   -dir ..\src\Smile\src\AffAction\config\xml\examples
 *   -face_tracking -landmarks_zmq -landmarks_camera head_kinect_rgb_link
 *   -camera_view -face_gesture
 ******************************************************************************/
class ExampleMediapipeFaceView : public ExampleActionsECS
{
public:

  ExampleMediapipeFaceView(int argc, char** argv) : ExampleActionsECS(argc, argv)
  {
  }

  virtual ~ExampleMediapipeFaceView()
  {
  }

  std::string help()
  {
    std::stringstream s;

    s << ExampleActionsECS::help() << std::endl;
    s << "Start python webcam program: " << std::endl;
    s << "python webcam_tracking_socket.py --mediapipe --camera_config_file Logitech-C920.yaml";
    return s.str();
  }

  bool initParameters()
  {
    ExampleActionsECS::initParameters();
    xmlFileName = "g_example_curiosity_cocktails.xml";
    componentArgs += " -landmarks_zmq -landmarks_camera head_kinect_lens -face_tracking -face_bodyName face";
    componentArgs += " -face_gesture -face_bodyName face";
    componentArgs += " -camera_view -camera_view_body face";

    return true;
  }

};

RCS_REGISTER_EXAMPLE(ExampleMediapipeFaceView, "Actions", "Mediapipe Face View");


/*******************************************************************************
 * Virtual rendering:
 * TestLLMSim.exe
 *   -dir ..\src\Smile\src\AffAction\config\xml\examples\
 *   -f g_example_curiosity_cocktails.xml
 *   -physics
 *   -virtualCam xxx
 ******************************************************************************/
class ExampleVirtualRendering : public ExampleActionsECS
{
public:

  ExampleVirtualRendering(int argc, char** argv) : ExampleActionsECS(argc, argv)
  {
  }

  virtual ~ExampleVirtualRendering()
  {
  }

  bool initParameters()
  {
    ExampleActionsECS::initParameters();
    configDirectory = "config/xml/AffAction/xml/examples";
    xmlFileName = "g_example_curiosity_cocktails.xml";
    speedUp = 1;
    physicsEngine = "Bullet";
    virtualCameraWindowEnabled = true;
    return true;
  }

};

RCS_REGISTER_EXAMPLE(ExampleVirtualRendering, "Actions", "Render");


/*******************************************************************************
 *
 ******************************************************************************/
class ExamplePW70 : public ExampleActionsECS
{
public:

  ExamplePW70(int argc, char** argv) : ExampleActionsECS(argc, argv)
  {
  }

  virtual ~ExamplePW70()
  {
  }

  bool initParameters()
  {
    ExampleActionsECS::initParameters();
    configDirectory = "config/xml/AffAction/xml/examples";
    xmlFileName = "g_example_pizza.xml";
    speedUp = 1;
    componentArgs = "-pw70_vel -pw70_pan_joint_name ptu_pan_joint -pw70_tilt_joint_name ptu_tilt_joint -pw70_control_frequency 50";
    return true;
  }

  virtual bool initGraphics()
  {
    bool success = ExampleActionsECS::initGraphics();
    graphC->setEnableRender(true);
    entity.publish<std::string, const RcsGraph*>("RenderGraph", "Physics", getCurrentGraph());
    entity.publish<std::string, const RcsGraph*>("RenderGraph", "IK", ikc->getGraph());
    entity.process();
    Timer_waitDT(0.5);
    entity.publish("RenderCommand", std::string("Physics"), std::string("show"));
    entity.publish("RenderCommand", std::string("IK"), std::string("hide"));
    entity.process();

    return success;
  }

  bool initAlgo()
  {
    bool success = ExampleActionsECS::initAlgo();
    graphC->setEnableRender(true);
    return success;
  }

};

RCS_REGISTER_EXAMPLE(ExamplePW70, "Actions", "PTU velocity control");


/*******************************************************************************
 *
 ******************************************************************************/
class ExamplePW70_pos : public ExamplePW70
{
public:

  ExamplePW70_pos(int argc, char** argv) : ExamplePW70(argc, argv)
  {
  }

  virtual ~ExamplePW70_pos()
  {
  }

  bool initParameters()
  {
    ExamplePW70::initParameters();
    withEventGui = true;
    componentArgs = "-pw70_pos -pw70_pan_joint_name ptu_pan_joint -pw70_tilt_joint_name ptu_tilt_joint -pw70_control_frequency 50";
    return true;
  }

};

RCS_REGISTER_EXAMPLE(ExamplePW70_pos, "Actions", "PTU position control");

}   // namespace aff
