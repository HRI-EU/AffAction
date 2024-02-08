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

#include <pybind11/stl.h>
#include "pybind11_json.hpp"
#include "pybind_dict_utils.h"
#include "type_casters.h"

namespace py = pybind11;

#include <LandmarkBase.h>
#include <ExampleLLMSim.h>
#include <ActionFactory.h>
#include <HardwareComponent.h>

#include <Rcs_resourcePath.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_utilsCPP.h>
#include <json.hpp>

#include <SegFaultHandler.h>

#if !defined(_MSC_VER)
#include <X11/Xlib.h>
#endif

#include <chrono>

RCS_INSTALL_ERRORHANDLERS



//////////////////////////////////////////////////////////////////////////////
// Fully threaded tree prediction and execution function.
//////////////////////////////////////////////////////////////////////////////
void _planActionSequenceThreaded(aff::ExampleLLMSim& ex,
                                 std::string sequenceCommand,
                                 size_t maxNumThreads)
{
  std::vector<std::string> seq = Rcs::String_split(sequenceCommand, ";");
  auto res = ex.sceneQuery2->planActionSequence(seq, seq.size(), maxNumThreads);

  if (res.empty())
  {
    RLOG_CPP(0, "Could not find solution");
    ex.processingAction = false;
    ex.clearCompletedActionStack();
    return;
  }

  RLOG_CPP(0, "Sequence has " << res.size() << " steps");

  std::string newCmd;
  for (size_t i = 0; i < res.size(); ++i)
  {
    newCmd += res[i];
    newCmd += ";";
  }

  RLOG_CPP(0, "Command : " << newCmd);
  ex.entity.publish("ActionSequence", newCmd);
}

//////////////////////////////////////////////////////////////////////////////
// Simple helper class that blocks the execution in the wait() function
// until the ActionResult event has been received.
//////////////////////////////////////////////////////////////////////////////
class PollBlockerComponent
{
public:

  PollBlockerComponent(aff::ExampleLLMSim* sim_) : sim(sim_)
  {
    sim->processingAction = true;
  }

  void wait()
  {
    while (sim->processingAction)
    {
      Timer_waitDT(0.1);
    }
    RLOG(0, "Done wait");
  }

  aff::ExampleLLMSim* sim;
};





//////////////////////////////////////////////////////////////////////////////
// Affordance enums for convenience in the python world
//////////////////////////////////////////////////////////////////////////////
void define_AffordanceTypes(py::module& m)
{
  py::enum_<aff::Affordance::Type>(m, "AffordanceType")
  .value("Affordance", aff::Affordance::Type::Affordance)
  .value("Graspable", aff::Affordance::Type::Graspable)
  .value("PowerGraspable", aff::Affordance::Type::PowerGraspable)
  .value("PincerGraspable", aff::Affordance::Type::PincerGraspable)
  .value("PalmGraspable", aff::Affordance::Type::PalmGraspable)
  .value("BallGraspable", aff::Affordance::Type::BallGraspable)
  .value("CircularGraspable", aff::Affordance::Type::CircularGraspable)
  .value("TwistGraspable", aff::Affordance::Type::TwistGraspable)
  .value("Twistable", aff::Affordance::Type::Twistable)
  .value("PushSwitchable", aff::Affordance::Type::PushSwitchable)
  .value("Supportable", aff::Affordance::Type::Supportable)
  .value("Stackable", aff::Affordance::Type::Stackable)
  .value("Containable", aff::Affordance::Type::Containable)
  .value("Pourable", aff::Affordance::Type::Pourable)
  .value("PointPushable", aff::Affordance::Type::PointPushable)
  .value("PointPokable", aff::Affordance::Type::PointPokable)
  .value("Hingeable", aff::Affordance::Type::Hingeable)
  .value("Dispensible", aff::Affordance::Type::Dispensible)
  .value("Wettable", aff::Affordance::Type::Wettable)
  .value("Openable", aff::Affordance::Type::Openable);
}





//////////////////////////////////////////////////////////////////////////////
// The python affaction module, mainly consisting off the LlmSim class.
//////////////////////////////////////////////////////////////////////////////
PYBIND11_MODULE(pyAffaction, m)
{
  define_AffordanceTypes(m);

  //////////////////////////////////////////////////////////////////////////////
  // LlmSim constructor
  //////////////////////////////////////////////////////////////////////////////
  py::class_<aff::ExampleLLMSim>(m, "LlmSim")
  .def(py::init<>([]()
  {
#if !defined(_MSC_VER)// Avoid crashes when running remotely.
    static bool xInitialized = false;
    if (!xInitialized)
    {
      xInitialized = true;
      XInitThreads();
    }
#endif

    auto ex = std::unique_ptr<aff::ExampleLLMSim>(new aff::ExampleLLMSim());
    ex->initParameters();
    return std::move(ex);
  }))

  //////////////////////////////////////////////////////////////////////////////
  // Initialization function, to be called after member variables have been
  // configured.
  //////////////////////////////////////////////////////////////////////////////
  .def("init", [](aff::ExampleLLMSim& ex, bool debug=false)
  {
    bool success = ex.initAlgo();

    if (debug)
    {
      success = ex.initGraphics() && success;
      ex.entity.publish("Render");
      ex.entity.process();

      // ex.viewer->setKeyCallback('l', [&ex](char k)
      // {
      //   RLOG(0, "Toggle talk flag");
      //   ex.entity.publish("ToggleASR");

      // }, "Toggle talk flag");
    }

    std::string starLine(80, '*');
    std::cerr << "\n\n" + starLine;
    if (success)
    {
      std::cerr << "\n* TestLLMSim initialized\n";
    }
    else
    {
      std::cerr << "\n* Failed to initialize TestLLMSim\n";
    }
    std::cerr << starLine << "\n";

    return success;
  }, "Initializes algorithm, guis and graphics")

  //////////////////////////////////////////////////////////////////////////////
  // Update one LlmSim instance from anotuer one
  //////////////////////////////////////////////////////////////////////////////
  .def("sync", [](aff::ExampleLLMSim& ex, py::object obj)
  {
    aff::ExampleLLMSim* sim = obj.cast<aff::ExampleLLMSim*>();
    RcsGraph_copy(ex.controller->getGraph(), sim->controller->getGraph());
    ex.getScene()->agents = sim->getScene()->agents;
    ex.step();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns empty json if the agent can see all objects or a json in the form:
  // {"occluded": ["entity name 1", "entity name 2"] }
  //////////////////////////////////////////////////////////////////////////////
  .def("getOccludedObjectsForAgent", [](aff::ExampleLLMSim& ex, std::string agentName) -> nlohmann::json
  {
    return ex.sceneQuery->getOccludedObjectsForAgent(agentName);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns empty json if not occluded, or occluding objects sorted by distance
  // to eye (increasing): {"occluded_by": ["id_1", "id_2"] }
  //////////////////////////////////////////////////////////////////////////////
  .def("isOccludedBy", [](aff::ExampleLLMSim& ex, std::string agentName, std::string objectName) -> nlohmann::json
  {
    return ex.sceneQuery->getObjectOccludersForAgent(agentName, objectName);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns a boolean indicating if any scene entity is closer to any hand of
  // the agent closer than a distance threshold.
  //////////////////////////////////////////////////////////////////////////////
  .def("isBusy", [](aff::ExampleLLMSim& ex, std::string agentName) -> bool
  {
    return ex.sceneQuery->isAgentBusy(agentName, 0.15);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns the pan tilt angles for the agent when it looks at the gazeTarget
  //////////////////////////////////////////////////////////////////////////////
  .def("getPanTilt", [](aff::ExampleLLMSim& ex, std::string roboAgent, std::string gazeTarget)
  {
    RLOG(0, "Pan tilt angle calculation");

    if (!ex.panTiltQuery)
    {
      RLOG(0, "panTiltQuery not yet constructed");
      double buf = 0.0;
      MatNd tmp = MatNd_fromPtr(0, 0, &buf);
      return pybind11::detail::MatNd_toNumpy(&tmp);
    }

    std::vector<double> panTilt = ex.panTiltQuery->getPanTilt(roboAgent, gazeTarget);

    if (panTilt.empty())
    {
      double buf = 0.0;
      MatNd tmp = MatNd_fromPtr(0, 0, &buf);
      return pybind11::detail::MatNd_toNumpy(&tmp);
    }

    MatNd tmp = MatNd_fromPtr(2, 1, panTilt.data());
    return pybind11::detail::MatNd_toNumpy(&tmp);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Kinematic check of all agents if the object is within a reachable range.
  // For the robot agent: If the result is true, it does not necessarily mean
  // that it can be grasped
  //////////////////////////////////////////////////////////////////////////////
  .def("isReachable", [](aff::ExampleLLMSim& ex, std::string agentName, std::string objectName)
  {
    auto agent = ex.getScene()->getAgent(agentName);
    if (!agent)
    {
      RLOG_CPP(0, "Agent " << agentName << " unknown in scene. "
               << ex.getScene()->agents.size() << " agents:");
      for (const auto& agent : ex.getScene()->agents)
      {
        agent->print();
      }
      return false;
    }

    auto ntts = ex.getScene()->getAffordanceEntities(objectName);
    if (ntts.empty())
    {
      RLOG_CPP(0, "Object " << objectName << " unknown in scene");
      return false;
    }

    for (const auto& ntt : ntts)
    {
      const double* pos = ntt->body(ex.controller->getGraph())->A_BI.org;
      if (agent->canReachTo(ex.getScene(), ex.controller->getGraph(), pos))
      {
        return true;
      }
    }

    return false;
  }, "Check if agent can reach to the given position")

  .def("initHardwareComponents", [](aff::ExampleLLMSim& ex)
  {
    ex.entity.initialize(ex.graphC->getGraph());
  }, "Initializes hardware components")

  //////////////////////////////////////////////////////////////////////////////
  // Calls the run method in a new thread, releases the GIL and returns to the
  // python context (e.g. console).
  //////////////////////////////////////////////////////////////////////////////
  .def("run", &aff::ExampleLLMSim::startThreaded, py::call_guard<py::gil_scoped_release>(), "Starts endless loop")

  //////////////////////////////////////////////////////////////////////////////
  // Simulates the action command in a copy of the simulator class.
  //////////////////////////////////////////////////////////////////////////////
  .def("simulate", [](aff::ExampleLLMSim& ex, std::string actionCommand)
  {
    auto sim = std::unique_ptr<aff::ExampleLLMSim>(new aff::ExampleLLMSim());
    sim->initParameters();
    sim->noTextGui = true;
    sim->unittest = true;
    sim->useWebsocket = false;
    sim->speedUp = 1e8;
    sim->sequenceCommand = actionCommand;
    sim->xmlFileName = ex.xmlFileName;
    sim->config_directory = ex.config_directory;
    sim->verbose = ex.verbose;
    sim->initAlgo();
    ex.stepMtx.lock();
    RcsGraph_copy(sim->controller->getGraph(), ex.controller->getGraph());
    sim->getScene()->agents = ex.getScene()->agents;
    ex.stepMtx.unlock();
    sim->start();

    return STRNEQ(sim->lastResultMsg.c_str(), "SUCCESS", 7) ? true : false;
  })
  .def("isGraspable", [](aff::ExampleLLMSim& ex, std::string agentName, std::string objName)
  {
    auto sim = std::unique_ptr<aff::ExampleLLMSim>(new aff::ExampleLLMSim());
    sim->initParameters();
    sim->noTextGui = true;
    sim->unittest = true;
    sim->useWebsocket = false;
    sim->speedUp = 1e8;
    sim->sequenceCommand = "get " + objName;
    sim->xmlFileName = ex.xmlFileName;
    sim->config_directory = ex.config_directory;
    sim->verbose = ex.verbose;
    sim->initAlgo();
    ex.stepMtx.lock();
    RcsGraph_copy(sim->controller->getGraph(), ex.controller->getGraph());
    sim->getScene()->agents = ex.getScene()->agents;
    ex.stepMtx.unlock();
    sim->start();

    return STRNEQ(sim->lastResultMsg.c_str(), "SUCCESS", 7) ? true : false;
  })

  .def("run", [](aff::ExampleLLMSim& ex, std::string actionCommand)
  {
    double t_calc = Timer_getSystemTime();
    ex.sequenceCommand = actionCommand;
    ex.start();
    t_calc = Timer_getSystemTime() - t_calc;

    std::string starLine(80, '*');
    std::cerr << "\n\n" + starLine + "\n* TestLLMSim exits with "
              << ex.getNumFailedActions() << " failed actions";
    std::cerr << " after " << std::to_string(t_calc) << " seconds\n";
    std::cerr << starLine << "\n";

    return ex.lastResultMsg;
  })
  .def("reset", [](aff::ExampleLLMSim& ex)
  {
    ex.entity.publish("ActionSequence", std::string("reset"));
    ex.entity.process();
  })
  .def("render", [](aff::ExampleLLMSim& ex)
  {
    ex.entity.publish("Render");
    ex.entity.process();
  })
  .def("process", [](aff::ExampleLLMSim& ex)
  {
    ex.entity.process();
  })
  .def("showGraphicsWindow", [](aff::ExampleLLMSim& ex)
  {
    bool success = ex.initGraphics();

    if (success)
    {
      ex.entity.publish("Render");
      ex.entity.process();
    }

    return success;
  })
  .def("showGuis", [](aff::ExampleLLMSim& ex)
  {
    return ex.initGuis();;
  })
  .def("hideGraphicsWindow", [](aff::ExampleLLMSim& ex)
  {
    if (!ex.viewer)
    {
      return false;
    }

    ex.viewer.reset();

    return true;
  })

  .def("get_state", [](aff::ExampleLLMSim& ex)
  {
    return ex.sceneQuery->getSceneState().dump();
  })
  .def("get_scene_entities", &aff::ExampleLLMSim::getSceneEntities)
  .def("step", &aff::ExampleActionsECS::step)
  .def("stop", &aff::ExampleActionsECS::stop)
  .def("isRunning", &aff::ExampleActionsECS::isRunning)

  //////////////////////////////////////////////////////////////////////////////
  // Execute the action command, and return immediately.
  //////////////////////////////////////////////////////////////////////////////
  //.def("execute", &aff::ExampleActionsECS::onActionSequence)
  .def("execute", [](aff::ExampleLLMSim& ex, std::string actionCommand)
  {
    ex.entity.publish("ActionSequence", actionCommand);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Execute the action command, and return only after finished.
  //////////////////////////////////////////////////////////////////////////////
  .def("executeBlocking", [](aff::ExampleLLMSim& ex, std::string actionCommand) -> bool
  {
    PollBlockerComponent blocker(&ex);
    ex.entity.publish("ActionSequence", actionCommand);
    blocker.wait();
    RLOG_CPP(0, "Finished: " << actionCommand);
    bool success =  STRNEQ(ex.lastResultMsg.c_str(), "SUCCESS", 7);

    RLOG(0, "   success=%s   result=%s", success ? "true" : "false", ex.lastResultMsg.c_str());
    return success;
  })
  .def("getRobotCapabilities", [](aff::ExampleLLMSim& ex)
  {
    return aff::ActionFactory::printToString();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Predict action sequence as tree
  // Call it like: agent.sim.predictActionSequence("get fanta_bottle;put fanta_bottle lego_box;")
  //////////////////////////////////////////////////////////////////////////////
  .def("predictActionSequence", [](aff::ExampleLLMSim& ex, std::string sequenceCommand) -> std::vector<std::string>
  {
    std::vector<std::string> seq = Rcs::String_split(sequenceCommand, ";");
    return ex.sceneQuery2->planActionSequence(seq, seq.size());
  })

  //////////////////////////////////////////////////////////////////////////////
  // Predict action sequence as tree
  // Call it like: agent.sim.planActionSequence("get fanta_bottle;put fanta_bottle lego_box;")
  //////////////////////////////////////////////////////////////////////////////
  .def("planActionSequenceThreaded", [](aff::ExampleLLMSim& ex, std::string sequenceCommand)
  {
    ex.processingAction = true;
    const size_t maxNumthreads = 0;   // 0 means auto-select
    std::thread t1(_planActionSequenceThreaded, std::ref(ex), sequenceCommand, maxNumthreads);
    t1.detach();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Predict action sequence as tree
  // Call it like: agent.sim.planActionSequence("get fanta_bottle;put fanta_bottle lego_box;")
  //////////////////////////////////////////////////////////////////////////////
  .def("planActionSequence", [](aff::ExampleLLMSim& ex, std::string sequenceCommand, bool blocking) -> bool
  {
    std::vector<std::string> seq = Rcs::String_split(sequenceCommand, ";");
    auto res = ex.sceneQuery->planActionSequence(seq, seq.size());

    if (res.empty())
    {
      RLOG_CPP(0, "Could not find solution");
      return false;
    }

    RLOG_CPP(0, "Sequence has " << res.size() << " steps");

    std::string newCmd;
    for (size_t i = 0; i < res.size(); ++i)
    {
      newCmd += res[i];
      newCmd += ";";
    }

    RLOG_CPP(0, "Command : " << newCmd);

    PollBlockerComponent blocker(&ex);
    ex.entity.publish("ActionSequence", newCmd);

    bool success = true;

    if (blocking)
    {
      blocker.wait();
      STRNEQ(ex.lastResultMsg.c_str(), "SUCCESS", 7);
      RLOG(0, "   success=%s   result=%s", success ? "true" : "false", ex.lastResultMsg.c_str());
    }

    RLOG_CPP(0, "Finished: " << newCmd);

    return success;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to listen to the Respeaker ROS nose, and to acquire the
  // sound directions, ASR etc.
  //////////////////////////////////////////////////////////////////////////////
  .def("addRespeaker", [](aff::ExampleLLMSim& ex,
                          bool listenWitHandRaisedOnly,
                          bool gazeAtSpeaker,
                          bool speakOut)
  {
    if (!ex.actionC)
    {
      RLOG(0, "Initialize ExampleLLMSim before adding Respeaker - skipping");
      return false;
    }

    aff::ComponentBase* respeaker = getComponent(ex.entity, ex.controller->getGraph(), ex.getScene(), "-respeaker");
    if (respeaker)
    {
      respeaker->setParameter("PublishDialogueWithRaisedHandOnly", listenWitHandRaisedOnly);
      respeaker->setParameter("GazeAtSpeaker", gazeAtSpeaker);
      respeaker->setParameter("SpeakOutSpeakerListenerText", speakOut);
      ex.addComponent(respeaker);
      return true;
    }

    RLOG(1, "Can't instantiate respeaker");
    return false;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Sets or clears the talk flag. This only has an effect if the Respeaker
  // component has been added, and the ASR module is running.
  //////////////////////////////////////////////////////////////////////////////
  .def("enableASR", [](aff::ExampleLLMSim& ex, bool enable)
  {
    ex.entity.publish("EnableASR", enable);
    RLOG(0, "%s ASR", enable ? "Enabling" : "Disabling");
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to listen to the landmarks publishers through ROS, which
  // is for instance the Azure Kinect, and later also the Mediapipe components
  //////////////////////////////////////////////////////////////////////////////
  .def("addLandmarkROS", [](aff::ExampleLLMSim& ex)
  {
    RCHECK_MSG(ex.actionC, "Initialize ExampleLLMSim before adding PTU");
    aff::ComponentBase* c = getComponent(ex.entity, ex.controller->getGraph(), ex.getScene(), "-landmarks_ros");
    if (c)
    {
      ex.addComponent(c);
      // return c->setParameter("DebugViewer", (void*)ex.viewer.get());
      aff::LandmarkBase* lmc = dynamic_cast<aff::LandmarkBase*>(c);
      RCHECK(lmc);
      lmc->enableDebugGraphics(ex.viewer.get());
      return true;
    }

    return true;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to connect to the PTU action server ROS node, and to being
  // able to send pan / tilt commands to the PTU
  //////////////////////////////////////////////////////////////////////////////
  .def("addPTU", [](aff::ExampleLLMSim& ex)
  {
    RCHECK_MSG(ex.actionC, "Initialize ExampleLLMSim before adding PTU");
    aff::ComponentBase* c = getComponent(ex.entity, ex.controller->getGraph(), ex.getScene(), "-ptu");
    if (c)
    {
      ex.addHardwareComponent(c);
      return true;
    }

    return false;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to connect to enable the text-to-speech functionality.
  // Currently, 2 modes are supported: the Nuance TTS which requires the
  // corresponding ROS node to run, and a native Unix espeak TTS.
  //////////////////////////////////////////////////////////////////////////////
  .def("addTTS", [](aff::ExampleLLMSim& ex, std::string type)
  {
    if (type == "nuance")
    {
      auto c = getComponent(ex.entity, ex.controller->getGraph(),
                            ex.getScene(), "-nuance_tts");
      ex.addComponent(c);
      return c ? true : false;
    }
    else if (type == "native")
    {
      auto c = getComponent(ex.entity, ex.controller->getGraph(),
                            ex.getScene(), "-tts");
      ex.addComponent(c);
      return c ? true : false;
    }

    return false;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to connect to the left Jaco7 Gen2 arm
  //////////////////////////////////////////////////////////////////////////////
  .def("addJacoLeft", [](aff::ExampleLLMSim& ex)
  {
    auto c = aff::getComponent(ex.entity, ex.controller->getGraph(),
                               ex.getScene(), "-jacoShm7l");
    ex.addHardwareComponent(c);
    return c ? true : false;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to connect to the right Jaco7 Gen2 arm
  //////////////////////////////////////////////////////////////////////////////
  .def("addJacoRight", [](aff::ExampleLLMSim& ex)
  {
    auto c = aff::getComponent(ex.entity, ex.controller->getGraph(),
                               ex.getScene(), "-jacoShm7r");
    ex.addHardwareComponent(c);
    return c ? true : false;
  })
  .def("getCompletedActionStack", &aff::ExampleActionsECS::getCompletedActionStack)
  .def("isFinalPoseRunning", &aff::ExampleActionsECS::isFinalPoseRunning)

  //////////////////////////////////////////////////////////////////////////////
  // Expose several internal variables to the python layer
  //////////////////////////////////////////////////////////////////////////////
  .def_property("useWebsocket", &aff::ExampleLLMSim::getUseWebsocket, &aff::ExampleLLMSim::setUseWebsocket)
  .def_readwrite("unittest", &aff::ExampleLLMSim::unittest)
  .def_readwrite("noTextGui", &aff::ExampleLLMSim::noTextGui)
  .def_readwrite("sequenceCommand", &aff::ExampleLLMSim::sequenceCommand)
  .def_readwrite("speedUp", &aff::ExampleLLMSim::speedUp)
  .def_readwrite("xmlFileName", &aff::ExampleLLMSim::xmlFileName)
  .def_readwrite("noLimits", &aff::ExampleLLMSim::noLimits)
  .def_readwrite("noCollCheck", &aff::ExampleLLMSim::noCollCheck)   // Set before init()
  .def_readwrite("noTrajCheck", &aff::ExampleLLMSim::noTrajCheck)
  .def_readwrite("verbose", &aff::ExampleActionsECS::verbose)
  .def_readwrite("processingAction", &aff::ExampleActionsECS::processingAction)
  .def_readwrite("noViewer", &aff::ExampleActionsECS::noViewer)
  ;








  //////////////////////////////////////////////////////////////////////////////
  // LandmarkBase perception class wrapper
  //////////////////////////////////////////////////////////////////////////////
  py::class_<aff::LandmarkBase>(m, "LandmarkBase")
  .def(py::init<>())
  .def(py::init<>([](py::object obj)
  {
    aff::ExampleLLMSim* sim = obj.cast<aff::ExampleLLMSim*>();
    RLOG_CPP(1, sim->help());
    auto lm = std::unique_ptr<aff::LandmarkBase>(new aff::LandmarkBase());
    lm->setScenePtr(sim->controller->getGraph(), sim->actionC->getDomain());

    sim->entity.subscribe("PostUpdateGraph", &aff::LandmarkBase::onPostUpdateGraph, lm.get());
    sim->entity.subscribe("FreezePerception", &aff::LandmarkBase::onFreezePerception, lm.get());

    return std::move(lm);
  }))
  .def("addArucoTracker", &aff::LandmarkBase::addArucoTracker)
  .def("addSkeletonTrackerForAgents", &aff::LandmarkBase::addSkeletonTrackerForAgents)
  .def("setJsonInput", &aff::LandmarkBase::setJsonInput)
  .def("getTrackerState", &aff::LandmarkBase::getTrackerState)
  .def("startCalibration", &aff::LandmarkBase::startCalibration)
  .def("isCalibrating", &aff::LandmarkBase::isCalibrating)
  .def("setSyncInputWithWallclock", &aff::LandmarkBase::setSyncInputWithWallclock)
  .def("getSyncInputWithWallclock", &aff::LandmarkBase::getSyncInputWithWallclock)
  .def("enableDebugGraphics", [](aff::LandmarkBase& lm, py::object obj)
  {
    aff::ExampleLLMSim* sim = obj.cast<aff::ExampleLLMSim*>();
    lm.enableDebugGraphics(sim->viewer.get());
  })
  .def("setCameraTransform", [](aff::LandmarkBase& lm, std::string cameraName)
  {
    const RcsBody* cam = RcsGraph_getBodyByName(lm.getGraph(), cameraName.c_str());
    lm.setCameraTransform(&cam->A_BI);
  })
  .def("getAffordanceFrame", [](aff::LandmarkBase& lm, std::string bodyName, aff::Affordance::Type affordanceType) -> nlohmann::json
  {
    std::vector<std::string> frames;
    nlohmann::json data;

    const aff::AffordanceEntity* entity = lm.getScene()->getAffordanceEntity(bodyName);

    if (entity)
    {
      for (aff::Affordance* affordance : entity->affordances)
      {
        if (affordance->classType == affordanceType)
        {
          frames.push_back(affordance->frame);
        }
      }
    }
    else
    {
      NLOG(0, "Entity `%s` found in scene!", bodyName.c_str());
    }

    for (auto f : frames)
    {
      const RcsBody* b = RcsGraph_getBodyByName(lm.getGraph(), f.c_str());

      data[f] = {};
      data[f]["position"] = b->A_BI.org;

      double ea[3];
      Mat3d_toEulerAngles(ea, (double(*)[3])b->A_BI.rot);
      data[f]["euler_xyzr"] = ea;
    }

    return data;
  })
#if 0
  .def("getPanTilt", [](aff::LandmarkBase& lm, std::string roboAgent, std::string gazeTarget)
  {
    const aff::Agent* robo_ = lm.getScene()->getAgent(roboAgent);
    const aff::RobotAgent* robo = dynamic_cast<const aff::RobotAgent*>(robo_);
    if (!robo)
    {
      RLOG(0, "Robo agent '%s' not found", roboAgent.c_str());
      double buf = 0.0;
      MatNd tmp = MatNd_fromPtr(0, 0, &buf);
      return pybind11::detail::MatNd_toNumpy(&tmp);
    }

    double panTilt[2], err[2];
    size_t maxIter = 100;
    double eps = 1.0e-8;

    double t_calc = Timer_getSystemTime();
    int iter = robo->getPanTilt(lm.getGraph(), gazeTarget.c_str(), panTilt, maxIter, eps, err);

    // Can't retrieve joints / bodies etc.
    if (iter==-1)
    {
      RLOG(0, "Pan tilt computation failed due to missing joints / bodies / graph");
      double buf = 0.0;
      MatNd tmp = MatNd_fromPtr(0, 0, &buf);
      return pybind11::detail::MatNd_toNumpy(&tmp);
    }

    // Couldn't converge to precision given in eps
    if ((err[0]>eps) || (err[1]>eps))
    {
      RLOG(0, "Pan tilt computation did not converge");
      double buf = 0.0;
      MatNd tmp = MatNd_fromPtr(0, 0, &buf);
      return pybind11::detail::MatNd_toNumpy(&tmp);
    }

    t_calc = Timer_getSystemTime() - t_calc;

    RLOG(0, "pan=%.1f tilt=%.1f err=%f %f took %d iterations and %.1f msec",
         RCS_RAD2DEG(panTilt[0]), RCS_RAD2DEG(panTilt[1]), err[0], err[1], iter, 1.0e3*t_calc);

    MatNd tmp = MatNd_fromPtr(2, 1, panTilt);
    return pybind11::detail::MatNd_toNumpy(&tmp);
  })
#endif
  ;







  //////////////////////////////////////////////////////////////////////////////
  // Rcs function wrappers
  //////////////////////////////////////////////////////////////////////////////

  // Sets the rcs log level
  m.def("setLogLevel", [](int level)
  {
    RcsLogLevel = level;
  });

  // Adds a directory to the resource path
  m.def("addResourcePath", [](const char* path)
  {
    return Rcs_addResourcePath(path);
  });

  // Prints the resource path to the console
  m.def("printResourcePath", []()
  {
    Rcs_printResourcePath();
  });

  m.def("getWallclockTime", []()
  {
    // Get the current time point
    auto currentTime = std::chrono::system_clock::now();

    // Convert the time point to a duration since the epoch
    std::chrono::duration<double> durationSinceEpoch = currentTime.time_since_epoch();

    // Convert the duration to seconds as a floating-point number
    double seconds = durationSinceEpoch.count();

    return seconds;
  });

}
