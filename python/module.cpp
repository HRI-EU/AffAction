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
#include <ExampleActionsECS.h>
#include <ActionFactory.h>
#include <ActionSequence.h>
#include <HardwareComponent.h>
#include <LandmarkZmqComponent.hpp>

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
void _planActionSequenceThreaded(aff::ExampleActionsECS& ex,
                                 std::string sequenceCommand,
                                 size_t maxNumThreads)
{
  std::string errMsg;
  std::vector<std::string> seq = Rcs::String_split(sequenceCommand, ";");
  auto tree = ex.getQuery()->planActionTree(aff::PredictionTree::SearchType::DFSMT,
                                            seq, ex.getEntity().getDt(), maxNumThreads);
  auto res = tree->findSolutionPathAsStrings();

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
  ex.getEntity().publish("ActionSequence", newCmd);
}

//////////////////////////////////////////////////////////////////////////////
// Simple helper class that blocks the execution in the wait() function
// until the ActionResult event has been received.
//////////////////////////////////////////////////////////////////////////////
class PollBlockerComponent
{
public:

  PollBlockerComponent(aff::ExampleActionsECS* sim_) : sim(sim_)
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

  aff::ExampleActionsECS* sim;
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
  py::class_<aff::ExampleActionsECS>(m, "LlmSim")
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

    auto ex = std::unique_ptr<aff::ExampleActionsECS>(new aff::ExampleActionsECS());
    ex->initParameters();
    return std::move(ex);
  }))

  //////////////////////////////////////////////////////////////////////////////
  // Initialization function, to be called after member variables have been
  // configured.
  //////////////////////////////////////////////////////////////////////////////
  .def("init", [](aff::ExampleActionsECS& ex, bool debug=false) -> bool
  {
    bool success = ex.initAlgo();

    if (debug)
    {
      success = ex.initGraphics() && success;
      ex.getEntity().publish("Render");
      ex.getEntity().process();

      // ex.getViewer()->setKeyCallback('l', [&ex](char k)
      // {
      //   RLOG(0, "Toggle talk flag");
      //   ex.getEntity().publish("ToggleASR");

      // }, "Toggle talk flag");
    }

    std::string starLine(80, '*');
    std::cerr << "\n\n" + starLine;
    if (success)
    {
      std::cerr << "\n* LLMSim initialized\n";
    }
    else
    {
      std::cerr << "\n* Failed to initialize LLMSim\n";
    }
    std::cerr << starLine << "\n";

    return success;
  }, "Initializes algorithm, guis and graphics")

  //////////////////////////////////////////////////////////////////////////////
  // Update one LlmSim instance from anotuer one
  //////////////////////////////////////////////////////////////////////////////
  .def("sync", [](aff::ExampleActionsECS& ex, py::object obj)
  {
    aff::ExampleActionsECS* sim = obj.cast<aff::ExampleActionsECS*>();
    RcsGraph_copy(ex.getGraph(), sim->getGraph());
    ex.getScene()->agents = sim->getScene()->agents;
    ex.step();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns empty json if the agent can see all objects or a json in the form:
  // for the agent: {"occluded": [{"name": "entity name 1", "instance_id": "entity id 1"},
  //                              {"name": "entity name 2", "instance_id": "entity id 2"}]}
  //////////////////////////////////////////////////////////////////////////////
  .def("getOccludedObjectsForAgent", [](aff::ExampleActionsECS& ex, std::string agentName) -> nlohmann::json
  {
    return ex.getQuery()->getOccludedObjectsForAgent(agentName);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns empty json if not occluded, or occluding objects sorted by distance
  // to eye (increasing): {"occluded_by": ["id_1", "id_2"] }
  //////////////////////////////////////////////////////////////////////////////
  .def("isOccludedBy", [](aff::ExampleActionsECS& ex, std::string agentName, std::string objectName) -> nlohmann::json
  {
    return ex.getQuery()->getObjectOccludersForAgent(agentName, objectName);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns a boolean indicating if any scene entity is closer to any hand of
  // the agent closer than a distance threshold.
  //////////////////////////////////////////////////////////////////////////////
  .def("isBusy", [](aff::ExampleActionsECS& ex, std::string agentName) -> bool
  {
    return ex.getQuery()->isAgentBusy(agentName, 0.15);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns the pan tilt angles for the agent when it looks at the gazeTarget
  //////////////////////////////////////////////////////////////////////////////
  .def("getPanTilt", [](aff::ExampleActionsECS& ex, std::string roboAgent, std::string gazeTarget)
  {
    RLOG(0, "Pan tilt angle calculation");

    if (!ex.getQuery())
    {
      RLOG(0, "panTiltQuery not yet constructed");
      double buf = 0.0;
      MatNd tmp = MatNd_fromPtr(0, 0, &buf);
      return pybind11::detail::MatNd_toNumpy(&tmp);
    }

    std::vector<double> panTilt = ex.getQuery()->getPanTilt(roboAgent, gazeTarget);

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
  .def("isReachable", [](aff::ExampleActionsECS& ex, std::string agentName, std::string objectName) -> bool
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
      const double* pos = ntt->body(ex.getGraph())->A_BI.org;
      if (agent->canReachTo(ex.getScene(), ex.getGraph(), pos))
      {
        return true;
      }
    }

    return false;
  }, "Check if agent can reach to the given position")

  .def("initHardwareComponents", [](aff::ExampleActionsECS& ex)
  {
    ex.getEntity().initialize(ex.getCurrentGraph());
  }, "Initializes hardware components")

  //////////////////////////////////////////////////////////////////////////////
  // Calls the run method in a new thread, releases the GIL and returns to the
  // python context (e.g. console).
  //////////////////////////////////////////////////////////////////////////////
  .def("run", &aff::ExampleActionsECS::startThreaded, py::call_guard<py::gil_scoped_release>(), "Starts endless loop")

  //////////////////////////////////////////////////////////////////////////////
  // Simulates the action command in a copy of the simulator class.
  //////////////////////////////////////////////////////////////////////////////
  .def("simulate", [](aff::ExampleActionsECS& ex, std::string actionCommand) -> bool
  {
    auto sim = std::unique_ptr<aff::ExampleActionsECS>(new aff::ExampleActionsECS());
    sim->initParameters();
    sim->noTextGui = true;
    sim->unittest = true;
    sim->speedUp = 1e8;
    sim->sequenceCommand = actionCommand;
    sim->xmlFileName = ex.xmlFileName;
    sim->config_directory = ex.config_directory;
    sim->verbose = ex.verbose;
    sim->initAlgo();
    ex.lockStepMtx();
    RcsGraph_copy(sim->getGraph(), ex.getGraph());
    sim->getScene()->agents = ex.getScene()->agents;
    ex.unlockStepMtx();
    sim->start();

    return STRNEQ(sim->lastResultMsg.c_str(), "SUCCESS", 7) ? true : false;
  })
  .def("isGraspable", [](aff::ExampleActionsECS& ex, std::string agentName, std::string objName) -> bool
  {
    auto sim = std::unique_ptr<aff::ExampleActionsECS>(new aff::ExampleActionsECS());
    sim->initParameters();
    sim->noTextGui = true;
    sim->unittest = true;
    sim->speedUp = 1e8;
    sim->sequenceCommand = "get " + objName;
    sim->xmlFileName = ex.xmlFileName;
    sim->config_directory = ex.config_directory;
    sim->verbose = ex.verbose;
    sim->initAlgo();
    ex.lockStepMtx();
    RcsGraph_copy(sim->getGraph(), ex.getGraph());
    sim->getScene()->agents = ex.getScene()->agents;
    ex.unlockStepMtx();
    sim->start();

    return STRNEQ(sim->lastResultMsg.c_str(), "SUCCESS", 7) ? true : false;
  })

  .def("run", [](aff::ExampleActionsECS& ex, std::string actionCommand) -> std::string
  {
    double t_calc = Timer_getSystemTime();
    ex.sequenceCommand = actionCommand;
    ex.start();
    t_calc = Timer_getSystemTime() - t_calc;

    std::string starLine(80, '*');
    std::cerr << "\n\n" + starLine + "\n* LLMSim exits with "
              << ex.getNumFailedActions() << " failed actions";
    std::cerr << " after " << std::to_string(t_calc) << " seconds\n";
    std::cerr << starLine << "\n";

    return ex.lastResultMsg;
  })
  .def("callEvent", [](aff::ExampleActionsECS& ex, std::string eventName)
  {
    ex.getEntity().publish(eventName);
    ex.getEntity().process();
  })
  .def("reset", [](aff::ExampleActionsECS& ex)
  {
    ex.getEntity().publish("ActionSequence", std::string("reset"));
    ex.getEntity().process();
  })
  .def("render", [](aff::ExampleActionsECS& ex)
  {
    ex.getEntity().publish("Render");
    ex.getEntity().process();
  })
  .def("process", [](aff::ExampleActionsECS& ex)
  {
    ex.getEntity().process();
  })
  .def("showGraphicsWindow", [](aff::ExampleActionsECS& ex) -> bool
  {
    bool success = ex.initGraphics();

    if (success)
    {
      ex.getEntity().publish("Render");
      ex.getEntity().process();
    }

    return success;
  })
  .def("showGuis", [](aff::ExampleActionsECS& ex) -> bool
  {
    return ex.initGuis();
  })

  .def("hideGraphicsWindow", [](aff::ExampleActionsECS& ex) -> bool
  {
    if (!ex.viewer)
    {
      return false;
    }

    ex.viewer.reset();

    return true;
  })

  .def("get_state", [](aff::ExampleActionsECS& ex) -> std::string
  {
    return ex.getQuery()->getSceneState().dump();
  })

  .def("get_parent", [](aff::ExampleActionsECS& ex, std::string child) -> std::string
  {
    return ex.getQuery()->getParent(child);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns empty json if there are no objects or a json in the form:
  // {"objects": ['iphone', 'red_glass', 'fanta_bottle'] }
  //////////////////////////////////////////////////////////////////////////////
  .def("get_objects", [](aff::ExampleActionsECS& ex) -> nlohmann::json
  {
    return ex.getQuery()->getObjects();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns empty json if there are no objects or a json in the form:
  // {"agents": ['Daniel', 'Felix', 'Robot'] }
  //////////////////////////////////////////////////////////////////////////////
  .def("get_agents", [](aff::ExampleActionsECS& ex) -> nlohmann::json
  {
    return ex.getQuery()->getAgents();
  })

  .def("step", &aff::ExampleActionsECS::step)
  .def("stop", &aff::ExampleActionsECS::stop)
  .def("isRunning", &aff::ExampleActionsECS::isRunning)

  //////////////////////////////////////////////////////////////////////////////
  // Execute the action command, and return immediately.
  //////////////////////////////////////////////////////////////////////////////
  //.def("execute", &aff::ExampleActionsECS::onActionSequence)
  .def("execute", [](aff::ExampleActionsECS& ex, std::string actionCommand)
  {
    ex.getEntity().publish("ActionSequence", actionCommand);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Execute the action command, and return only after finished.
  //////////////////////////////////////////////////////////////////////////////
  .def("executeBlocking", [](aff::ExampleActionsECS& ex, std::string actionCommand) -> bool
  {
    PollBlockerComponent blocker(&ex);
    ex.getEntity().publish("ActionSequence", actionCommand);
    blocker.wait();
    RLOG_CPP(0, "Finished: " << actionCommand);
    bool success =  STRNEQ(ex.lastResultMsg.c_str(), "SUCCESS", 7);

    RLOG(0, "   success=%s   result=%s", success ? "true" : "false", ex.lastResultMsg.c_str());
    return success;
  })
  .def("getRobotCapabilities", [](aff::ExampleActionsECS& ex)
  {
    return aff::ActionFactory::printToString();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Predict action sequence as tree
  // Call it like: agent.sim.predictActionSequence("get fanta_bottle;put fanta_bottle lego_box;")
  //////////////////////////////////////////////////////////////////////////////
  .def("predictActionSequence", [](aff::ExampleActionsECS& ex, std::string sequenceCommand) -> std::vector<std::string>
  {
    std::string errMsg;
    std::vector<std::string> seq = Rcs::String_split(sequenceCommand, ";");
    auto tree = ex.getQuery()->planActionTree(aff::PredictionTree::SearchType::DFSMT,
                                              seq, ex.getEntity().getDt());
    return tree ? tree->findSolutionPathAsStrings() : std::vector<std::string>();
    //return ex.getQuery()->planActionSequence(seq, seq.size());
  })

  //////////////////////////////////////////////////////////////////////////////
  // Predict action sequence as tree
  // Call it like: agent.sim.planActionSequence("get fanta_bottle;put fanta_bottle lego_box;")
  //////////////////////////////////////////////////////////////////////////////
  .def("planActionSequenceThreaded", [](aff::ExampleActionsECS& ex, std::string sequenceCommand)
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
  .def("planActionSequence", [](aff::ExampleActionsECS& ex, std::string sequenceCommand, bool blocking) -> bool
  {
    std::vector<std::string> seq = Rcs::String_split(sequenceCommand, ";");
    std::string errMsg;

    auto tree = ex.getQuery()->planActionTree(aff::PredictionTree::SearchType::DFSMT,
                                              seq, ex.getEntity().getDt());
    auto res = tree->findSolutionPathAsStrings();

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
    ex.getEntity().publish("ActionSequence", newCmd);

    bool success = true;

    if (blocking)
    {
      blocker.wait();
      success = STRNEQ(ex.lastResultMsg.c_str(), "SUCCESS", 7);
      RLOG(0, "   success=%s   result=%s", success ? "true" : "false", ex.lastResultMsg.c_str());
    }

    RLOG_CPP(0, "Finished: " << newCmd);

    return success;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Predict action sequence as tree
  //////////////////////////////////////////////////////////////////////////////
  .def("plan_fb", [](aff::ExampleActionsECS& ex, std::string sequenceCommand) -> std::string
  {
    PollBlockerComponent blocker(&ex);
    ex.getEntity().publish("PlanDFSEE", sequenceCommand);
    blocker.wait();
    //bool success = STRNEQ(ex.lastResultMsg.c_str(), "SUCCESS", 7);
    bool success = STRNEQ(ex.lastFeedbackMsg[0].c_str(), "SUCCESS", 7);

    std::string fbmsgAsString;
    size_t i = 0;
    for (const auto& fb : ex.lastFeedbackMsg)
    {
      if (i<3)
      {
        fbmsgAsString += fb + " ";
      }
      i++;
    }

    RLOG(0, "Success=%s, Feedback is: \n'%s'", success ? "true" : "false", fbmsgAsString.c_str());

    if (success)
    {
      fbmsgAsString = "SUCCESS";
    }

    return fbmsgAsString;
  })

  .def("plan", [](aff::ExampleActionsECS& ex, std::string sequenceCommand) -> bool
  {
    PollBlockerComponent blocker(&ex);
    ex.getEntity().publish("PlanDFSEE", sequenceCommand);
    blocker.wait();
    //bool success = STRNEQ(ex.lastResultMsg.c_str(), "SUCCESS", 7);
    bool success = STRNEQ(ex.lastFeedbackMsg[0].c_str(), "SUCCESS", 7);
    RLOG(0, "   success=%s   result=%s", success ? "true" : "false", ex.lastResultMsg.c_str());

    return success;
  })

  .def("plan2", [](aff::ExampleActionsECS& ex, std::string sequenceCommand) -> bool
  {
    sequenceCommand = aff::ActionSequence::resolve(ex.getGraph()->cfgFile, sequenceCommand);
    std::vector<std::string> seq = Rcs::String_split(sequenceCommand, ";");
    auto tree = ex.getQuery()->planActionTree(aff::PredictionTree::SearchType::DFSMT,
                                              seq, ex.getEntity().getDt(), 0, true, true);
    if (!tree)
    {
      RLOG_CPP(0, "Could not find solution for: '" << sequenceCommand << "'");
      return false;
    }

    std::vector<std::string> predictedActions;  // Action sequence which will be returned

    auto sln = tree->findSolutionPath();
    for (const auto& nd : sln)
    {
      predictedActions.push_back(nd->actionCommand());
      RLOG_CPP(0, "Action: " << nd->actionCommand() <<
               " cost: " << nd->cost);
    }

    if (predictedActions.empty())
    {
      RLOG_CPP(0, "Could not find solution");
      return false;
    }

    RLOG_CPP(0, "Sequence has " << predictedActions.size() << " steps");

    std::string newCmd;
    for (size_t i = 0; i < predictedActions.size(); ++i)
    {
      newCmd += predictedActions[i];
      newCmd += ";";
    }

    RLOG_CPP(0, "Command : " << newCmd);

    PollBlockerComponent blocker(&ex);
    ex.getEntity().publish("ActionSequence", newCmd);

    bool success = true;
    bool blocking = true;

    if (blocking)
    {
      blocker.wait();
      success = STRNEQ(ex.lastResultMsg.c_str(), "SUCCESS", 7);
      RLOG(0, "   success=%s   result=%s", success ? "true" : "false", ex.lastResultMsg.c_str());
    }

    RLOG_CPP(0, "Finished: " << newCmd);

    return success;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to listen to the Respeaker ROS nose, and to acquire the
  // sound directions, ASR etc.
  //////////////////////////////////////////////////////////////////////////////
  .def("addRespeaker", [](aff::ExampleActionsECS& ex,
                          bool listenWitHandRaisedOnly,
                          bool gazeAtSpeaker,
                          bool speakOut) -> bool
  {
    if (!ex.getScene())
    {
      RLOG(0, "Initialize ExampleActionsECS before adding Respeaker - skipping");
      return false;
    }

    aff::ComponentBase* respeaker = createComponent(ex.getEntity(), ex.getGraph(), ex.getScene(), "-respeaker");
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
  .def("enableASR", [](aff::ExampleActionsECS& ex, bool enable)
  {
    ex.getEntity().publish("EnableASR", enable);
    RLOG(0, "%s ASR", enable ? "Enabling" : "Disabling");
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to listen to the landmarks publishers through ROS, which
  // is for instance the Azure Kinect, and later also the Mediapipe components
  //////////////////////////////////////////////////////////////////////////////
  .def("addLandmarkROS", [](aff::ExampleActionsECS& ex) -> bool
  {
    RCHECK_MSG(ex.getScene(), "Initialize ExampleActionsECS before adding PTU");
    aff::ComponentBase* c = createComponent(ex.getEntity(), ex.getGraph(), ex.getScene(), "-landmarks_ros");
    if (c)
    {
      ex.addComponent(c);
      aff::LandmarkBase* lmc = dynamic_cast<aff::LandmarkBase*>(c);
      RCHECK(lmc);
      lmc->enableDebugGraphics(ex.getViewer());
      return true;
    }

    return true;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to listen to the landmarks publishers through ROS, which
  // is for instance the Azure Kinect, and later also the Mediapipe components
  //////////////////////////////////////////////////////////////////////////////
  .def("addLandmarkZmq", [](aff::ExampleActionsECS& ex) -> bool
  {
    RCHECK_MSG(ex.getScene(), "Initialize ExampleActionsECS before adding PTU");

    RLOG(0, "Adding trackers");
    std::string connection="tcp://localhost:5555";
    double r_agent = DBL_MAX;
    // argP.getArgument("-r_agent", &r_agent, "Radius around skeleton default position to start tracking (default: inf)");

    auto lmc = new aff::LandmarkZmqComponent(&ex.getEntity(), connection);
    ex.addComponent(lmc);   // Takes care of deletion
    lmc->setScenePtr(ex.getGraph(), ex.getScene());

    const RcsBody* cam = RcsGraph_getBodyByName(ex.getGraph(), "camera");
    RCHECK(cam);
    lmc->addArucoTracker(cam->name, "aruco_base");

    // Add skeleton tracker and ALL agents in the scene
    int nSkeletons = lmc->addSkeletonTrackerForAgents(r_agent);
    lmc->enableDebugGraphics(ex.getViewer());
    RLOG(0, "Added skeleton tracker with %d agents", nSkeletons);

    // Initialize all tracker camera transforms from the xml file
    lmc->setCameraTransform(&cam->A_BI);

    ex.getViewer()->setKeyCallback('W', [&ex](char k)
    {
      RLOG(0, "Calibrate camera");
      ex.getEntity().publish("EstimateCameraPose", 20);
    }, "Calibrate camera");

    RLOG(0, "Done adding trackers");

    return true;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to connect to the PTU action server ROS node, and to being
  // able to send pan / tilt commands to the PTU
  //////////////////////////////////////////////////////////////////////////////
  .def("addPTU", [](aff::ExampleActionsECS& ex) -> bool
  {
    RCHECK_MSG(ex.getScene(), "Initialize ExampleActionsECS before adding PTU");
    aff::ComponentBase* c = createComponent(ex.getEntity(), ex.getGraph(), ex.getScene(), "-ptu");
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
  .def("addTTS", [](aff::ExampleActionsECS& ex, std::string type) -> bool
  {
    if (type == "nuance")
    {
      auto c = createComponent(ex.getEntity(), ex.getGraph(),
                               ex.getScene(), "-nuance_tts");
      ex.addComponent(c);
      return c ? true : false;
    }
    else if (type == "native")
    {
      auto c = createComponent(ex.getEntity(), ex.getGraph(),
                               ex.getScene(), "-tts");
      ex.addComponent(c);
      return c ? true : false;
    }

    return false;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to connect to a websocket client. The component receives
  // action commands, and sends back the state.
  //////////////////////////////////////////////////////////////////////////////
  .def("addWebsocket", [](aff::ExampleActionsECS& ex) -> bool
  {
    auto c = createComponent(ex.getEntity(), ex.getGraph(),
                             ex.getScene(), "-websocket");
    ex.addComponent(c);
    return c ? true : false;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to connect to the left Jaco7 Gen2 arm
  //////////////////////////////////////////////////////////////////////////////
  .def("addJacoLeft", [](aff::ExampleActionsECS& ex) -> bool
  {
    auto c = aff::createComponent(ex.getEntity(), ex.getGraph(),
                                  ex.getScene(), "-jacoShm7l");
    ex.addHardwareComponent(c);
    return c ? true : false;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Adds a component to connect to the right Jaco7 Gen2 arm
  //////////////////////////////////////////////////////////////////////////////
  .def("addJacoRight", [](aff::ExampleActionsECS& ex) -> bool
  {
    auto c = aff::createComponent(ex.getEntity(), ex.getGraph(),
                                  ex.getScene(), "-jacoShm7r");
    ex.addHardwareComponent(c);
    return c ? true : false;
  })
  .def("getCompletedActionStack", &aff::ExampleActionsECS::getCompletedActionStack)
  .def("isFinalPoseRunning", &aff::ExampleActionsECS::isFinalPoseRunning)

  //////////////////////////////////////////////////////////////////////////////
  // Expose several internal variables to the python layer
  //////////////////////////////////////////////////////////////////////////////
  .def_readwrite("unittest", &aff::ExampleActionsECS::unittest)
  .def_readwrite("noTextGui", &aff::ExampleActionsECS::noTextGui)
  .def_readwrite("sequenceCommand", &aff::ExampleActionsECS::sequenceCommand)
  .def_readwrite("speedUp", &aff::ExampleActionsECS::speedUp)
  .def_readwrite("xmlFileName", &aff::ExampleActionsECS::xmlFileName)
  .def_readwrite("configDirectory", &aff::ExampleActionsECS::config_directory)
  .def_readwrite("noLimits", &aff::ExampleActionsECS::noLimits)
  .def_readwrite("noCollCheck", &aff::ExampleActionsECS::noCollCheck)   // Set before init()
  .def_readwrite("noTrajCheck", &aff::ExampleActionsECS::noTrajCheck)
  .def_readwrite("verbose", &aff::ExampleActionsECS::verbose)
  .def_readwrite("processingAction", &aff::ExampleActionsECS::processingAction)
  .def_readwrite("noViewer", &aff::ExampleActionsECS::noViewer)
  .def_readwrite("turbo", &aff::ExampleActionsECS::turbo)
  .def_readwrite("maxNumThreads", &aff::ExampleActionsECS::maxNumThreads)
  ;








  //////////////////////////////////////////////////////////////////////////////
  // LandmarkBase perception class wrapper
  //////////////////////////////////////////////////////////////////////////////
  py::class_<aff::LandmarkBase>(m, "LandmarkBase")
  .def(py::init<>())
  .def(py::init<>([](py::object obj)
  {
    aff::ExampleActionsECS* sim = obj.cast<aff::ExampleActionsECS*>();
    RLOG_CPP(1, sim->help());
    auto lm = std::unique_ptr<aff::LandmarkBase>(new aff::LandmarkBase());
    lm->setScenePtr(sim->getGraph(), sim->getScene());

    sim->getEntity().subscribe("PostUpdateGraph", &aff::LandmarkBase::onPostUpdateGraph, lm.get());
    sim->getEntity().subscribe("FreezePerception", &aff::LandmarkBase::onFreezePerception, lm.get());

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
    aff::ExampleActionsECS* sim = obj.cast<aff::ExampleActionsECS*>();
    lm.enableDebugGraphics(sim->getViewer());
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
