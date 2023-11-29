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
#include <ArucoTracker.h>
#include <AzureSkeletonTracker.h>
#include <ExampleLLMSim.h>
#include <ActionFactory.h>
#include <HardwareComponent.h>
#include <RespeakerComponent.h>

#include <ControllerBase.h>
#include <Rcs_resourcePath.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <PhysicsFactory.h>
#include <json.hpp>

#include <SegFaultHandler.h>

#if !defined(_MSC_VER)
#include <X11/Xlib.h>
#endif

#include <chrono>

RCS_INSTALL_ERRORHANDLERS













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

PYBIND11_MODULE(pyAffaction, m)
{
  define_AffordanceTypes(m);

  //////////////////////////////////////////////////////////////////////////////
  // LlmSim
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
  .def("init", [](aff::ExampleLLMSim& ex, bool debug=false)
  {
    bool success = ex.initAlgo();

    if (debug)
    {
      success = ex.initGraphics() && success;
      ex.entity.publish("Render");
      ex.entity.process();
    }

    ex.viewer->setKeyCallback('l', [&ex](char k)
    {
      RLOG(0, "Toggle talk flag");
      ex.entity.publish("ToggleASR");

    }, "Toggle talk flag");

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
  .def("initHardwareComponents", [](aff::ExampleLLMSim& ex)
  {
    ex.entity.initialize(ex.graphC->getGraph());
  }, "Initializes hardware components")
  .def("run", &aff::ExampleLLMSim::startThreaded, py::call_guard<py::gil_scoped_release>(), "Starts endless loop")
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
    ex.entity.publish("TextCommand", std::string("reset"));
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
  .def("hideGraphicsWindow", [](aff::ExampleLLMSim& ex)
  {
    if (!ex.viewer)
    {
      return false;
    }

    ex.viewer.reset();

    return true;
  })

  .def("get_state", &aff::ExampleLLMSim::collectFeedback)
  .def("get_scene_entities", &aff::ExampleLLMSim::getSceneEntities)
  .def("step", &aff::ExampleActionsECS::step)
  .def("stop", &aff::ExampleActionsECS::stop)
  .def("isRunning", &aff::ExampleActionsECS::isRunning)
  .def("setActionSequence", &aff::ExampleActionsECS::onActionSequence)
  .def("getRobotCapabilities", [](aff::ExampleLLMSim& ex)
  {
    return aff::ActionFactory::printToString();
  })
  .def("addRespeaker", [](aff::ExampleLLMSim& ex,
                          bool listenWitHandRaisedOnly,
                          bool gazeAtSpeaker,
                          bool speakOut)
  {
    RCHECK_MSG(ex.actionC, "Initialize ExampleLLMSim before adding Respeaker");
    aff::initROS(HWC_DEFAULT_ROS_SPIN_DT);
    aff::RespeakerComponent* respeaker = new aff::RespeakerComponent(&ex.entity, ex.getScene());
    respeaker->setPublishDialogueWithRaisedHandOnly(listenWitHandRaisedOnly);
    respeaker->setGazeAtSpeaker(gazeAtSpeaker);
    respeaker->setSpeakOutSpeakerListenerText(speakOut);
    ex.addComponent(respeaker);
  })
  .def("addPTU", [](aff::ExampleLLMSim& ex)
  {
    RCHECK_MSG(ex.actionC, "Initialize ExampleLLMSim before adding PTU");
    aff::ComponentBase* c = getComponent(ex.entity, ex.controller->getGraph(), ex.getScene(), "-ptu");
    if (c)
    {
      ex.addHardwareComponent(c);
    }
  })
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
  .def("addJacoLeft", [](aff::ExampleLLMSim& ex)
  {
    auto c = aff::getComponent(ex.entity, ex.controller->getGraph(),
                               ex.getScene(), "-jacoShm7l");
    ex.addHardwareComponent(c);
    return c ? true : false;
  })
  .def("addJacoRight", [](aff::ExampleLLMSim& ex)
  {
    auto c = aff::getComponent(ex.entity, ex.controller->getGraph(),
                               ex.getScene(), "-jacoShm7r");
    ex.addHardwareComponent(c);
    return c ? true : false;
  })
  .def("getCompletedActionStack", &aff::ExampleActionsECS::getCompletedActionStack)
  .def_property("useWebsocket", &aff::ExampleLLMSim::getUseWebsocket, &aff::ExampleLLMSim::setUseWebsocket)
  .def_readwrite("unittest", &aff::ExampleLLMSim::unittest)
  .def_readwrite("noTextGui", &aff::ExampleLLMSim::noTextGui)
  .def_readwrite("sequenceCommand", &aff::ExampleLLMSim::sequenceCommand)
  .def_readwrite("speedUp", &aff::ExampleLLMSim::speedUp)
  .def_readwrite("xmlFileName", &aff::ExampleLLMSim::xmlFileName)
  .def_readwrite("noLimits", &aff::ExampleLLMSim::noLimits)
  .def_readwrite("noTrajCheck", &aff::ExampleLLMSim::noTrajCheck)
  ;








  //////////////////////////////////////////////////////////////////////////////
  // LandmarkBase perception class wrapper
  //////////////////////////////////////////////////////////////////////////////
  py::class_<aff::LandmarkBase>(m, "LandmarkBase")
  .def(py::init<>())
  .def(py::init<>([](py::object obj)
  {
    aff::ExampleLLMSim* sim = obj.cast<aff::ExampleLLMSim*>();
    RLOG_CPP(0, sim->help());
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
  .def("enableDebugGraphics", [](aff::LandmarkBase& lm, py::object obj)
  {
    aff::ExampleLLMSim* sim = obj.cast<aff::ExampleLLMSim*>();

    // Add skeleton graphics
    for (auto& tracker : lm.getTrackers())
    {
      aff::AzureSkeletonTracker* st = dynamic_cast<aff::AzureSkeletonTracker*>(tracker.get());
      if (st)
      {
        st->initGraphics(sim->viewer.get());
      }
    }

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
  .def("getPanTilt", [](aff::LandmarkBase& lm, std::string roboAgent, std::string gazeTarget)
  {
    RLOG(0, "Pan tilt angle calculation");
    const aff::Agent* robo_ = lm.getScene()->getAgent(roboAgent);
    if (!robo_)
    {
      RLOG(0, "Robo agent '%s' not found", roboAgent.c_str());
      double buf;
      MatNd tmp = MatNd_fromPtr(0, 0, &buf);
      return pybind11::detail::MatNd_toNumpy(&tmp);
    }

    double panTilt[2], err[2];
    size_t maxIter = 100;
    double eps = 1.0e-8;
    const aff::RobotAgent* robo = dynamic_cast<const aff::RobotAgent*>(robo_);

    double t_calc = Timer_getSystemTime();
    int iter = robo->getPanTilt(lm.getGraph(), gazeTarget.c_str(), panTilt, maxIter, eps, err);

    if (iter==-1)
    {
      RLOG(0, "Pan tilt computation failed");
      double buf;
      MatNd tmp = MatNd_fromPtr(0, 0, &buf);
      return pybind11::detail::MatNd_toNumpy(&tmp);
    }

    t_calc = Timer_getSystemTime() - t_calc;

    RLOG(0, "pan=%.1f tilt=%.1f err=%f %f took %d iterations and %.1f msec",
         RCS_RAD2DEG(panTilt[0]), RCS_RAD2DEG(panTilt[1]), err[0], err[1], iter, 1.0e3*t_calc);

    MatNd tmp = MatNd_fromPtr(2, 1, panTilt);
    return pybind11::detail::MatNd_toNumpy(&tmp);
  })
  ;







  //////////////////////////////////////////////////////////////////////////////
  // Rcs function wrappers
  //////////////////////////////////////////////////////////////////////////////
  py::class_<Rcs::ControllerBase>(m, "ControllerBase")
  .def(py::init<std::string>())
  .def("print", &Rcs::ControllerBase::print)
  ;

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

  // Check if physics engine is available (can't list, unfortunately)
  m.def("supportsPhysicsEngine", &Rcs::PhysicsFactory::hasEngine);

  //
  m.def("printPhysicsEngines", &Rcs::PhysicsFactory::print);

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
