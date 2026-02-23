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
#include <QTimer>
#include <QApplication>
#include <QMetaObject>
#include <QThread>
#include <QDebug>

#ifdef slots
#  undef slots
#endif

#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "pybind11_json.hpp"
#include "pybind_dict_utils.h"
#include "type_casters.h"

namespace py = pybind11;

#include <LandmarkBase.h>
#include <ExampleActionsECS.h>
#include <ExampleViapoints.h>
#include <ActionFactory.h>
#include <ActionSequence.h>
#include <HardwareComponent.h>
#include <LandmarkZmqComponent.h>
#include <TTSComponent.h>
#include <PredictionTree.h>
#include <AzureSkeletonTracker.h>
#include <ActionEyeGaze.h>
#include <SceneHelpers.h>
#include <EyeModelIKComponent.h>
#include "ZmqRouterComponent.h"

#include <Rcs_resourcePath.h>
#include <Rcs_macros.h>
#include <Rcs_quaternion.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_utilsCPP.h>
#include <json.hpp>

#include <SegFaultHandler.h>

#if !defined(_MSC_VER) && !defined(__APPLE__)
#include <X11/Xlib.h>
#endif

#include <chrono>
#include <vector>
#include <tuple>
#include <algorithm>
#include <locale.h>

#include "py_components.hpp"
#include "py_planning.hpp"
#include "py_mirror_eyes.hpp"
#include "py_virtual_camera.hpp"
#include "py_exploration.hpp"

RCS_INSTALL_ERRORHANDLERS





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
  auto cls = py::class_<aff::ExampleActionsECS>(m, "LlmSim")
             .def(py::init<>([]()
  {
#if !defined(_MSC_VER) && !defined(__APPLE__)// Avoid crashes when running remotely.
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
  // Initialization function, to be called after member variables have been
  // configured.
  //////////////////////////////////////////////////////////////////////////////
  .def("initBlocking", [](aff::ExampleActionsECS& ex, bool headless, bool catch_keyboard_interrupt) -> int
  {
    // Release the GIL for the function's duration
    pybind11::gil_scoped_release release_gil;

    ex.blockingMainThread = true;
    bool success = ex.initAlgo();

    int argc = 1;
    char* argv[] = { (char*)"AppName" };
    QApplication app(argc, argv);

    std::setlocale(LC_ALL, "C");
    QApplication::setQuitOnLastWindowClosed(false);

    if (!headless)
    {
      success = ex.initGraphics() && success;
      success = ex.initGuis() && success;
    }

    std::thread t(&aff::ExampleActionsECS::start, &ex);
    t.detach();

    std::atomic<bool> kb_int{false};   // To be sure
    const int dt_msec = 16;  // ~60fps
    const int cycles_per_sec = 200/dt_msec;   // 5 Hz
    int loopCount = 0;
    QTimer* timer = new QTimer(&app);  // or any parent
    QObject::connect(timer, &QTimer::timeout, [&]()
    {
      ex.updateUI();

      // Safely check Python signals by acquiring the GIL first, once per second.
      if (catch_keyboard_interrupt && (++loopCount%cycles_per_sec==0))
      {
        pybind11::gil_scoped_acquire guard;

        if (PyErr_CheckSignals() != 0 &&
            PyErr_ExceptionMatches(PyExc_KeyboardInterrupt))
        {
          // 1 clear so ~gil_scoped_acquire won't throw
          PyErr_Clear();
          kb_int.store(true, std::memory_order_relaxed);

          // 2 quit Qt cleanly
          QCoreApplication::quit();
        }
      }

    });
    timer->start(dt_msec);

    int res = app.exec();

    /* ---------- back in the outer C++ stack ---------- */
    if (catch_keyboard_interrupt && kb_int.load())
    {
      pybind11::gil_scoped_acquire guard;       // need GIL
      PyErr_SetNone(PyExc_KeyboardInterrupt);   // restore
      throw pybind11::error_already_set();      // safe to throw now
    }


    return res;

  },
  "Initializes algorithm, guis and graphics",
  py::arg("headless") = false,
  py::arg("catch_keyboard_interrupt") = false)

  //////////////////////////////////////////////////////////////////////////////
  // Initialization function, to be called after member variables have been
  // configured.
  //////////////////////////////////////////////////////////////////////////////
  .def("quitBlocking", [](aff::ExampleActionsECS& ex) -> bool
  {
    ex.stop();

    // This can be done in a standard std::thread, pthread, or any non-Qt thread
    QMetaObject::invokeMethod(qApp, []()
    {
      RLOG_CPP(1, "Quitting from thread:" << QThread::currentThread());
      QCoreApplication::quit();
    }, Qt::QueuedConnection);

    return true;

  }, "Stops example")



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
  // Returns the position of the object in camera coordinates
  //////////////////////////////////////////////////////////////////////////////
  .def("getObjectInCamera", [](aff::ExampleActionsECS& ex, std::string objectName, std::string cameraName) -> nlohmann::json
  {
    return ex.getQuery()->getObjectInCamera(objectName, cameraName);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns the position of the objects in camera coordinates
  //////////////////////////////////////////////////////////////////////////////
  .def("getObjectsInCamera", [](aff::ExampleActionsECS& ex, std::vector<std::string> entityNames, std::string cameraName) -> nlohmann::json
  {
    return ex.getQuery()->getObjectsInCamera(entityNames, cameraName);
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

  //////////////////////////////////////////////////////////////////////////////
  // Calls the run method in a new thread, releases the GIL and returns to the
  // python context (e.g. console).
  //////////////////////////////////////////////////////////////////////////////
  .def("run", &aff::ExampleActionsECS::startThreaded, py::call_guard<py::gil_scoped_release>(), "Starts endless loop")

  //////////////////////////////////////////////////////////////////////////////
  // Calls an event without arguments. We must not call process() here, since
  // this method might run concurrently to the event queue thread.
  //////////////////////////////////////////////////////////////////////////////
  .def("callEvent", [](aff::ExampleActionsECS& ex, std::string eventName)
  {
    ex.getEntity().publish(eventName);
  })

  //////////////////////////////////////////////////////////////////////////////
  // -1: grow down, 0: symmetric, 1: grow up
  // Returns number of changed shapes
  // Fatal error if body
  //////////////////////////////////////////////////////////////////////////////
  .def("changeShapeHeight", [](aff::ExampleActionsECS& ex, std::string nttName, double height, int growMode) -> size_t
  {
    auto ntts = ex.getScene()->getAffordanceEntities(nttName);
    RcsGraph* ikGraph = ex.getGraph();

    for (const auto& ntt : ntts)
    {
      const RcsBody* bdy = ntt->body(ikGraph);
      RCHECK_MSG(bdy->nShapes>0, "Body %s has no shapes attached", ntt->bdyName.c_str());
      const RcsShape* sh = &bdy->shapes[0];
      std::vector<double> newOrigin(sh->A_CB.org, sh->A_CB.org+3);
      newOrigin[2] += 0.5*growMode*(height-sh->extents[2]);

      ex.getEntity().publish("ChangeShapeHeight", ikGraph, ntt->bdyName, height);
      ex.getEntity().publish("ChangeShapeOrigin", ikGraph, ntt->bdyName, newOrigin);
    }

    return ntts.size();
  })
  .def("changeShapeDiameter", [](aff::ExampleActionsECS& ex, std::string nttName, double diameter) -> size_t
  {
    auto ntts = ex.getScene()->getAffordanceEntities(nttName);
    RcsGraph* ikGraph = ex.getGraph();

    for (const auto& ntt : ntts)
    {
      ex.getEntity().publish("ChangeShapeDiameter", ikGraph, ntt->bdyName, diameter);
    }

    return ntts.size();
  })
  .def("changeBodyOrigin", [](aff::ExampleActionsECS& ex, std::string bodyName, double x, double y, double z)
  {
    std::vector<double> org {x, y, z};
    RcsGraph* ikGraph = ex.getGraph();

    ex.getEntity().publish("ChangeBodyOrigin", ikGraph, bodyName, org);
  })
  .def("changeShapeOrigin", [](aff::ExampleActionsECS& ex, std::string bodyName, double x, double y, double z)
  {
    std::vector<double> org {x, y, z};
    RcsGraph* ikGraph = ex.getGraph();
    RLOG(1, "***");
    ex.getEntity().publish("ChangeShapeOrigin", ikGraph, bodyName, org);
  })
  .def("reset", [](aff::ExampleActionsECS& ex)
  {
    ex.getEntity().publish("ActionSequence", std::string("reset"));
  })
  .def("render", [](aff::ExampleActionsECS& ex)
  {
    ex.getEntity().publish("Render");
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
    }

    return success;
  })
  .def("showGuis", [](aff::ExampleActionsECS& ex) -> bool
  {
    return ex.initGuis();
  })
  .def("hideGraphicsWindow", [](aff::ExampleActionsECS& ex) -> bool
  {
    return ex.eraseViewer();
  })
  .def("get_state", [](aff::ExampleActionsECS& ex) -> std::string
  {
    return ex.getQuery()->getSceneState().dump();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Call tts event
  //////////////////////////////////////////////////////////////////////////////
  .def("speak", [](aff::ExampleActionsECS& ex, std::string text)
  {
    nlohmann::json payload =
    {
      {"type", "tts"},
      {"cmd",  "SAY"},
      {"text", text}
    };

    ex.getEntity().publish("TriggerPerception", std::string("tts"), 0, payload.dump());
    //ex.getEntity().publish("Speak", text);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns the entire scene in a URDF format
  //////////////////////////////////////////////////////////////////////////////
  .def("get_state_urdf", [](aff::ExampleActionsECS& ex) -> std::string
  {
    return ex.getQuery()->getURDF();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns the entity of which child is a child of, or an empty string
  //////////////////////////////////////////////////////////////////////////////
  .def("get_parent_entity", [](aff::ExampleActionsECS& ex, std::string child) -> std::string
  {
    return ex.getQuery()->getParentEntity(child);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Looks for the topoligical parent entity, and determines the closest frame
  // with the given type. Returns an empty string if none is found, or the name
  // of the affordance frame
  //////////////////////////////////////////////////////////////////////////////
  .def("get_closest_parent_affordance", [](aff::ExampleActionsECS& ex,
                                           std::string child, std::string affordanceType) -> std::string
  {
    return ex.getQuery()->getClosestParentAffordance(child, affordanceType);
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
  // Returns an array of positions and quaternions for the set of bodies passed.
  //////////////////////////////////////////////////////////////////////////////
  .def("get_body_transforms", [](aff::ExampleActionsECS& ex,
                                 const std::vector<std::string>& bodyNames,
                                 bool useCurrentGraph) -> py::tuple
  {
    if (bodyNames.empty())
    {
      auto pos  = py::array_t<double>(py::array::ShapeContainer{py::ssize_t(0), py::ssize_t(3)});
      auto quat = py::array_t<double>(py::array::ShapeContainer{py::ssize_t(0), py::ssize_t(4)});
      return py::make_tuple(pos, quat);
    }

    auto transforms = ex.getQuery()->getBodyTransforms(bodyNames, useCurrentGraph);

    if (transforms.size() != bodyNames.size())
    {
      throw py::value_error("Body transform computation failed (no results returned).");
    }

    const py::ssize_t N = static_cast<py::ssize_t>(transforms.size());

    py::array_t<double> pos(py::array::ShapeContainer{N, py::ssize_t(3)});
    py::array_t<double> quat(py::array::ShapeContainer{N, py::ssize_t(4)});

    double* pd = pos.mutable_data();   // contiguous N*3
    double* qd = quat.mutable_data();  // contiguous N*4

    for (py::ssize_t i = 0; i < N; ++i)
    {
      auto& trf = transforms[i];
      Vec3d_copy(pd + 3*i, trf.org);
      Quat_fromRotationMatrix(qd + 4*i, trf.rot);
    }

    return py::make_tuple(pos, quat);
  },
  py::arg("bodyNames"),
  py::arg("useCurrentGraph") = true,
  R"doc(
Return body transforms as two NumPy arrays: (pos, quat). If one of the elements in bodyNames
cannot be retrieved, a value_error is thrown.

pos:  (N,3) float64 array [px, py, pz]
quat: (N,4) float64 array [qw, qx, qy, qz]
)doc"
)

  //////////////////////////////////////////////////////////////////////////////
  // Returns a json in the form:
  // {"hand_name_1": ['iphone', 'red_glass', 'fanta_bottle'],
  //  "hand_name_2": ['iphone', 'green_glass', 'milk_bottle']}
  // where the values are the reachable objects from the manipulator (key).
  // This function assumes a RobotAgent to be passed
  //////////////////////////////////////////////////////////////////////////////
  .def("get_object_reachabilities", [](aff::ExampleActionsECS& ex, std::string agentName) -> nlohmann::json
  {
    return ex.getQuery()->getObjectReachabilities(agentName);
  },
  py::arg("agentName") = std::string())

  //////////////////////////////////////////////////////////////////////////////
  // Same as above, but with internal simulation
  //////////////////////////////////////////////////////////////////////////////
  .def("get_object_graspabilities", [](aff::ExampleActionsECS& ex, std::string agentName) -> nlohmann::json
  {
    return ex.getQuery()->getObjectGraspabilities(agentName);
  },
  py::arg("agentName") = std::string())

  //////////////////////////////////////////////////////////////////////////////
  // Returns empty json if there are no objects or a json in the form:
  // {"agents": ['Daniel', 'Felix', 'Robot'] }
  //////////////////////////////////////////////////////////////////////////////
  .def("get_agents", [](aff::ExampleActionsECS& ex, bool onlyVisibleAgents) -> nlohmann::json
  {
    return ex.getQuery()->getAgents();
  },
  py::arg("onlyVisibleAgents") = false)

  //////////////////////////////////////////////////////////////////////////////
  // Returns gaze data as a JSON object. If no gaze data is available,
  // it returns an empty JSON object. The JSON is structured as:
  // {
  //     "agent_name": "AgentName",
  //     "gaze_data": [
  //         {
  //             "time": 123.45,
  //             "gaze_velocity": 5.67,
  //             "objects": [
  //                 {
  //                     "name": "ObjectName",
  //                     "angle_diff": 12.34,
  //                     "distance": 1.23,
  //                     "angle_diffXY": 5.67,
  //                     "angle_diffXZ": 8.90
  //                 }
  //             ]
  //         }
  //     ]
  // }
  //////////////////////////////////////////////////////////////////////////////
  .def("get_gaze_data", [](aff::ExampleActionsECS& ex) -> nlohmann::json
  {
    return ex.getQuery()->getGazeData();
  })
  .def("get_recorded_transformations", [](aff::ExampleActionsECS& ex, double start_time, double end_time) -> nlohmann::json
  {
    return ex.getQuery()->getRecordedTransformations(start_time, end_time);
  })

  .def("load_transformation_data_from_file", [](aff::ExampleActionsECS& ex, std::string filename)
  {
    ex.getQuery()->loadTransformationDataFromFile(filename);
  })

  .def("start_playback_transformation_data", [](aff::ExampleActionsECS& ex)
  {
    ex.getQuery()->startPlaybackTransformationData();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns an empty string if there are no objects held in the hand, or the
  // name of the holding hand
  //////////////////////////////////////////////////////////////////////////////
  .def("is_held_by", [](aff::ExampleActionsECS& ex, std::string ntt) -> std::string
  {
    return ex.getQuery()->getHoldingHand(ntt);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns an empty string if there are no objects held in the hand, or the
  // name of the holding hand
  //////////////////////////////////////////////////////////////////////////////
  .def("get_objects_held_by", [](aff::ExampleActionsECS& ex, std::string agent) -> nlohmann::json
  {
    return ex.getQuery()->getObjectsHeldBy(agent);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns a json string with the affordance description of an entity
  //////////////////////////////////////////////////////////////////////////////
  .def("getAffordanceFrame", [](aff::ExampleActionsECS& ex, std::string bodyName, aff::Affordance::Type affordanceType) -> nlohmann::json
  {
    std::vector<std::string> frames;
    nlohmann::json data;

    const aff::AffordanceEntity* entity = ex.getScene()->getAffordanceEntity(bodyName);

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

    for (const auto& f : frames)
    {
      const RcsBody* b = RcsGraph_getBodyByName(ex.getGraph(), f.c_str());

      data[f] = {};
      data[f]["position"] = b->A_BI.org;

      double ea[3];
      Mat3d_toEulerAngles(ea, (double(*)[3])b->A_BI.rot);
      data[f]["euler_xyzr"] = ea;
    }

    return data;
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
  // Sets the path for finding the piper TTS executables, libraries and voices
  //////////////////////////////////////////////////////////////////////////////
  .def_static("setPiperPath", &aff::TTSComponent::setPiperPath)

  .def("getCompletedActionStack", &aff::ExampleActionsECS::getCompletedActionStack)
  .def("isFinalPoseRunning", &aff::ExampleActionsECS::isFinalPoseRunning)
  .def("isProcessingAction", &aff::ExampleActionsECS::isProcessingAction)
  .def("step", &aff::ExampleActionsECS::step)
  .def("stop", &aff::ExampleActionsECS::stop)
  .def("isRunning", &aff::ExampleActionsECS::isRunning)
  .def("addComponentArgument", &aff::ExampleActionsECS::addComponentArgument)
  .def("cleanup", &aff::ExampleActionsECS::cleanup)

  //////////////////////////////////////////////////////////////////////////////
  // Scales the durations of actions (global scope)
  //////////////////////////////////////////////////////////////////////////////
  .def("setDurationScaling", [](aff::ExampleActionsECS& ex, double value)
  {
    aff::PredictionTree::setTurboDurationScaler(value);
  })
  .def("setDefaultDurationScaling", [](aff::ExampleActionsECS& ex)
  {
    aff::PredictionTree::setTurboDurationScaler(aff::PredictionTree::getDefaultTurboDurationScaler());
  })
  .def("getDurationScaling", [](aff::ExampleActionsECS& ex) -> double
  {
    return aff::PredictionTree::getTurboDurationScaler();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Sets the gaze target body for the eye gaze model. This only takes effect
  // if the class has been initialized with the eyeIkEnabled flag set to true.
  //////////////////////////////////////////////////////////////////////////////
  .def("setGazeTarget", [](aff::ExampleActionsECS& ex, std::string targetBody) -> std::string
  {
    std::vector<const aff::AffordanceEntity*> ntts = ex.getScene()->getAffordanceEntities(targetBody);

    if (!ntts.empty())
    {
      if (ntts.size() != 1)
      {
        RLOG_CPP(1, "Found several object with the name " << targetBody << " - taking first one.");
      }

      targetBody = ntts[0]->bdyName;
    }
    else
    {
      const aff::Agent* agent = ex.getScene()->getAgent(targetBody);

      if (agent)
      {
        // From here on, we have a valid agent. We look at its (first) head
        auto m = agent->getManipulatorsOfType(ex.getScene(), "head");

        if (m.empty())
        {
          RLOG_CPP(1, "Agent " << targetBody << " has no head to look at. Looking at body.");
          targetBody = agent->bdyName;
        }
        else
        {
          targetBody = m[0]->bdyName;
        }

      }

    }

    ex.getEntity().publish("SetGazeTarget", targetBody);

    return targetBody;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Pausing and interrupting trajectories
  //////////////////////////////////////////////////////////////////////////////
  .def("clearTrajectory", [](aff::ExampleActionsECS& ex)
  {
    ex.getEntity().publish("ClearTrajectory");
  })
  .def("pauseTrajectory", [](aff::ExampleActionsECS& ex)
  {
    ex.getEntity().publish("PauseTrajectory");
  })
  .def("resumeTrajectory", [](aff::ExampleActionsECS& ex)
  {
    ex.getEntity().publish("ResumeTrajectory");
  })

  //////////////////////////////////////////////////////////////////////////////
  // Expose several internal variables to the python layer
  //////////////////////////////////////////////////////////////////////////////
  .def_readwrite("unittest", &aff::ExampleActionsECS::unittest)
  .def_readwrite("noTextGui", &aff::ExampleActionsECS::noTextGui)
  .def_readwrite("speedUp", &aff::ExampleActionsECS::speedUp)
  .def_readwrite("xmlFileName", &aff::ExampleActionsECS::xmlFileName)
  .def_readwrite("configDirectory", &aff::ExampleActionsECS::configDirectory)
  .def_readwrite("noLimits", &aff::ExampleActionsECS::noLimits)
  .def_readwrite("noCollCheck", &aff::ExampleActionsECS::noCollCheck)   // Set before init()
  .def_readwrite("noTrajCheck", &aff::ExampleActionsECS::noTrajCheck)
  .def_readwrite("hasBeenStopped", &aff::ExampleActionsECS::hasBeenStopped)
  .def_readwrite("verbose", &aff::ExampleActionsECS::verbose)
  .def_readwrite("noViewer", &aff::ExampleActionsECS::noViewer)
  .def_readwrite("virtualCameraWidth", &aff::ExampleActionsECS::virtualCameraWidth)
  .def_readwrite("virtualCameraHeight", &aff::ExampleActionsECS::virtualCameraHeight)
  .def_readwrite("virtualCameraEnabled", &aff::ExampleActionsECS::virtualCameraEnabled)
  .def_readwrite("virtualCameraWindowEnabled", &aff::ExampleActionsECS::virtualCameraWindowEnabled)
  .def_readwrite("turbo", &aff::ExampleActionsECS::turbo)
  .def_readwrite("maxNumThreads", &aff::ExampleActionsECS::maxNumThreads)
  .def_readwrite("numSceneQueries", &aff::ExampleActionsECS::numSceneQueries)
  .def_readwrite("eyeIkEnabled", &aff::ExampleActionsECS::eyeIkEnabled)
  .def_readwrite("eventQueue", &aff::ExampleActionsECS::eventQueue)
  .def_readwrite("dt", &aff::ExampleActionsECS::dt)
  .def_readwrite("enableWireframeToggle", &aff::ExampleActionsECS::enableWireframeToggle)
  .def_readwrite("enableRealGraphVisualization", &aff::ExampleActionsECS::enableRealGraphVisualization)

  //////////////////////////////////////////////////////////////////////////////
  // GazeDisambiguation
  //////////////////////////////////////////////////////////////////////////////
  .def_readwrite("sceneTransformationDataRecorderEnabled", &aff::ExampleActionsECS::sceneTransformationDataRecorderEnabled)
  .def_readwrite("sceneTransformationDataPlayerEnabled", &aff::ExampleActionsECS::sceneTransformationDataPlayerEnabled)
  .def_readwrite("usersGazeComponentEnabled", &aff::ExampleActionsECS::usersGazeComponentEnabled)

  //////////////////////////////////////////////////////////////////////////////
  // viaPoint action
  //////////////////////////////////////////////////////////////////////////////
  .def("getControls", [](aff::ExampleActionsECS& ex, std::vector<std::string> endeffectors) -> nlohmann::json
  {
    nlohmann::json controls;

    RCSGRAPH_TRAVERSE_JOINTS(ex.getGraph())
    {
      if (!JNT->constrained)
      {
        controls["Joints"][JNT->name] = ex.getGraph()->q->ele[JNT->jointIndex];
      }
    }

    for (const auto& ee : endeffectors)
    {
      const RcsBody* bdy = RcsGraph_getBodyByName(ex.getGraph(), ee.c_str());
      if (bdy)
      {
        std::vector<double> position = std::vector<double>(bdy->A_BI.org, bdy->A_BI.org+3);
        std::vector<double> rotation(4, 0.0);
        Quat_fromRotationMatrix(rotation.data(), MAT3D_CAST bdy->A_BI.rot);

        controls["Endeffector"][ee]["position"] = position;
        controls["Endeffector"][ee]["quaternion"] = rotation;
      }
      else
      {
        RLOG_CPP(0, "Body '" << ee << "' not found in graph");
      }
    }

    return controls;
  },
  py::arg("endeffectors") = std::vector<std::string>())

  //////////////////////////////////////////////////////////////////////////////
  // viaPoint action
  //////////////////////////////////////////////////////////////////////////////
  .def("createActionFile", [](aff::ExampleActionsECS& ex, std::string inputFile)
  {
    RLOG(0, "Creating action file");

    const bool pickAndPlace = false;
    bool success = aff::ExamplePlayBackViapoints::createActionFile(inputFile, "action_iros.xml", pickAndPlace);

    if (!success)
    {
      RLOG(0, "Failed to create action file");
    }
    else
    {
      RLOG(0, "Successfully created action file");
    }

  },
  py::arg("inputFile") = "test_robot_traj.txt")
  .def("createViaPointAction", [](aff::ExampleActionsECS& ex, std::string inputFile)
  {
    RLOG(0, "Creating action file");

    const bool pickAndPlace = false;
    bool success = aff::ExamplePlayBackViapoints::createActionFile(inputFile, "action_iros.xml", pickAndPlace);

    if (!success)
    {
      RLOG(0, "Failed to create action file");
    }
    else
    {
      ex.getEntity().publish("PlanDFSEE", std::string("load action_iros.xml; pose default_top"));
    }

  },
  py::arg("inputFile") = "test_robot_traj.txt")
  ;

  bind_components(cls);
  bind_planning(cls);
  bind_mirror_eyes(cls);
  bind_virtual_camera(cls);
  bind_exploration(cls);







  //////////////////////////////////////////////////////////////////////////////
  // LandmarkBase perception class wrapper
  //////////////////////////////////////////////////////////////////////////////
  py::class_<aff::LandmarkBase>(m, "LandmarkBase")
  .def(py::init<>([](py::object obj)
  {
    aff::ExampleActionsECS* sim = obj.cast<aff::ExampleActionsECS*>();
    RLOG_CPP(1, sim->help());
    auto lm = std::unique_ptr<aff::LandmarkBase>(new aff::LandmarkBase());

    sim->getEntity().subscribe("UpdateScene", &aff::LandmarkBase::onUpdateScene, lm.get());
    sim->getEntity().subscribe("FreezePerception", &aff::LandmarkBase::onFreezePerception, lm.get());

    return std::move(lm);
  }))
  .def("addArucoTracker", &aff::LandmarkBase::addArucoTracker)
  .def("addSkeletonTrackerForAgents", [](aff::LandmarkBase& lm, py::object sim_, double r, std::string camera) -> int
  {
    aff::ExampleActionsECS* sim = sim_.cast<aff::ExampleActionsECS*>();
    if (!sim->getScene())
    {
      RLOG(0, "Can't add skeleton tracker for agents - scene has not been set");
      return 0;
    }

    int numHumanAgents = 0;
    for (const auto& agent : sim->getScene()->agents)
    {
      if (dynamic_cast<aff::HumanAgent*>(agent))
      {
        numHumanAgents++;
      }
    }

    if (numHumanAgents == 0)
    {
      RLOG(0, "Can't add skeleton tracker for agents - no human agent found");
      return 0;
    }

    auto tracker = new aff::AzureSkeletonTracker(numHumanAgents, camera);
    lm.addTracker(std::unique_ptr<aff::AzureSkeletonTracker>(tracker));
    tracker->addAgents(sim->getScene());
    tracker->setSkeletonDefaultPositionRadius(r);
    tracker->registerAgentAppearDisappearCallback([sim](const std::string& agentName, bool appear)
    {
      std::string appearStr = appear ? " appeared" : " disappered";
      RLOG_CPP(0, "Agent " << agentName << appearStr);
      sim->getEntity().publish("AgentChanged", agentName, appear);
    });

    return numHumanAgents;
  })
  .def("setJsonInput", &aff::LandmarkBase::setJsonInput)
  .def("startCalibration", &aff::LandmarkBase::startCalibration)
  .def("isCalibrating", &aff::LandmarkBase::isCalibrating)
  .def("setSyncInputWithWallclock", &aff::LandmarkBase::setSyncInputWithWallclock)
  .def("getSyncInputWithWallclock", &aff::LandmarkBase::getSyncInputWithWallclock)
  .def("enableDebugGraphics", [](aff::LandmarkBase& lm, py::object obj)
  {
    aff::ExampleActionsECS* sim = obj.cast<aff::ExampleActionsECS*>();
    lm.enableDebugGraphics(sim->getViewer());
  })
  .def("getAffordanceFrame", [](aff::LandmarkBase& lm, std::string bodyName, aff::Affordance::Type affordanceType) -> nlohmann::json
  {
    nlohmann::json data;

    RFATAL("This method is now part of the LlmSim class - please don't call it from this here");

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
