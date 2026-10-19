/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

*******************************************************************************/

#include <QTimer>
#include <QApplication>
#include <QMetaObject>
#include <QThread>
#include <QDebug>

#ifdef slots
#  undef slots
#endif

#include <ExampleTeleOp.h>
#include <SegFaultHandler.h>
#include <Rcs_resourcePath.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>

#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11_json.hpp>

#if !defined(_MSC_VER) && !defined(__APPLE__)
#include <X11/Xlib.h>
#endif

#include <stdexcept>
#include <algorithm>
#include <limits>
#include <iostream>
#include <iomanip>



RCS_INSTALL_ERRORHANDLERS

namespace py = pybind11;



static py::array_t<float> convert_policy(py::array_t<float, py::array::c_style> trajectory,
                                         size_t num_steps, std::vector<double> curr_wrench,
                                         double dx_max, double dphi_max, bool verbose)
{
  if ((trajectory.ndim() != 2) ||
      (trajectory.shape(1) != 9) ||
      (num_steps > static_cast<size_t>(trajectory.shape(0))) ||
      (dx_max <= 0.0) ||
      (dphi_max <= 0.0))
  {
    throw std::runtime_error("Invalid trajectory input");
  }

  // Enforce dtype exactly float32 (no implicit casting; no forcecast)
  if (!py::dtype::of<float>().is(trajectory.dtype()))
  {
    throw std::runtime_error("Input must be np.float32");
  }

  if (curr_wrench.size() != 6)
  {
    throw std::runtime_error("Wrench must have size 6");
  }

  if (num_steps < 2)
  {
    throw std::runtime_error("num_steps must be larger than 2");
  }

  if (num_steps > (size_t)trajectory.shape(0))   // usually 16 or 32
  {
    throw std::runtime_error("num_steps must be smaller or equal number of trajectory steps");
  }


  //////////////////////////////////////////////////////////////////////////
  // // Convert input trajectory array to transform sequence
  //////////////////////////////////////////////////////////////////////////
  std::vector<HTr> in_trf;
  in_trf.reserve(num_steps);
  auto trajectory_u = trajectory.unchecked<2>();
  for (size_t i=0; i<num_steps; ++i)
  {
    HTr trf;

    // In the first step, we take the measured robot's wrench
    if (i==0)
    {
      HTr_from6DVector(&trf, curr_wrench.data());
    }
    // In the consecutive steps, we use the previous prediction
    else
    {
      // Position is first 3 elements
      trf.org[0] = trajectory_u(i-1, 0);
      trf.org[1] = trajectory_u(i-1, 1);
      trf.org[2] = trajectory_u(i-1, 2);

      // Build rotation matrix from rows 0 and 1 (cols 3..8)
      for (int k = 0; k < 3; ++k)
      {
        trf.rot[0][k] = static_cast<double>(trajectory_u(i-1, 3 + k)); // rot row 0
        trf.rot[1][k] = static_cast<double>(trajectory_u(i-1, 6 + k)); // rot row 1
      }
      Vec3d_crossProduct(trf.rot[2], trf.rot[0], trf.rot[1]);
    }

    if (!Mat3d_isValid(trf.rot))
    {
      RLOG_CPP(0, "Invalid rotation matrix!");
      throw std::runtime_error("Invalid rotation matrix!");
    }

    in_trf.push_back(trf);
  }

  if (verbose)
  {
    for (size_t i=0; i<in_trf.size(); ++i)
    {
      double x[6];
      HTr_to6DVector(x, &in_trf[i]);

      std::cout << std::fixed << std::setprecision(6)
                << "in_trf[" << i << "]: " << x[0] << " " << x[1] << " " << x[2] << " "
                //<< x[3] << " " << x[4] << " " << x[5]
                << x[3]*180.0/M_PI << " " << x[4]*180.0/M_PI << " " << x[5]*180.0/M_PI
                << std::defaultfloat << std::endl;

    }
  }

  //////////////////////////////////////////////////////////////////////////
  // Determine required number of subdivisions
  //////////////////////////////////////////////////////////////////////////
  size_t subdivisions = 1;
  for (size_t i=1; i<num_steps; ++i)
  {
    const double dx_step = Vec3d_distance(in_trf[i].org, in_trf[i-1].org);
    size_t needed = static_cast<size_t>(std::ceil(dx_step/dx_max));
    subdivisions = std::max(subdivisions, needed);

    // Avoid ceil to jump to next larger value due to floating point round-off error
    const double dphi_step = Mat3d_diffAngle(in_trf[i].rot, in_trf[i-1].rot);
    needed = static_cast<size_t>(std::ceil(dphi_step/dphi_max));
    subdivisions = std::max(subdivisions, needed);
  }

  if (verbose)
  {
    std::cout << "Subdivisions: " << subdivisions << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////
  // Subdivision loop
  //
  // subdivisions   j   ratio
  //
  //            2   0   0.0
  //            2   1   0.5
  //
  //            3   0   0.0
  //            3   1   0.33
  //            3   2   0.66
  //////////////////////////////////////////////////////////////////////////
  std::vector<HTr> out;
  out.reserve((num_steps-1)*subdivisions+1);
  for (size_t i=1; i<num_steps; ++i)
  {
    double dx_vec[3];
    Vec3d_sub(dx_vec, in_trf[i].org, in_trf[i-1].org);

    for (size_t j=0; j<subdivisions; ++j)
    {
      const double ratio = static_cast<double>(j) / static_cast<double>(subdivisions);
      HTr intermediate;

      Vec3d_constMulAndAdd(intermediate.org, in_trf[i-1].org, dx_vec, ratio);
      Mat3d_slerp(intermediate.rot, in_trf[i-1].rot, in_trf[i].rot, ratio);
      out.push_back(intermediate);
    }

  }

  out.push_back(in_trf[num_steps-1]);

  if (verbose)
  {
    for (size_t i=0; i<out.size(); ++i)
    {
      double x[6];
      HTr_to6DVector(x, &out[i]);

      std::cout << std::fixed << std::setprecision(6)
                << "out_trf[" << i << "]: " << x[0] << " " << x[1] << " " << x[2] << " "
                //<< x[3] << " " << x[4] << " " << x[5]
                << x[3]*180.0/M_PI << " " << x[4]*180.0/M_PI << " " << x[5]*180.0/M_PI
                << std::defaultfloat << std::endl;

    }
  }


  //////////////////////////////////////////////////////////////////////////
  // Convert to vector of 6d wrenches
  //////////////////////////////////////////////////////////////////////////

  py::array_t<float> out_wrench({static_cast<py::ssize_t>(out.size()-1), static_cast<py::ssize_t>(6)});
  auto out_wrench_u = out_wrench.mutable_unchecked<2>();

  // This loop starts from index 1, since on index 0, we have the current state
  for (size_t i=1; i<out.size(); ++i)
  {
    double wrench_i[6];
    HTr_to6DVector(wrench_i, &out[i]);
    for (size_t j=0; j<6; ++j)
    {
      out_wrench_u(i-1, j) = static_cast<float>(wrench_i[j]);
    }
  }


  return out_wrench;
}


//////////////////////////////////////////////////////////////////////////////
// The python affaction module, mainly consisting off the TeleOp class.
//////////////////////////////////////////////////////////////////////////////
PYBIND11_MODULE(pyTeleOp, m)
{
  //////////////////////////////////////////////////////////////////////////////
  // Constructor
  //////////////////////////////////////////////////////////////////////////////
  auto cls = py::class_<aff::ExampleTeleOp>(m, "TeleOp")
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

    auto ex = std::unique_ptr<aff::ExampleTeleOp>(new aff::ExampleTeleOp());
    //ex->setBuildPath(build_path);
    ex->initParameters();
    return std::move(ex);
  }))


  //////////////////////////////////////////////////////////////////////////////
  // Initialization function, to be called after member variables have been
  // configured.
  //////////////////////////////////////////////////////////////////////////////
  .def("init", [](aff::ExampleTeleOp& ex, bool debug=false) -> bool
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
      std::cerr << "\n* TeleOp initialized\n";
    }
    else
    {
      std::cerr << "\n* Failed to initialize TeleOp\n";
    }
    std::cerr << starLine << "\n";

    return success;
  }, "Initializes algorithm, guis and graphics")

  //////////////////////////////////////////////////////////////////////////////
  // Initialization function, to be called after member variables have been
  // configured.
  //////////////////////////////////////////////////////////////////////////////
  .def("initBlocking", [](aff::ExampleTeleOp& ex, bool headless, bool catch_keyboard_interrupt) -> int
  {
    RFATAL("Use initializeBlocking() instead of initBlocking()");
  })

  .def("initializeBlocking", [](aff::ExampleTeleOp& ex, bool headless) -> int
  {
    // Release the GIL for the function's duration
    pybind11::gil_scoped_release release_gil;

    bool success = ex.initAlgo();

    if (!headless)
    {
      success = ex.initGraphics() && success;
      success = ex.initGuis() && success;
    }

    return success;
  },
  "Initializes algorithm, guis and graphics",
  py::arg("headless") = false)

  //////////////////////////////////////////////////////////////////////////////
  // Initialization function, to be called after member variables have been
  // configured.
  //////////////////////////////////////////////////////////////////////////////
  .def("startBlocking", [](aff::ExampleTeleOp& ex, bool catch_keyboard_interrupt) -> int
  {
    // Release the GIL for the function's duration
    pybind11::gil_scoped_release release_gil;

    int argc = 1;
    char* argv[] = { (char*)"AppName" };
    QApplication app(argc, argv);

    std::setlocale(LC_ALL, "C");
    QApplication::setQuitOnLastWindowClosed(false);

    std::thread t(&aff::ExampleTeleOp::start, &ex);
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
  "Start loop with correct threading",
  py::arg("catch_keyboard_interrupt") = false)

  //////////////////////////////////////////////////////////////////////////////
  // Calls the run method in a new thread, releases the GIL and returns to the
  // python context (e.g. console).
  //////////////////////////////////////////////////////////////////////////////
  .def("run", &aff::ExampleTeleOp::startThreaded, py::call_guard<py::gil_scoped_release>(), "Starts endless loop")

  //////////////////////////////////////////////////////////////////////////////
  // Sets the position of a rigid body, outside the step function.
  //////////////////////////////////////////////////////////////////////////////
  .def("setBodyPosition", [](aff::ExampleTeleOp& ex, std::string bdyName, double x, double y, double z) -> bool
  {
    const RcsBody* bdy = RcsGraph_getBodyByName(ex.getGraph(), bdyName.c_str());
    double* rbj = RcsBody_getStatePtr(ex.getGraph(), bdy);

    if (!rbj)
    {
      return false;
    }

    Vec3d_set(rbj, x, y, z);
    RcsGraph_setState(ex.getGraph(), nullptr, nullptr);
    return true;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Calls an event without arguments. We must not call process() here, since
  // this method might run concurrently to the event queue thread.
  //////////////////////////////////////////////////////////////////////////////
  .def("callEvent", [](aff::ExampleTeleOp& ex, std::string eventName)
  {
    ex.getEntity().publish(eventName);
  })
  .def("process", [](aff::ExampleTeleOp& ex)
  {
    ex.getEntity().process();
  })
  .def("enableRetargetting", [](aff::ExampleTeleOp& ex, bool enable)
  {
    ex.getEntity().publish("EnableRetargetting", enable);
  })
  .def("enableTasks", [](aff::ExampleTeleOp& ex, bool enable)
  {
    ex.getEntity().publish("EnableTasks", enable);
  })
  .def("setLeRobotAction", [](aff::ExampleTeleOp& ex, nlohmann::json command)
  {
    ex.getEntity().publish("SetPolicyCommand", command);
  },
  R"doc(
  Create control commands from LeRobot json commands.


  For bimanual hand pose retargetting:
  ------------------------------------

    command["input_type"] = 2 # IKTeleOp::ActionType::BiManualPose
    command["left_hand_pose"] = [ x, y, z, rm_00, rm_01, rm_02, rm_10, rm_11, rm_12 ]
    command["right_hand_pose"] = [ x, y, z, rm_00, rm_01, rm_02, rm_10, rm_11, rm_12 ]

    command["left_fingers_pose_0"] = "open_fingers_left"
    command["left_fingers_pose_1"] = "close_fingers_left"
    command["s_left_01"] = <value between 0.0 ... 1.0>

    command["right_fingers_pose_0"] = "open_fingers_right"
    command["right_fingers_pose_1"] = "close_fingers_right"
    command["s_right_01"] = <value between 0.0 ... 1.0>


  For skeleton retargetting input (arm direction vectors):
  --------------------------------------------------------

    command["input_type"] = 1 # IKTeleOp::ActionType::PolarRetarget
    command["right_upperarm_ori"] = [ x, y, z ] # right upper arm direction vector
    command["right_forearm_ori"]  = [ x, y, z ] # right forearm direction vector
    command["left_upperarm_ori"]  = [ x, y, z ] # left upper arm direction vector
    command["left_forearm_ori"]   = [ x, y, z ] # left forearm direction vector

    If trained with hand orientation as POLAR angles:
      command["right_hand_ori"]     = [ x, y, z ] # direction vector
      command["left_hand_ori"]      = [ x, y, z ]

    If trained with hand orientation as ABC (Euler) angles:
      command["right_hand_ori"]     = [ rm_00, rm_01, rm_02, rm_10, rm_11, rm_12 ] # truncated rotation matrix
      command["left_hand_ori"]      = [ rm_00, rm_01, rm_02, rm_10, rm_11, rm_12 ]

    Fingers:
      command["left_fingers_pose_0"] = "open_fingers_left"
      command["left_fingers_pose_1"] = "close_fingers_left"
      command["s_left_01"] = <value between 0.0 ... 10.>

      command["right_fingers_pose_0"] = "open_fingers_right"
      command["right_fingers_pose_1"] = "close_fingers_right"
      command["s_right_01"] = <value between 0.0 ... 10.>


  For joint-level position commands:
  ----------------------------------

    We assume a controller xml file with joint-tasks for each joint to be controlled, like:

        <Task name="fr3_joint1_right" controlVariable="Joint" jnt="fr3_joint1_right" />
        <Task name="fr3_joint2_right" controlVariable="Joint" jnt="fr3_joint2_right" />
        <Task name="fr3_joint3_right" controlVariable="Joint" jnt="fr3_joint3_right" />

    The values are expected in SI units. The finger commands can be done
    identically as in the other input types.

    command["input_type"] = 3 # IKTeleOp::ActionType::JointPosition
    command["joints"] = { "fr3_joint1_right": 1.1,
                          "fr3_joint2_right": 2.2,
                          "fr3_joint3_right": 3.3,
                          ... }

  )doc")
  .def("getLeRobotObservation", &aff::ExampleTeleOp::getObservation,
       R"doc(
  joints: "joint_" + <joint_name>
  tasks: "task_" + <task_name> + "_<coordinate index>
  sensors: "sensor_" + <sensor_name> + "_<coordinate index>

  )doc")
  .def("step", &aff::ExampleTeleOp::step)
  .def("stop", &aff::ExampleTeleOp::stop)
  .def("addComponentArgument", &aff::ExampleTeleOp::addComponentArgument)
  .def("cleanup", &aff::ExampleTeleOp::cleanup)

  //////////////////////////////////////////////////////////////////////////////
  // 6 x 1 vector with elements x, y, z, thx, thy, thz (Absolute Euler angles)
  //////////////////////////////////////////////////////////////////////////////
  .def("getBodyPose", &aff::ExampleTeleOp::getBodyPose)

  .def_readwrite("withScene", &aff::ExampleTeleOp::withScene)
  .def_readwrite("xmlFileName", &aff::ExampleTeleOp::xmlFileName)
  .def_readwrite("dataCollectionXmlFileName", &aff::ExampleTeleOp::dataCollectionXmlFileName,
                 "The XML file name for data collection.")
  .def_readwrite("configDirectory", &aff::ExampleTeleOp::configDirectory)
  .def_readwrite("noLimits", &aff::ExampleTeleOp::noLimits)
  .def_readwrite("enableRealGraphVisualization", &aff::ExampleTeleOp::enableRealGraphVisualization)
  .def_readwrite("inputType", &aff::ExampleTeleOp::inputType)
  .def_readwrite("dt", &aff::ExampleTeleOp::dt)

  //////////////////////////////////////////////////////////////////////////////
  // Head gestures: "yes", "no"
  // "StartGesture": std::string gestureName, double gestureAmplitude, int numTurns
  // "GestureThreeRepetitions": std::string gestureName, double gestureAmplitude
  //////////////////////////////////////////////////////////////////////////////
  .def("setHeadGesture", [](aff::ExampleTeleOp& ex, std::string gestureName, double gestureAmplitude, int numTurns)
  {
    ex.getEntity().publish("StartGesture", gestureName, gestureAmplitude, numTurns);
  },
  py::arg("gestureName"),
  py::arg("gestureAmplitude") = RCS_DEG2RAD(5.0),
  py::arg("numTurns") = 3)

  //////////////////////////////////////////////////////////////////////////////
  // Gaze model methods: 0: Neck only, 1: pupils only.
  //////////////////////////////////////////////////////////////////////////////
  .def("setPupilSpeedWeight", [](aff::ExampleTeleOp& ex, double value)
  {
    ex.getEntity().publish("SetPupilWeight", value);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns empty json if there are no agents or a json in the form:
  // {"agents": ['Daniel', 'Felix', 'Robot'] }
  //////////////////////////////////////////////////////////////////////////////
  .def("get_agents", [](aff::ExampleTeleOp& ex, bool onlyVisibleAgents) -> nlohmann::json
  {
    nlohmann::json json;
    json["agents"] = std::vector<nlohmann::json>();

    for (const auto& a : ex.getScene()->agents)
    {
      if ((onlyVisibleAgents && a->isVisible()) || (!onlyVisibleAgents))
      {
        json["agents"].push_back(a->name);
      }
    }

    return json;
  },
  py::arg("onlyVisibleAgents") = false)

  //////////////////////////////////////////////////////////////////////////////
  // Returns empty json if there are no human agents or a json in the form:
  // {"agents": ['Daniel', 'Felix', 'Robot'] }
  //////////////////////////////////////////////////////////////////////////////
  .def("get_human_agents", [](aff::ExampleTeleOp& ex, bool onlyVisibleAgents) -> nlohmann::json
  {
    nlohmann::json json;
    json["agents"] = std::vector<nlohmann::json>();

    for (const auto& a : ex.getScene()->agents)
    {
      if ((onlyVisibleAgents && a->isVisible()) || (!onlyVisibleAgents))
      {
        if (dynamic_cast<aff::HumanAgent*>(a))
        {
          json["agents"].push_back(a->name);
        }
      }
    }

    return json;
  },
  py::arg("onlyVisibleAgents") = false)

  //////////////////////////////////////////////////////////////////////////////
  // Which body a gaze should aim at, for one human agent, asked by ROLE rather
  // than by name:
  //
  //   get_agent_tracked_frame("", "Head")
  //     -> {"agent": "conversation_partner",
  //         "frame": "Head_conversation_partner"}
  //
  // An empty agent_name means "the only human in the scene". With several
  // humans and no name it returns {"error": ...} instead of choosing: staring
  // confidently at the wrong person is worse than not moving.
  //
  // body_type is a TrackedFrames id from the scene XML -- "Head", "HandLeft",
  // "HandRight", "ShoulderLeft", "ElbowRight" and so on. The frames already
  // carry the agent's group suffix, so a caller never builds a body name by
  // gluing strings, which is what let home_ai's gaze rules stop hardcoding
  // `conversation_partner` and `Hand_<side>_conversation_partner`.
  //////////////////////////////////////////////////////////////////////////////
  .def("get_agent_tracked_frame", [](aff::ExampleTeleOp& ex,
                                     std::string agent_name,
                                     std::string body_type) -> nlohmann::json
  {
    std::vector<const aff::HumanAgent*> humans;
    for (const auto& a : ex.getScene()->agents)
    {
      const aff::HumanAgent* human = dynamic_cast<aff::HumanAgent*>(a);
      if (human && (agent_name.empty() || human->name == agent_name))
      {
        humans.push_back(human);
      }
    }

    if (humans.empty())
    {
      return nlohmann::json{{"error", agent_name.empty()
          ? "the scene has no human agents"
          : "no human agent named '" + agent_name + "'"
        }};
    }

    // Several humans and no name: refuse. Picking the first would look at a
    // confidently chosen wrong person, which is worse than not moving at all.
    if (humans.size() > 1)
    {
      std::string names;
      for (const auto* h : humans)
      {
        names += (names.empty() ? "" : ", ") + h->name;
      }
      return nlohmann::json{{"error", "the scene has several human agents (" +
          names + "); name the one to look at"
        }};
    }

    const aff::HumanAgent* human = humans.front();
    // Asked through the agent's own accessor, so the body-type vocabulary stays in
    // one place and the returned name already carries the group suffix.
    const std::string frame = human->getTrackedFrameName(body_type);
    if (frame.empty())
    {
      return nlohmann::json{{"error", "agent '" + human->name +
          "' has no tracked frame '" + body_type + "'"
        }};
    }

    return nlohmann::json{{"agent", human->name}, {"frame", frame}};
  },
  py::arg("agent_name") = "", py::arg("body_type") = "Head")

  //////////////////////////////////////////////////////////////////////////////
  // Sets the gaze target body for the eye gaze model. This only takes effect
  // if the class has been initialized with the eyeIkEnabled flag set to true.
  //////////////////////////////////////////////////////////////////////////////
  .def("setGazeTarget", [](aff::ExampleTeleOp& ex, std::string targetBody) -> std::string
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
  //
  //////////////////////////////////////////////////////////////////////////////
  .def("addVirtualCamera",
       py::overload_cast<std::string, std::string, int, int>(&aff::ExampleTeleOp::addVirtualCamera),
       py::arg("camera_name"),
       py::arg("camera_type"),
       py::arg("width"),
       py::arg("height"))

  //////////////////////////////////////////////////////////////////////////////
  // Returns a rendered image from the given coordinates
  //////////////////////////////////////////////////////////////////////////////
  .def("captureColorImageFromFrame", [](aff::ExampleTeleOp& ex, std::string cameraName)
       -> py::array_t<uint8_t>
  {
    const RcsBody* cam = RcsGraph_getBodyByName(ex.getGraph(), cameraName.c_str());
    if (!cam)
    {
      RLOG_CPP(1, "Camera body " << cameraName << " not found - returning empty array");
      return py::array_t<uint8_t>({ 0, 0, 3 });
    }

    auto virtualCameras = ex.getVirtualCameras();
    if (virtualCameras.empty())
    {
      RLOG_CPP(1, "No virtual cameras found - returning empty array");
      return py::array_t<uint8_t>({ 0, 0, 3 });
    }


    for (size_t i=0; i<virtualCameras.size(); ++i)
    {
      if (cameraName != virtualCameras[i].first)
      {
        continue;
      }

      aff::VirtualCamera* vcam = virtualCameras[i].second;

      if (!vcam)
      {
        RLOG_CPP(1, "Found NULL virtual camera on index " << i << " - skipping");
        continue;
      }

      int width = (int)vcam->getWidth();
      int height = (int)vcam->getHeight();

      vcam->capture(&cam->A_BI);

      // Here width and height need to be reversed
      py::array_t<uint8_t> colorImageUint8({height, width, 3});
      vcam->getColorImage(colorImageUint8.mutable_data(), colorImageUint8.size());
      return colorImageUint8;
    }

    RLOG_CPP(1, "Camera " << cameraName << " not found - returning empty array");
    return py::array_t<uint8_t>({ 0, 0, 3 });
  }, R"pbdoc(
  Renders the desired state of the scene from the given camera. Outputs the color image.
  If there is no virtual camera instantiated in the simulator, this will be done in this
  function. This leads to the first call being a bit more slow than the consecutive ones,
  since the camera construction takes 1-2 secs.
  Camera frame convention: x points forward, z point upward, and y points left

  Example
  -------
  import cv2
  import numpy as np

  color = sim.captureColorImageFromFrame("camera_01")
  color_np = np.array(color)
  color_bgr = cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR)
  cv2.imwrite("color_image.jpg", color_bgr)
  )pbdoc")

  ;



  //////////////////////////////////////////////////////////////////////////////
  // Rcs function wrappers
  //////////////////////////////////////////////////////////////////////////////

  // Sets the rcs log level
  m.def("setLogLevel", [](int level)
  {
    RcsLogLevel = level;
  });

  // Sets the rcs log level
  m.def("getLogLevel", []()
  {
    return RcsLogLevel;
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

  m.def("policy_to_command",
        [](py::array_t<float, py::array::c_style> in,
           std::vector<double> wrench,
           py::ssize_t N,
           double dx_max,
           double dphi_max,
           bool verbose) -> py::array_t<float>
  {
    return convert_policy(in, N, wrench, dx_max, dphi_max, verbose);
  },
  py::arg("in"),
  py::arg("wrench"),
  py::arg("N") = 8,
  py::arg("dx_max") = std::numeric_limits<double>::max(),
  py::arg("dphi_max") = std::numeric_limits<double>::max(),
  py::arg("verbose") = false),
  R"doc(
policy_to_command(in, N=8) -> numpy.ndarray

Convert a fixed-size policy output matrix into a command matrix.

Parameters
----------
in : numpy.ndarray, shape (16, 9), dtype float32
     Input policy matrix. Each row encodes a pose in the following layout:

    - in[r, 0:3]   : position (x, y, z)
    - in[r, 3:6]   : first rotation basis vector (R row 0 or axis 0)
    - in[r, 6:9]   : second rotation basis vector (R row 1 or axis 1)

    The third basis vector is reconstructed as a right-handed vector via
    cross product: row2 = cross(row0, row1). The resulting 3x3 rotation
    matrix is validated and converted to Euler angles. It is assumed that
    the first and second given row are orthonormal.

wrench: vector<double> of size 6 x 1, holding position (0-2) and Euler
        angles (index 3-5).

N : int, optional (default: 8)
    Number of output rows to produce. Output row i uses input row (i % 16).

Returns
-------
out : numpy.ndarray, shape (N, 6), dtype float32
    Command matrix with per-row layout:

    - out[i, 0:2] :  position (x, y, z) copied from input
    - out[i, 3:5] :  Euler angles (a0, a1, a2) derived from the reconstructed
                     rotation matrix

Raises
------
RuntimeError
    If `in` is not a 2D array with shape (16, 9), if dtype is not float32,
    and many other picky issues.

Notes
-----
- The binding requires a C-contiguous (row-major) array. Non-contiguous views
  (e.g., transposes) will be rejected.
- Euler angle convention is defined by the underlying `Mat3d_toEulerAngles`
  implementation.

Examples
--------
>>> import numpy as np
>>> inp = np.zeros((16, 9), dtype=np.float32)
>>> wrench = np.zeros(6, dtype=np.float64)
>>> out = my_module.policy_to_command(inp, wrench, N=8)
>>> out.shape
(8, 12)
)doc";















}
