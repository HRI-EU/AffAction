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

#include <pybind11/stl.h>
#include <pybind11/numpy.h>

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

  if (num_steps > trajectory.shape(0))   // usually 16 or 32
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
  auto cls = py::class_<aff::ExampleTeleOpFrankaRight>(m, "TeleOp")
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

    auto ex = std::unique_ptr<aff::ExampleTeleOpFrankaRight>(new aff::ExampleTeleOpFrankaRight());
    //ex->setBuildPath(build_path);
    ex->initParameters();
    return std::move(ex);
  }))


  //////////////////////////////////////////////////////////////////////////////
  // Initialization function, to be called after member variables have been
  // configured.
  //////////////////////////////////////////////////////////////////////////////
  .def("init", [](aff::ExampleTeleOpFrankaRight& ex, bool debug=false) -> bool
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
  .def("initBlocking", [](aff::ExampleTeleOpFrankaRight& ex, bool headless, bool catch_keyboard_interrupt) -> int
  {
    // Release the GIL for the function's duration
    pybind11::gil_scoped_release release_gil;

    //ex.blockingMainThread = true;
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

    Timer_waitDT(0.5);
    std::thread t(&aff::ExampleTeleOpFrankaRight::start, &ex);
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
  // Calls the run method in a new thread, releases the GIL and returns to the
  // python context (e.g. console).
  //////////////////////////////////////////////////////////////////////////////
  .def("run", &aff::ExampleTeleOpFrankaRight::startThreaded, py::call_guard<py::gil_scoped_release>(), "Starts endless loop")

  //////////////////////////////////////////////////////////////////////////////
  // Calls an event without arguments. We must not call process() here, since
  // this method might run concurrently to the event queue thread.
  //////////////////////////////////////////////////////////////////////////////
  .def("callEvent", [](aff::ExampleTeleOpFrankaRight& ex, std::string eventName)
  {
    ex.getEntity().publish(eventName);
  })
  .def("process", [](aff::ExampleTeleOpFrankaRight& ex)
  {
    ex.getEntity().process();
  })
  .def("enableRetargetting", [](aff::ExampleTeleOpFrankaRight& ex, bool enable)
  {
    ex.getEntity().publish("EnableRetargetting", enable);
  })
  .def("enableTasks", [](aff::ExampleTeleOpFrankaRight& ex, bool enable)
  {
    ex.getEntity().publish("EnableTasks", enable);
  })
  .def("setTwist", [](aff::ExampleTeleOpFrankaRight& ex, double vel_x, double vel_y, double vel_z,
                      double vel_thx, double vel_thy, double vel_thz, bool inWorldFrame)
  {
    ex.getEntity().publish("SetTwist", vel_x, vel_y, vel_z, vel_thx, vel_thy, vel_thz, inWorldFrame);
  },
  "Set twist command for IK",
  py::arg("vel_x"),
  py::arg("vel_y"),
  py::arg("vel_z"),
  py::arg("vel_thx"),
  py::arg("vel_thy"),
  py::arg("vel_thz"),
  py::arg("inWorldFrame") = true)
  .def("setWrench", [](aff::ExampleTeleOpFrankaRight& ex, double pos_x, double pos_y, double pos_z,
                       double eul_thx, double eul_thy, double eul_thz, bool inWorldFrame)
  {
    ex.getEntity().publish("SetWrench", pos_x, pos_y, pos_z, eul_thx, eul_thy, eul_thz, inWorldFrame);
  },
  "Set wrench command for IK",
  py::arg("pos_x"),
  py::arg("pos_y"),
  py::arg("pos_z"),
  py::arg("eul_thx"),
  py::arg("eul_thy"),
  py::arg("eul_thz"),
  py::arg("inWorldFrame") = true)
  .def("setFingerPose", [](aff::ExampleTeleOpFrankaRight& ex, std::string fingerPose)
  {
    ex.getEntity().publish("SetFingerPose", fingerPose);
  })

  .def("step", &aff::ExampleTeleOpFrankaRight::step)
  .def("stop", &aff::ExampleTeleOpFrankaRight::stop)
  .def("addComponentArgument", &aff::ExampleTeleOpFrankaRight::addComponentArgument)

  //////////////////////////////////////////////////////////////////////////////
  // Collected data:
  // pos_in_world (3 x 1)
  // rotmat_world_to_endeffector (9 x 1, row-major)
  // twist_in_world (6 x 1)
  // twist_in_endeffector (6 x 1)
  // q_current (dof x 1, for elements please check with printCollectedData())
  // q_desired (joint command, same shape as q_current)
  // wrench in base coordinates (6 x 1)
  // wrench in endeffector coordinates (6 x 1)
  //////////////////////////////////////////////////////////////////////////////
  .def("printCollectedData", &aff::ExampleTeleOpFrankaRight::printCollectedData)
  .def("getCollectedData", &aff::ExampleTeleOpFrankaRight::getCollectedData)
  .def("cleanup", &aff::ExampleTeleOpFrankaRight::cleanup)
  .def("getEndEffectorWrench", &aff::ExampleTeleOpFrankaRight::getEndEffectorWrench)
  .def_readwrite("withScene", &aff::ExampleTeleOpFrankaRight::withScene)
  .def_readwrite("xmlFileName", &aff::ExampleTeleOpFrankaRight::xmlFileName)
  .def_readwrite("configDirectory", &aff::ExampleTeleOpFrankaRight::configDirectory)
  .def_readwrite("noLimits", &aff::ExampleTeleOpFrankaRight::noLimits)
  .def_readwrite("enableRealGraphVisualization", &aff::ExampleTeleOpFrankaRight::enableRealGraphVisualization)
  .def_readwrite("inputType", &aff::ExampleTeleOpFrankaRight::inputType)
  .def_readwrite("dt", &aff::ExampleTeleOpFrankaRight::dt)

  //////////////////////////////////////////////////////////////////////////////
  //
  //////////////////////////////////////////////////////////////////////////////
  .def("addVirtualCamera",
       py::overload_cast<std::string, std::string, int, int>(&aff::ExampleTeleOpFrankaRight::addVirtualCamera),
       py::arg("camera_name"),
       py::arg("camera_type"),
       py::arg("width"),
       py::arg("height"))

  //////////////////////////////////////////////////////////////////////////////
  // Returns a rendered image from the given coordinates
  //////////////////////////////////////////////////////////////////////////////
  .def("captureColorImageFromFrame", [](aff::ExampleTeleOpFrankaRight& ex, std::string cameraName)
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
