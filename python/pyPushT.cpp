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
#include <pybind11/numpy.h>
#include "pybind11_json.hpp"
#include "pybind_dict_utils.h"
#include "type_casters.h"

namespace py = pybind11;

#include <ExampleFlowMatching.h>
#include <SegFaultHandler.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>

#if !defined(_MSC_VER)
#include <X11/Xlib.h>
#endif

#include <iostream>

RCS_INSTALL_ERRORHANDLERS



static py::array_t<double> getColorImage(aff::ExampleFlowMatching& ex)
{
  ex.vcamC->update();

  // Get the color buffer and dimensions
  std::vector<double> buffer = ex.vcamC->getColorBuffer();
  int width = ex.vcamC->getWidth();
  int height = ex.vcamC->getHeight();

  // Ensure the buffer size matches the dimensions
  if (buffer.size() != width* height * 3)
  {
    throw std::runtime_error("Buffer size does not match image dimensions.");
  }

  // Create a NumPy array with shape (height, width, 3)
  return py::array_t<double>({height, width, 3}, buffer.data());
}





//////////////////////////////////////////////////////////////////////////////
// The python affaction module, mainly consisting off the LlmSim class.
//////////////////////////////////////////////////////////////////////////////
PYBIND11_MODULE(pyPushT, m)
{

  //////////////////////////////////////////////////////////////////////////////
  // Constructor
  //////////////////////////////////////////////////////////////////////////////
  py::class_<aff::ExampleFlowMatching>(m, "PushT")
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

    auto ex = std::unique_ptr<aff::ExampleFlowMatching>(new aff::ExampleFlowMatching());
    ex->initParameters();
    return std::move(ex);
  }))

  //////////////////////////////////////////////////////////////////////////////
  // Initialization function, to be called after member variables have been
  // configured.
  //////////////////////////////////////////////////////////////////////////////
  .def("init", [](aff::ExampleFlowMatching& ex, bool withGraphics=true) -> bool
  {
    bool success = ex.initAlgo();

    if (withGraphics)
    {
      success = ex.initGraphics() && success;
      ex.entity.publish("Render");
      ex.entity.process();
    }

    std::string starLine(80, '*');
    std::cerr << "\n\n" + starLine;
    if (success)
    {
      std::cerr << "\n* PushT initialized\n";
    }
    else
    {
      std::cerr << "\n* Failed to initialize PushT\n";
    }
    std::cerr << starLine << "\n";

    return success;
  }, "Initializes algorithm, guis and graphics")

  //////////////////////////////////////////////////////////////////////////////
  // New random pose. Needs to return image.
  //////////////////////////////////////////////////////////////////////////////
  .def("reset", [](aff::ExampleFlowMatching& ex) -> py::array_t<double>
  {
    ex.randomize(ex.rndGraph);
    ex.entity.call<const RcsGraph*>("InitFromState", ex.rndGraph);
    ex.entity.call("UpdateGraph", ex.graphC->getGraph());
    ex.entity.call("ComputeKinematics", ex.graphC->getGraph());
    ex.entity.call("Render");
    ex.entity.process();
    ex.vcamC->update();

    // Get the color buffer and dimensions
    return getColorImage(ex);
  }, "New random pose")

  //////////////////////////////////////////////////////////////////////////////
  // Returns the image
  //////////////////////////////////////////////////////////////////////////////
  .def("get_observation", [](aff::ExampleFlowMatching& ex) -> py::array_t<double>
  {
    return getColorImage(ex);
  }, "Return observation")

  //////////////////////////////////////////////////////////////////////////////
  // Returns the x-y position of the push block
  //////////////////////////////////////////////////////////////////////////////
  .def("get_control", [](aff::ExampleFlowMatching& ex) -> py::array_t<double>
  {
    py::array_t<double> result(2);

    const RcsBody* block = RcsGraph_getBodyByName(ex.graphC->getGraph(), "block");
    if (!block)
    {
      throw std::runtime_error("Could not find body with the name 'block' - but it is required.");
    }

    auto buf = result.mutable_unchecked<1>();// 1d indexing
    buf(0) = block->A_BI.org[0];
    buf(1) = block->A_BI.org[1];
    return result;
  }, "Return the x-y position of the push block")

  //////////////////////////////////////////////////////////////////////////////
  // Step one control command
  //////////////////////////////////////////////////////////////////////////////
  .def("step", [](aff::ExampleFlowMatching& ex, py::array_t<double> vel_des) -> py::array_t<double>
  {
    const RcsBody* block = RcsGraph_getBodyByName(ex.graphC->getGraph(), "block");
    if (!block)
    {
      throw std::runtime_error("Could not find body with the name 'block' - but it is required.");
    }

    HTr A_BI = block->A_BI;
    A_BI.org[0] += vel_des.data()[0];
    A_BI.org[1] += vel_des.data()[1];
    ex.physicsC->getPhysics()->applyTransform(block, &A_BI);

    ex.entity.call("UpdateGraph", ex.graphC->getGraph());
    ex.entity.call("ComputeKinematics", ex.graphC->getGraph());
    ex.entity.call("Render");
    ex.entity.process();
    ex.entity.stepTime();

    // Get camera image
    return getColorImage(ex);
  }, "Return observation")
  ;

}
