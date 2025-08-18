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

void bind_exploration(py::class_<aff::ExampleActionsECS>& cls)
{
  //////////////////////////////////////////////////////////////////////////////
  //
  //////////////////////////////////////////////////////////////////////////////
  cls.def("recognize_faces", [](aff::ExampleActionsECS& ex, int n_iterations, double timeout_in_seconds) -> std::string
  {
    py::gil_scoped_release release;  // Unblock waiting period
    return aff::recognize_faces(ex.getEntity(), std::string(), n_iterations, timeout_in_seconds);
  },
  py::arg("n_iterations") = 5,
  py::arg("timeout_in_seconds") = 2.5)

  //////////////////////////////////////////////////////////////////////////////
  //
  //////////////////////////////////////////////////////////////////////////////
  .def("recognize_agent_face", [](aff::ExampleActionsECS& ex, std::string agentName, int n_iterations, double timeout_in_seconds) -> std::pair<std::string, std::string>
  {
    py::gil_scoped_release release;  // Unblock waiting period
    return aff::recognize_agent_face(ex.getEntity(), ex.getScene(), agentName, n_iterations, timeout_in_seconds);
  },
  py::arg("agentName") = std::string(),
  py::arg("n_iterations") = 5,
  py::arg("timeout_in_seconds") = 2.5)

  //////////////////////////////////////////////////////////////////////////////
  //
  //////////////////////////////////////////////////////////////////////////////
  .def("track_facemesh", [](aff::ExampleActionsECS& ex, int n_iterations, double timeout_in_seconds) -> bool
  {
    py::gil_scoped_release release;  // Unblock waiting period
    return aff::track_facemesh(ex.getEntity(), std::string(), n_iterations, timeout_in_seconds);
  },
  py::arg("n_iterations") = 5,
  py::arg("timeout_in_seconds") = 2.5)

  //////////////////////////////////////////////////////////////////////////////
  //
  //////////////////////////////////////////////////////////////////////////////
  .def("track_agent_facemesh", [](aff::ExampleActionsECS& ex, std::string agentName, int n_iterations, double timeout_in_seconds) -> bool
  {
    py::gil_scoped_release release;  // Unblock waiting period
    return aff::track_agent_facemesh(ex.getEntity(), ex.getScene(), agentName, n_iterations, timeout_in_seconds);
  },
  py::arg("agentName") = std::string(),
  py::arg("n_iterations") = 5,
  py::arg("timeout_in_seconds") = 2.5)

  ;

}
