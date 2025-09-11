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

#include <GraphNode.h>


void bind_virtual_camera(py::class_<aff::ExampleActionsECS>& cls)
{
  //////////////////////////////////////////////////////////////////////////////
  // Returns a rendered image from the given coordinates
  // a = sim.captureImage(0, 0, 1, 0, 0, 0)
  //////////////////////////////////////////////////////////////////////////////
  cls.def("captureImage", [](aff::ExampleActionsECS& ex, int cam_idx,
                             double x, double y, double z,
                             double thx, double thy, double thz)
          -> std::tuple<py::array_t<uint8_t>, py::array_t<float>>
  {
    aff::VirtualCamera* virtualCamera = ex.getVirtualCamera(cam_idx);

    if (!virtualCamera)
    {
      RLOG(1, "No virtual camera found on index %d - returning empty images", cam_idx);
      return std::tuple<py::array_t<uint8_t>, py::array_t<float>>();
    }

    HTr A_camI;
    double x6[6];
    VecNd_set6(x6, x, y, z, thx, thy, thz);
    HTr_from6DVector(&A_camI, x6);
    virtualCamera->capture(&A_camI);

    py::array_t<uint8_t> colorImageUint8({ (int)virtualCamera->getHeight(), (int)virtualCamera->getWidth(), 3 });
    virtualCamera->getColorImage(colorImageUint8.mutable_data(), colorImageUint8.size());

    py::array_t<float> depthImageFloat({ (int)virtualCamera->getHeight(), (int)virtualCamera->getWidth(), 1 });
    virtualCamera->getDepthImage(depthImageFloat.mutable_data(), depthImageFloat.size());

    return std::make_tuple(colorImageUint8, depthImageFloat);

  },
  R"pbdoc(
Renders the desired state of the scene. The input is the camera origin and yrp rotation
around that origin. Outputs the color and depth image. If there is no virtual camera
instantiated in the simulator, this will be done in this function. This leads to the
first call being a bit more slow than the consecutive ones, since the camera construction
takes 1-2 secs.
Camera frame convention: x points forward, z point upward, and y points left

Example
-------
import cv2
import numpy as np

color, depth = sim.captureImage(-0.77, 0.0, 1.66, 0.0, 1.0, 0.0)
color_np = np.array(color)
color_bgr = cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR)
cv2.imwrite("color_image.jpg", color_bgr)

depth_np = np.array(depth)
depth_normalized = cv2.normalize(depth_np, None, 0, 255, cv2.NORM_MINMAX)
depth_display = depth_normalized.astype(np.uint8)
cv2.imwrite("depth_image.jpg", depth_display)
)pbdoc")

  //////////////////////////////////////////////////////////////////////////////
  // Adds a text label to an object
  //////////////////////////////////////////////////////////////////////////////
  .def("addTextLabelToBody", [](aff::ExampleActionsECS& ex, std::string cameraName, std::string bodyName, std::string text) -> bool
  {
    auto virtualCameras = ex.getVirtualCameras();
    if (virtualCameras.empty())
    {
      RLOG_CPP(1, "No virtual cameras found - returning empty array");
      return false;
    }

    bool success = true;
    int num_calls = 0;

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

        osg::Group* root = dynamic_cast<osg::Group*>(vcam->getRenderer()->getSceneData());


        auto gnVec = Rcs::findChildrenOfType<Rcs::GraphNode>(root);

        for (const auto& gn : gnVec)
        {
          success = gn->addTextLabel(bodyName, text) && success;
          num_calls++;
        }

      }

    if (num_calls==0)
    {
      RLOG_CPP(1, "No text labes could be assigned");
      success = false;
    }

    return success;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns a rendered image from the given coordinates
  //////////////////////////////////////////////////////////////////////////////
  .def("captureColorImageFromFrame", [](aff::ExampleActionsECS& ex, std::string cameraName)
       -> py::array_t<uint8_t>
  {
    const RcsBody* cam = RcsGraph_getBodyByName(ex.getGraph(), cameraName.c_str());
    if (!cam)
    {
      RLOG_CPP(1, "Camera body " << cameraName << " not found - returning empty array");
      return py::array_t<double>({ 0, 0, 3 });
    }

    auto virtualCameras = ex.getVirtualCameras();
    if (virtualCameras.empty())
    {
      RLOG_CPP(1, "No virtual cameras found - returning empty array");
      return py::array_t<double>({ 0, 0, 3 });
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
    return py::array_t<double>({ 0, 0, 3 });
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

  //////////////////////////////////////////////////////////////////////////////
  //
  //////////////////////////////////////////////////////////////////////////////
  .def("addVirtualCamera",
       py::overload_cast<std::string, std::string, int, int>(&aff::ExampleActionsECS::addVirtualCamera),
       py::arg("camera_name"), py::arg("camera_type"), py::arg("width"), py::arg("height"))


    ;

}
