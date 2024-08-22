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

#include "CameraViewComponent.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>

#include <iomanip>



namespace aff
{

/*******************************************************************************
 * Constructs and subscribes to all events. The ROS subscribers are initialized
 * from the Start event.
 ******************************************************************************/
CameraViewComponent::CameraViewComponent(EntityBase* parent,
                                         const std::string& cameraBodyName,
                                         bool currentGraph) :
  ComponentBase(parent), camBdyName(cameraBodyName), viewFromKinect(false),
  toggleViewFromKinect(false), useCurrentGraph(currentGraph),
  fovx(71.0), fovy(60.0)
{
  subscribe("PostUpdateGraph", &CameraViewComponent::onPostUpdateGraph);
  subscribe("SetViewFromKinect", &CameraViewComponent::onToggleViewFromKinect);
  subscribe("SetFieldOfView", &CameraViewComponent::onSetFieldOfView);
}

/*******************************************************************************
 * Handle the change of the viewer's perspective between mouse manipulator
 * and fixed to the Kinect's transform. In case the view is set to the
 * Kinect, we update the camera transform.
 ******************************************************************************/
void CameraViewComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  // Handle the change of the viewer's perspective between mouse manipulator
  // and fixed to the Kinect's transform. In case the view is set to the
  // Kinect, we update the camera transform.
  RcsGraph* graph = useCurrentGraph ? current : desired;

  const RcsBody* cam = RcsGraph_getBodyByName(graph, camBdyName.c_str());
  if (cam)
  {
    handleViewChange(&cam->A_BI);
  }
  else
  {
    RLOG_CPP(4, "Camera body " << camBdyName << " not found in graph");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void CameraViewComponent::onToggleViewFromKinect()
{
  viewFromKinect = !viewFromKinect;
  toggleViewFromKinect = true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void CameraViewComponent::onSetFieldOfView(double fov_x, double fov_y)
{
  this->fovx = fov_x;
  this->fovy = fov_y;
}

/*******************************************************************************
 * Change viewer to camera perspective
 * FoV of Kinect2 ist approximately:
 * RGB: 84 x 54 deg (1920 x 1080)
 * IR/D: 71 x 60deg  (512 x 424)
 ******************************************************************************/
void CameraViewComponent::handleViewChange(const HTr* A_camI)
{
  if (viewFromKinect)
  {
    HTr camTrf;
    HTr_copy(&camTrf, A_camI);
    char trfStr[256];
    HTr_toString(trfStr, &camTrf);
    getEntity()->publish("RenderCommand", std::string("CameraTransform"),
                         std::string(trfStr));

    if (toggleViewFromKinect)
    {
      toggleViewFromKinect = false;
      std::string fov = std::to_string(this->fovx) + " " + std::to_string(this->fovy);
      getEntity()->publish("RenderCommand", std::string("FieldOfView"), fov);
    }

  }
  else
  {
    if (toggleViewFromKinect)
    {
      toggleViewFromKinect = false;
      getEntity()->publish("RenderCommand", std::string("ResetView"),
                           std::string("30 40"));
    }

  }

}

}   // namespace aff
