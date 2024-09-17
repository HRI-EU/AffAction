/*******************************************************************************

  Copyright (c) by Honda Research Institute Europe GmbH

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

#include "VirtualCameraWindow.h"

#include <PPSGui.h>

namespace aff
{

VirtualCameraWindow::VirtualCameraWindow(EntityBase* parent,
                    VirtualCamera* _virtualCamera,
                    bool color, bool depth, const HTr* A_CamI) :
  ComponentBase(parent), virtualCamera(_virtualCamera), 
  colorBuffer(color ? _virtualCamera->width * _virtualCamera->height * 3 : 0),
  depthBuffer(depth ? _virtualCamera->width * _virtualCamera->height : 0)
{
  if (A_CamI)
  {
    setCameraTransform(A_CamI);
  }
  else
  {
    HTr_setIdentity(&cameraTransform);
  }

  subscribe("Render", &VirtualCameraWindow::update);
  subscribe("ToggleVirtualRenderGui", &VirtualCameraWindow::toggle);
}
VirtualCameraWindow::~VirtualCameraWindow() {}

void VirtualCameraWindow::setCameraTransform(double x, double y, double z,
                                             double thx, double thy, double thz)
{
  double transform6d[] = {x, y, z, thx, thy, thz};
  HTr_from6DVector(&cameraTransform, transform6d);
}


void VirtualCameraWindow::setCameraTransform(const HTr* A_CamI)
{
  HTr_copy(&cameraTransform, A_CamI);
}

void VirtualCameraWindow::update()
{
  virtualCamera->render(&cameraTransform, colorBuffer.empty() ? nullptr : colorBuffer.data(), 
                        depthBuffer.empty() ? nullptr : depthBuffer.data());
}

void VirtualCameraWindow::setEnabled(bool enabled)
{
  if (enabled)
  {
    enable();
  }
  else
  {
    disable();
  }
}

bool VirtualCameraWindow::isEnabled()
{
  return pixelGui && pixelGui->getWidget();
}

void VirtualCameraWindow::toggle()
{
  setEnabled(!isEnabled());
}

void VirtualCameraWindow::enable()
{
  if (pixelGui)
  {
    return;
  }

  std::vector<Rcs::PPSGui::Entry> pps;

  if (!depthBuffer.empty())
  {
    pps.push_back(Rcs::PPSGui::Entry("Depth image", virtualCamera->width, virtualCamera->height, depthBuffer.data(), 1, 0.1));
  }

  if (!colorBuffer.empty())
  {
    pps.push_back(Rcs::PPSGui::Entry("Color image", virtualCamera->width, virtualCamera->height, colorBuffer.data(), 3, 1.0));
  }

  pixelGui = std::make_unique<Rcs::PixelGui>(pps);
}

void VirtualCameraWindow::disable()
{
  pixelGui.reset();
}

} // aff