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

#ifndef AFF_VIRTUALCAMERAWINDOW_H
#define AFF_VIRTUALCAMERAWINDOW_H

#include "ComponentBase.h"
#include "VirtualCamera.h"

#include <DepthRenderer.h>
#include <AsyncWidget.h>

#include <mutex>
#include <memory>

namespace aff
{

class VirtualCameraWindow : public ComponentBase
{
public:

  VirtualCameraWindow(EntityBase* base,
                      VirtualCamera* virtualCamera,
                      bool color = true,
                      bool depth = false,
                      const HTr* A_camI = nullptr);
  virtual ~VirtualCameraWindow();
  
  void setCameraTransform(double x, double y, double z, double thx, double thy, double thz);
  void setCameraTransform(const HTr* A_camI);
  
  virtual void update();
  void setEnabled(bool);
  bool isEnabled();

protected:
  void enable();
  void disable();
  void toggle();

  std::unique_ptr<Rcs::AsyncWidget> pixelGui;

  std::vector<double> colorBuffer;
  std::vector<double> depthBuffer;

  VirtualCamera* const virtualCamera;
  HTr cameraTransform;
  
private:
  VirtualCameraWindow(const VirtualCameraWindow&) = delete;
  VirtualCameraWindow& operator=(const VirtualCameraWindow&) = delete;
};

} // aff

#endif // AFF_VIRTUALCAMERA_H