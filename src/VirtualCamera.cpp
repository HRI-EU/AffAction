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

#include "VirtualCamera.h"

#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_math.h>

#include <mutex>



namespace aff
{

VirtualCamera::VirtualCamera(osg::Node* node, int width_, int height_) :
  width(width_), height(height_), virtualRenderer(width_, height_)
{
  // These come from a Kinect v2 calbration
  double fx = 1.36972287105 * height;
  double cx = width / 2 - 0.5;
  double fy = 1.36972287105 * height;
  double cy = height / 2 - 0.5;
  double near = 0.3;
  double far = 10.0;
  
  virtualRenderer.setProjectionFromFocalParams(fx, fy, cx, cy, near, far);
  virtualRenderer.setSceneData(node);
}

VirtualCamera::~VirtualCamera() {}

void VirtualCamera::render(double x, double y, double z, 
                           double thx, double thy, double thz,
                           double* colorBuffer, double* depthBuffer)
{
  double transform6d[] = {x, y, z, thx, thy, thz};

  HTr A_camI;
  HTr_from6DVector(&A_camI, transform6d);

  render(&A_camI, colorBuffer, depthBuffer);
}

void VirtualCamera::render(const HTr* A_camI, double* colorBuffer, double* depthBuffer)
{
  if (!colorBuffer && !depthBuffer)
  {
    RLOG(1, "No buffers to render to");
    return;
  }

  const std::lock_guard<std::mutex> lockGuard(virtualRendererLock);

  double t_render = Timer_getSystemTime();  
  virtualRenderer.setCameraTransform(A_camI);
  virtualRenderer.frame();

  if (colorBuffer)
  {
    const auto& colorImage = virtualRenderer.getRGBImageRef();
    for (size_t i = 0; i < height; i++)
    {
      for (size_t j = 0; j < width; j++)
      {
        colorBuffer[(i * width + j) * 3] = colorImage[i][j][0];
        colorBuffer[(i * width + j) * 3 + 1] = colorImage[i][j][1];
        colorBuffer[(i * width + j) * 3 + 2] = colorImage[i][j][2];
      }
    }
  }

  if (depthBuffer)
  {
    const auto& depthImage = virtualRenderer.getDepthImageRef();
    for (size_t i = 0; i < height; i++)
    {
      for (size_t j = 0; j < width; j++)
      {
        depthBuffer[i * width + j] = depthImage[i][j];
      }
    }
  }

  t_render = Timer_getSystemTime() - t_render;
  RLOG(1, "Rendering took %.1f msec", 1000.0 * t_render);
}

}   // namespace aff
