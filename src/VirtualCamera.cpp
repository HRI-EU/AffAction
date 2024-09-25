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
#include <Rcs_material.h>
#include <Rcs_math.h>

#include <osg/LightSource>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>
#include <osgFX/Cartoon>

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

  // This comes for the Logitech C910 through ChatGPT
  fx = 1.2602 * height;
  fy = 1.2602 * height;
  cx = 0.5*width - 0.5;
  cy = 0.5*height - 0.5;

  // These come from Azure Kinect calibration
  fx = 1.04857360564 * height;
  cx = 0.5*width - 0.5;
  fy = 1.04857360564 * height;
  cy = 0.5*height - 0.5;

  // These come from Azure Kinect WFOV calibration
  fx = 0.8201975534 * height;
  cx = 0.5*width - 0.5;
  fy = 0.8201975534 * height;
  cy = 0.5*height - 0.5;

  //osg::ref_ptr<osgFX::Cartoon> rootnode = new osgFX::Cartoon;
  osg::ref_ptr<osg::Group> rootnode = new osg::Group;

  double rgba[4];
  Rcs_colorFromString("LIGHT_GRAYISH_GREEN", rgba);
  osg::ref_ptr<osg::ClearNode> clearNode = new osg::ClearNode;
  clearNode->setClearColor(osg::Vec4(rgba[0], rgba[1], rgba[2], rgba[3]));
  rootnode->addChild(clearNode.get());
  rootnode->addChild(node);

  // Disable default light
  rootnode->getOrCreateStateSet()->setMode(GL_LIGHT0, osg::StateAttribute::OFF);

  // Light source that moves with the camera
  osg::ref_ptr<osg::LightSource> cameraLight = new osg::LightSource;
  cameraLight->getLight()->setLightNum(1);
  cameraLight->getLight()->setPosition(osg::Vec4(0.0, 0.0, 10.0, 1.0));
  cameraLight->getLight()->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));
  rootnode->addChild(cameraLight.get());
  rootnode->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);



  // Shadow map scene. We use the sunlight to case shadows.
  osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
  sm->setTextureSize(osg::Vec2s(2048, 2048));

  // Light source that shines down
  osg::ref_ptr<osg::LightSource> sunlight = new osg::LightSource;
  sunlight->getLight()->setLightNum(2);
  sunlight->getLight()->setPosition(osg::Vec4(0.0, 0.0, 10.0, 1.0));
  rootnode->addChild(sunlight.get());
  rootnode->getOrCreateStateSet()->setMode(GL_LIGHT2, osg::StateAttribute::ON);
  sm->setLight(sunlight->getLight());

  sm->setPolygonOffset(osg::Vec2(-0.7, 0.0));
  sm->setAmbientBias(osg::Vec2(0.7, 0.3));   // values need to sum up to 1.0

  osg::ref_ptr<osgShadow::ShadowedScene> shadowScene = new osgShadow::ShadowedScene;
  shadowScene->setShadowTechnique(sm.get());
  shadowScene->addChild(rootnode.get());
  // shadowScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
  // shadowScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);


  // Set anti-aliasing
  osg::ref_ptr<osg::DisplaySettings> ds = new osg::DisplaySettings;
  ds->setNumMultiSamples(4);
  virtualRenderer.setDisplaySettings(ds.get());
  virtualRenderer.setSceneData(shadowScene.get());
  virtualRenderer.setProjectionFromFocalParams(fx, fy, cx, cy, near, far);
}

VirtualCamera::~VirtualCamera()
{}

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
