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

VirtualCamera::VirtualCamera(const std::string& cameraType,
                             osg::Node* node, int width, int height,
                             double near, double far):
  virtualRenderer(width, height, near, far)
{
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

  double fx, cx, fy, cy;
  bool success = initCamera(cameraType, width, height, fx, fy, cx, cy, near, far);
  RCHECK(success);
  virtualRenderer.setProjectionFromFocalParams(fx, fy, cx, cy, near, far);
}

bool VirtualCamera::initCamera(const std::string& cameraName, int width, int height,
                               double& fx, double& fy, double& cx, double& cy, double& near, double& far)
{

  if (cameraName=="Kinect_v2")
  {
    fx = 1.36972287105 * height;
    fy = fx;
    cx = 0.5*width - 0.5;
    cy = 0.5*height - 0.5;
  }
  else if (cameraName=="Logitech_C910")
  {
    fx = 1.2602 * height;
    fy = fx;
    cx = 0.5*width - 0.5;
    cy = 0.5*height - 0.5;
  }
  else if (cameraName=="AzureKinect")
  {
    fx = 1.04857360564 * height;
    fy = fx;
    cx = 0.5*width - 0.5;
    cy = 0.5*height - 0.5;
  }
  else if (cameraName=="AzureKinect_WFOV")
  {
    fx = 0.8201975534 * height;
    fy = fx;
    cx = 0.5*width - 0.5;
    cy = 0.5*height - 0.5;
  }
  else
  {
    return false;
  }

  near = 0.3;
  far = 10.0;

  return true;
}

void VirtualCamera::capture(const HTr* A_camI)
{
  virtualRenderer.setCameraTransform(A_camI);
  virtualRenderer.frame();
}

void VirtualCamera::getColorImage(uint8_t* data, size_t size)
{
  virtualRenderer.getColorImage(data, size);
}

void VirtualCamera::getDepthImage(float* data, size_t size)
{
  virtualRenderer.getDepthImage(data, size);
}

Rcs::DepthRenderer* VirtualCamera::getRenderer()
{
  return &virtualRenderer;
}

size_t VirtualCamera::getHeight() const
{
  return virtualRenderer.getHeight();
}

size_t VirtualCamera::getWidth() const
{
  return virtualRenderer.getWidth();
}

}   // namespace aff
