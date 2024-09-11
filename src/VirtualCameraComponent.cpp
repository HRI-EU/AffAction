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

#include "VirtualCameraComponent.h"

#include <PPSGui.h>

#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_math.h>

#include <osgDB/WriteFile>

#include <thread>
#include <iomanip>
#include <sstream>


namespace aff
{

static
osg::ref_ptr<osg::Image> vectorToOsgImage(const std::vector<std::vector<std::vector<float>>>& imageData)
{
  // Get the dimensions of the input image
  int height = imageData.size();
  if (height == 0)
  {
    std::cerr << "Input data is empty." << std::endl;
    return nullptr;
  }

  int width = imageData[0].size();
  if (width == 0 || imageData[0][0].size() != 3)
  {
    std::cerr << "Input data is not correctly formatted." << std::endl;
    return nullptr;
  }

  // Allocate a buffer for the pixel data (using new to allow OSG to manage it)
  unsigned char* pixelData = new unsigned char[width * height * 3]; // 3 channels for RGB

  // Flatten the vector of vectors into a single array of unsigned char
  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      pixelData[(i * width + j) * 3 + 0] = static_cast<unsigned char>(imageData[i][j][0] * 255.0f); // Red
      pixelData[(i * width + j) * 3 + 1] = static_cast<unsigned char>(imageData[i][j][1] * 255.0f); // Green
      pixelData[(i * width + j) * 3 + 2] = static_cast<unsigned char>(imageData[i][j][2] * 255.0f); // Blue
    }
  }

  // Create an osg::Image and assign the pixel data to it
  osg::ref_ptr<osg::Image> osgImage = new osg::Image();
  osgImage->setImage(width, height, 1,  // width, height, depth
                     GL_RGB,            // Internal format (RGB)
                     GL_RGB,            // Pixel format
                     GL_UNSIGNED_BYTE,  // Data type
                     pixelData,         // Pointer to the pixel data
                     osg::Image::USE_NEW_DELETE); // Use USE_NEW_DELETE so OSG manages memory

  return osgImage;
}

VirtualCameraComponent::VirtualCameraComponent(EntityBase* parent, int width_, int height_,
                                               bool color, bool depth) :
  ComponentBase(parent), width(width_), height(height_),
  renderRGB(color), renderDepth(depth), recordImages(false),
  colData(nullptr), depthData(nullptr)
{
  subscribe("Capture", &VirtualCameraComponent::render);
  subscribe("TogglePixelGui", &VirtualCameraComponent::togglePixelGui);
  subscribe("ToggleRecording", &VirtualCameraComponent::toggleRecording);
  subscribe("Save", &VirtualCameraComponent::save);

  virtualRenderer = new Rcs::DepthRenderer(width, height);

  // These come from a Kinect v2 calbration
  double fx = 6.5746697810243404e+002;
  double cx = 3.1950000000000000e+002;
  double fy = 6.5746697810243404e+002;
  double cy = 2.3950000000000000e+002;
  double near = 0.3;
  double far = 10.0;
  virtualRenderer->setProjectionFromFocalParams(fx, fy, cx, cy, near, far);

  if (renderDepth)
  {
    this->depthData = new double[width * height];
    RCHECK(depthData);
    memset(this->depthData, 0, width*height*sizeof(double));
  }

  if (renderRGB)
  {
    this->colData = new double[width * height * 3];
    RCHECK(colData);
    memset(this->colData, 0, 3*width*height*sizeof(double));
  }

  //togglePixelGui();
}

VirtualCameraComponent::~VirtualCameraComponent()
{
}

void VirtualCameraComponent::togglePixelGui()
{
  if (pixelGui)
  {
    auto ptr = pixelGui.release();
    delete ptr;
  }
  else
  {
    std::vector<Rcs::PPSGui::Entry> pps;

    if (renderDepth)
    {
      pps.push_back(Rcs::PPSGui::Entry("Depth image", width, height, depthData, 1, 0.1));
    }

    if (renderRGB)
    {
      pps.push_back(Rcs::PPSGui::Entry("RGB image", width, height, colData, 3, 1.0));
    }

    pixelGui = std::make_unique<Rcs::PixelGui>(pps);
  }
}

void VirtualCameraComponent::setCameraTransform(const HTr* A_camI)
{
  virtualRenderer->setCameraTransform(A_camI);
}

void VirtualCameraComponent::setCameraTransform(const double xyzabc[6])
{
  HTr A_CI;
  HTr_from6DVector(&A_CI, xyzabc);
  virtualRenderer->setCameraTransform(&A_CI);
}

void VirtualCameraComponent::setSceneData(osg::Node* node)
{
  virtualRenderer->setSceneData(node);
}

void VirtualCameraComponent::render()
{
  const std::vector<std::vector<float>>& zImage = virtualRenderer->getDepthImageRef();
  const std::vector<std::vector<std::vector<float>>>& rgbImage = virtualRenderer->getRGBImageRef();

  double t_render = Timer_getSystemTime();
  virtualRenderer->frame();

  if (pixelGui)
  {
    // Update the pixel widget
    double* cDataPtr = colData;
    for (size_t i = 0; i < height; ++i)
    {
      for (size_t j = 0; j < width; ++j)
      {
        if (depthData)
        {
          depthData[i * width + j] = zImage[i][j];
        }

        if (colData)
        {
          cDataPtr[0] = rgbImage[i][j][0];
          cDataPtr[1] = rgbImage[i][j][1];
          cDataPtr[2] = rgbImage[i][j][2];
          cDataPtr += 3;
        }
      }

    }

  }   // if (pixelGui)

  if (recordImages)
  {
    osg::ref_ptr<osg::Image> img = vectorToOsgImage(rgbImage);
    recordings.push_back(img);
    RLOG_CPP(0, "Currenlty storing " << recordings.size() << " images.");
  }

  t_render = Timer_getSystemTime() - t_render;
  RLOG(1, "Rendering took %.1f msec", 1000.0 * t_render);
}

void VirtualCameraComponent::startRecording()
{
  recordImages = true;
}

void VirtualCameraComponent::stopRecording()
{
  recordImages = false;
}

void VirtualCameraComponent::clearRecordings()
{
  recordings.clear();
}

void VirtualCameraComponent::toggleRecording()
{
  if (recordImages)
  {
    stopRecording();
  }
  else
  {
    startRecording();
  }
}

bool VirtualCameraComponent::isRecording()
{
  return recordImages;
}

static void saveThread(std::vector<osg::ref_ptr<osg::Image>> images)
{
  static size_t runningIdx = 0;
  RLOG_CPP(0, "Saving " << images.size() << " images");

  for (size_t i = 0; i < images.size(); ++i)
  {
    // Create a stringstream to build the filename with padding
    std::ostringstream ss;
    ss << "rgb_" << std::setw(5) << std::setfill('0') << runningIdx++ << ".png";
    std::string filename = ss.str();

    RLOG_CPP(0, "Saving " << filename);
    bool success = osgDB::writeImageFile(*images[i].get(), filename);
    RCHECK(success);
  }

}

void VirtualCameraComponent::save()
{
  stopRecording();
  auto tmp = recordings;
  //saveThread(tmp);
  std::thread t(saveThread, std::move(tmp));
  t.detach();
  clearRecordings();
}

}   // namespace aff
