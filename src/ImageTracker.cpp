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

#include "ImageTracker.h"
#include "ImageHelpers.h"
#include "SceneHelpers.h"
#include "SceneJsonHelpers.h"

#include <Rcs_macros.h>
#include <GraphNode.h>

#include <algorithm>
#include <cmath>



namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
ImageTracker::ImageTracker(EntityBase* parent, const std::string& cameraName) :
  ComponentBase(parent), TrackerBase(cameraName), t_parse(0.0), showDebugWindow(false)
{
  subscribe("SetGazeTarget", &ImageTracker::onSetGazeTarget);
}

void ImageTracker::onSetGazeTarget(std::string bdyName)
{
  this->gazeTarget = bdyName;
}

void ImageTracker::enableDebugWindow(bool enable)
{
  this->showDebugWindow = enable;
}

std::string ImageTracker::getRequestKeyword() const
{
  return "image";
}

void ImageTracker::parse(const nlohmann::json& header, const nlohmann::json& data, double time)
{
  const double t_prev = t_parse;
  t_parse = getWallclockTime();

  try
  {
    // Extract sequence number safely
    if (!header.contains("seq") || !header["seq"].is_number_integer())
    {
      RLOG_CPP(1, "Warning: 'seq' not found or not an integer in header.");
      return;
    }

    std::string err;
    PinholeCamera phCam;
    if (!extract_intrinsics(header, phCam, err))
    {
      RLOG_CPP(1, "Failed to extract camera intrinsics: " << err << " header: " << header.dump(2));
      return;
    }

    // Serialize image JSON
    int stamp = header.at("seq").get<int>();
    std::string image_str = data.dump();

    {
      std::lock_guard<std::mutex> lock(imgMtx);
      this->stamped_image = std::make_pair(stamp, image_str);
      this->pinhole = phCam;
    }

    RLOG_CPP(1, "Received: count=" << stamped_image.first
             << " after " << 1.0e3 * (t_parse - t_prev) << " msec");
  }
  catch (const nlohmann::json::exception& e)
  {
    RLOG_CPP(1, "JSON processing error: " << e.what());
  }
  catch (const std::exception& e)
  {
    RLOG_CPP(1, "General error during JSON handling: " << e.what());
  }

}

void ImageTracker::update(ActionScene* scene, RcsGraph* graph)
{
  PinholeCamera tmpCam;

  {
    std::lock_guard<std::mutex> lock(imgMtx);
    tmpCam = this->pinhole;
  }

  std::vector<int> bb = getObjectBoundingBox(scene, graph, gazeTarget, getCameraName(), tmpCam, false);

  {
    std::lock_guard<std::mutex> lock(imgMtx);
    this->gaze_bb = bb;
  }

  if (this->showDebugWindow)
  {
    static size_t count = 0;

    if (++count % 10 == 0)
    {
      updateDebugWindow(bb);
    }
  }

}

void ImageTracker::updateDebugWindow(const std::vector<int>& bb) const
{
  if (bb.size() != 4)
  {
    RLOG_CPP(1, "Invalid bounding box of size " << bb.size());
    return;
  }

  std::pair<int, std::string> img_pair = getStampedImage();

  if (img_pair.second.empty())
  {
    return;
  }

  QString b64_qt = QString::fromStdString(img_pair.second);
  QImage image = decodeBase64JpegToQImage(b64_qt);

  // Bounding box: minX, minY, maxX, maxY
  QRect boundingBox(QPoint(bb[0], bb[1]), QPoint(bb[2], bb[3]));
  QRect safeBox = boundingBox & image.rect();  // ensures boundingBox is within image
  image = image.copy(safeBox);

  constexpr int kMinWidth = 160;
  if (image.width() < kMinWidth)
  {
    int h = static_cast<int>(std::round(static_cast<double>(kMinWidth) *
                                        image.height() / image.width()));
    image = image.scaled(kMinWidth, h, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  }

  showFrame(image);
}

std::pair<int, std::string> ImageTracker::getStampedImage(int frame_count) const
{
  std::lock_guard<std::mutex> lock(imgMtx);
  if ((frame_count == -1) || (stamped_image.first > frame_count))
  {
    return this->stamped_image;
  }

  return std::make_pair(stamped_image.first, std::string());
}

std::vector<int> ImageTracker::getObjectBoundingBox(const ActionScene* scene,
                                                    const RcsGraph* graph,
                                                    const std::string objName,
                                                    const std::string& cameraName,
                                                    const PinholeCamera& phCam,
                                                    bool isVirtualCamera)
{
  nlohmann::json j = getObjectInCamera(objName, cameraName, scene, graph);

  if (!j.contains("vertex"))
  {
    RLOG_CPP(1, "No 'vertex' key in JSON: " << j.dump(2));
    return std::vector<int>();
  }

  if (!j["vertex"].is_array())
  {
    RLOG_CPP(1, "JSON or it's not an array: " << j.dump(2));
    return std::vector<int>();
  }

  std::vector<std::array<double, 3>> vertices;

  for (const auto& item : j["vertex"])
  {
    if (item.is_array() && item.size() == 3)
    {
      vertices.push_back({ item[0], item[1], item[2] });
    }
    else
    {
      RLOG_CPP(1, "Warning: malformed vertex entry.");
      return std::vector<int>();
    }
  }


  // Project to pixel coordinates
  std::vector<std::array<int, 2>> imgPoints;
  for (const auto& v : vertices)
  {
    // The assumed camera orientation is:
    // +X: right
    // +Y: down
    // +Z: forward/outward from the camera
    //
    // We assume that the camera is oriented with x pointing forward, and z
    // pointing up. Here, we rotate it such that z points outwards, y points
    // downwards, and x points right.
    // double x_std = -v[1];  // Y
    // double y_std = -v[2];  // Z
    // double z_std =  v[0];  // X
    //RLOG(0, "VERTEX: %f %f %f", x_std, y_std, z_std);

    // That's needed for the real camera
    // x_std = v[0];
    // y_std = v[1];
    // z_std = v[2];

    double x_std, y_std, z_std;  // Y

    if (isVirtualCamera)
    {
      // The assumed camera orientation is:
      // +X: right
      // +Y: down
      // +Z: forward/outward from the camera
      //
      // We assume that the camera is oriented with x pointing forward, and z
      // pointing up. Here, we rotate it such that z points outwards, y points
      // downwards, and x points right.
      x_std = -v[1];  // Y
      y_std = -v[2];  // Z
      z_std =  v[0];  // X
    }
    else
    {
      // That's needed for the real camera
      x_std = v[0];
      y_std = v[1];
      z_std = v[2];
    }

    if (!std::isfinite(z_std) || std::fabs(z_std) < 1.0e-12)
    {
      RLOG_CPP(1, "Skipping vertex with invalid camera depth");
      continue;
    }

    // Project with the same rational radial/tangential model that
    // computeCameraGazeDirection inverts.
    const double xu = x_std / z_std;
    const double yu = y_std / z_std;
    const double r2 = xu*xu + yu*yu;
    const double r4 = r2*r2;
    const double r6 = r4*r2;
    const double radialDenominator =
      1.0 + phCam.k4*r2 + phCam.k5*r4 + phCam.k6*r6;
    if (!std::isfinite(radialDenominator) ||
        std::fabs(radialDenominator) < 1.0e-12)
    {
      RLOG_CPP(1, "Skipping vertex at which the camera distortion model is singular");
      continue;
    }

    const double radial =
      (1.0 + phCam.k1*r2 + phCam.k2*r4 + phCam.k3*r6) /
      radialDenominator;
    const double xd = xu*radial + 2.0*phCam.p1*xu*yu +
                      phCam.p2*(r2 + 2.0*xu*xu);
    const double yd = yu*radial + phCam.p1*(r2 + 2.0*yu*yu) +
                      2.0*phCam.p2*xu*yu;
    const double x = phCam.fx*xd + phCam.skew*yd + phCam.cx;
    const double y = phCam.fy*yd + phCam.cy;
    if (!std::isfinite(x) || !std::isfinite(y))
    {
      RLOG_CPP(1, "Skipping vertex with invalid projected coordinates");
      continue;
    }

    imgPoints.push_back({ static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)) });
  }

  if (imgPoints.empty())
  {
    RLOG_CPP(1, "No points to compute bounding box.");
    return std::vector<int>();
  }


  // Assign bounding box
  int minX = imgPoints[0][0];
  int maxX = imgPoints[0][0];
  int minY = imgPoints[0][1];
  int maxY = imgPoints[0][1];

  for (const auto& p : imgPoints)
  {
    minX = std::min(minX, p[0]);
    maxX = std::max(maxX, p[0]);
    minY = std::min(minY, p[1]);
    maxY = std::max(maxY, p[1]);
  }

  return std::vector<int> {minX, minY, maxX, maxY};
}

PinholeCamera ImageTracker::getCameraModel() const
{
  std::lock_guard<std::mutex> lock(imgMtx);
  return this->pinhole;
}

std::vector<int> ImageTracker::getGazeObjectBoundingBox() const
{
  std::lock_guard<std::mutex> lock(imgMtx);
  return gaze_bb;
}










/*******************************************************************************
 *
 ******************************************************************************/
VirtualImageTracker::VirtualImageTracker(EntityBase* parent,
                                         const std::string& cameraName,
                                         const std::string& cameraType_,
                                         int width,
                                         int height) :
  ImageTracker(parent, cameraName), cameraType(cameraType_), capture_count(0), vCamPtr(nullptr)
{
  this->pinhole.width = width;
  this->pinhole.height = height;
}

void VirtualImageTracker::parse(const nlohmann::json& header, const nlohmann::json& data, double time)
{
}

std::string VirtualImageTracker::getRequestKeyword() const
{
  return "virtual_image";
}

void VirtualImageTracker::update(ActionScene* scene, RcsGraph* graph)
{
  static int count = 0;

  if (++count % 5 != 0)
  {
    return;
  }

  // After this, we have a valid capture camera and pinhole camera model
  if (!vCamPtr)
  {
    vCamPtr = std::make_unique<VirtualCamera>(cameraType, new Rcs::GraphNode(graph), pinhole.width, pinhole.height);
    vCamPtr->getRenderer()->getFocalParams(pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy);
  }

  HTr A_camI = getCameraTransform(graph);
  vCamPtr->capture(&A_camI);
  capture_count++;

  std::vector<uint8_t> colorImageUint8(pinhole.height*pinhole.width*3);
  vCamPtr->getColorImage(colorImageUint8.data(), colorImageUint8.size());

  int quality = 90;
  std::string image_str = rgbToJpegBase64(colorImageUint8.data(), pinhole.width, pinhole.height, quality);

  // RLOG_CPP(0, "width: " << pinhole.width << " height: " << pinhole.height);
  // RLOG_CPP(0, "image_str is of size " << image_str.size() << " image size: " << colorImageUint8.size());

  // Project to pixel coordinates
  std::vector<int> bb = getObjectBoundingBox(scene, graph, gazeTarget, getCameraName(), pinhole, true);

  {
    std::lock_guard<std::mutex> lock(imgMtx);
    this->stamped_image = std::make_pair(capture_count, image_str);
    this->gaze_bb = bb;
  }

  if (this->showDebugWindow)
  {
    // // Get image directly from capture
    // const int bytesPerLine = pinhole.width * 3;
    // QImage img(colorImageUint8.data(), pinhole.width, pinhole.height, bytesPerLine, QImage::Format_RGB888);
    // showFrame(img.copy());

    // Get image the long way through decoding etc.
    QString b64_qt = QString::fromStdString(image_str);
    QImage image = decodeBase64JpegToQImage(b64_qt);

    if (gaze_bb.size() == 4)
    {
      // Here we have a valid bounding box: minX, minY, maxX, maxY
      QRect boundingBox(QPoint(gaze_bb[0], gaze_bb[1]), QPoint(gaze_bb[2], gaze_bb[3]));
      QRect safeBox = boundingBox & image.rect();  // ensures boundingBox is within image
      image = image.copy(safeBox);

      constexpr int kMinWidth = 160;
      if (image.width() < kMinWidth)
      {
        int h = static_cast<int>(std::round(static_cast<double>(kMinWidth) *
                                            image.height() / image.width()));
        image = image.scaled(kMinWidth, h, Qt::KeepAspectRatio, Qt::SmoothTransformation);
      }

    }

    showFrame(image);
  }

}




}   // namespace
