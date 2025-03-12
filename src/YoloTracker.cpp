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

#include "YoloTracker.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>

#include <cstdio>
#include <iostream>
#include <sstream>


// Function to compute the 3D ray direction from a pixel
static bool pixel_to_ray(double u, double v, double K[3][3], double ray_[3])
{
  double K_inv[3][3];

  // Compute the inverse of the camera intrinsic matrix
  const double det = Mat3d_inverse(K_inv, K);
  if (det==0.0)
  {
    RLOG(1, "Camera intrinsic matrix is not invertible");
    return false;
  }

  // Homogeneous pixel coordinate
  double pixel_homogeneous[3] = { u, v, 1.0 }, ray[3] = { 0.0, 0.0, 0.0 };

  // Compute ray direction in camera coordinates: ray = K_inv * pixel_homogeneous
  Vec3d_rotate(ray, K_inv, pixel_homogeneous);

  // Normalize the ray direction (optional, makes it a unit vector)
  const double norm = Vec3d_normalize(ray_, ray);

  if (norm == 0.0)
  {
    RLOG(1, "Failed to normalize ray");
    return false;
  }

  return true;
}


namespace aff
{

YoloTracker::YoloTracker()
{
  HTr_setIdentity(&A_camI);
}

YoloTracker::~YoloTracker()
{
}

std::string YoloTracker::getRequestKeyword() const
{
  return "yolo";
}

void YoloTracker::parse(const nlohmann::json& jsonString, double time, const std::string& cameraFrame)
{
  RLOG_CPP(1, "Received 'yolo':" << jsonString.dump(2));
  std::vector<YoloDetection> detections;

  try
  {
    // Iterate over each key ("yolo_1", "yolo_2", etc.)
    for (auto it = jsonString.begin(); it != jsonString.end(); ++it)
    {
      const auto& detectionJson = it.value(); // The object under "yolo_x"

      YoloDetection det;
      det.class_id = detectionJson.value("class_id", -1);
      det.class_name = detectionJson.value("class_name", "unknown");
      det.confidence = detectionJson.value("confidence", 0.0);
      det.frame_index = detectionJson.value("frame_index", 0);
      Vec3d_setZero(det.I_ray);

      // Extract bounding box if it exists
      if (detectionJson.contains("bounding_box") && detectionJson["bounding_box"].is_object())
      {
        const auto& bboxJson = detectionJson["bounding_box"];
        det.x1 = bboxJson.value("x1", 0);
        det.y1 = bboxJson.value("y1", 0);
        det.x2 = bboxJson.value("x2", 0);
        det.y2 = bboxJson.value("y2", 0);

        int center_pixel_u = (det.x1 + det.x2) / 2;
        int center_pixel_v = det.y1;

        // Compute camera ray in world coordinates
        double C_ray[3], I_ray[3];
        bool ray_success = pixel_to_ray(center_pixel_u, center_pixel_v, camera_matrix, C_ray);
        if (ray_success)
        {
          RLOG(1, "Ray in cam: %f %f %f", C_ray[0], C_ray[1], C_ray[2]);
          Vec3d_transRotate(det.I_ray, A_camI.rot, C_ray);
          RLOG(1, "Ray in world: %f %f %f", I_ray[0], I_ray[1], I_ray[2]);
        }
        else
        {
          RLOG(1, "Could not compute ray");
        }

      }

      // Store the parsed detection in the vector
      detections.push_back(det);
    }
  }
  catch (const nlohmann::json::exception& e)
  {
    std::cerr << "JSON Parsing Error: " << e.what() << std::endl;
  }

  RLOG_CPP(1, "Num detections: " << detections.size());
  RLOG_CPP(1, YoloDetectionsToString(detections));

  std::lock_guard<std::mutex> lock(updateMtx);
  this->yoloDetections = detections;
}

void YoloTracker::update(ActionScene* scene, RcsGraph* graph)
{
  RLOG_CPP(2, "YoloTracker::update()");
  std::vector<YoloDetection> detections;

  {
    std::lock_guard<std::mutex> lock(updateMtx);
    detections = this->yoloDetections;
  }

}

void YoloTracker::setCameraTransform(const HTr* A_CI)
{
  HTr_copy(&A_camI, A_CI);
}

std::string YoloTracker::YoloDetectionsToString(const std::vector<YoloTracker::YoloDetection>& detections)
{
  std::ostringstream oss;

  oss << "Detected Objects: (" << detections.size() << " detections)\n";
  oss << "----------------------------------------------------\n";

  for (const auto& d : detections)
  {
    oss << "Frame: " << d.frame_index << " | Class: " << d.class_name
        << " (ID: " << d.class_id << ")\n"
        << "  Bounding Box: [" << d.x1 << ", " << d.y1
        << "] -> [" << d.x2 << ", " << d.y2 << "]\n"
        << "  Confidence: " << d.confidence * 100 << "%\n"
        << "  Ray [in world]: " << d.I_ray[0] << " " << d.I_ray[1] << " " << d.I_ray[2] << "\n"
        << "----------------------------------------------------\n";
  }

  return oss.str();
}








}   // namespace aff
