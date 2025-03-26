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
#include <Rcs_body.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <map>



static void computePixelRayIntersection3D(const RcsGraph* graph, const RcsBody* yoloBody,
                                          const HTr* A_CI, const double I_ray[3],
                                          double intersect_pt[3])
{
  const RcsBody* yoloParent = RCSBODY_BY_ID(graph, yoloBody->parentId);
  const HTr* A_PI = yoloParent ? &yoloParent->A_BI : HTr_identity();

  // A_CP: parent to camera frame
  HTr A_CP;
  HTr_invTransform(&A_CP, A_PI, A_CI);

  // P_ray: pixel ray in parent coordinates
  double P_ray[3];
  Vec3d_rotate(P_ray, MAT3D_CAST A_PI->rot, I_ray);

  // Compute intersection
  const double height = 0.0;

  // org_z + s*dir_z = height => s = (height - org_z)/dir_z => intersect = org + s*dir
  const double* org = A_CP.org;
  const double* dir = P_ray;
  const double s = (height - org[2])/dir[2];

  Vec3d_constMulAndAdd(intersect_pt, org, dir, s);
}

// Function to compute the 3D ray direction from a pixel
static bool pixel_to_ray(double u, double v, double K[3][3], double ray_[3])
{
  double K_inv[3][3];

  // Compute the inverse of the camera intrinsic matrix
  const double det = Mat3d_inverse(K_inv, K);
  if (det==0.0)
  {
    RLOG(1, "Camera intrinsic matrix is not invertible");
    REXEC(1)
    {
      Mat3d_printCommentDigits("Camera matrix", K, 8);
    }
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

static double computeLengthToRayPoint(const int pixel2[2],
                                      const double intersect_pt[3],
                                      double K[3][3])
{
  double ray2[3];
  if (!pixel_to_ray(pixel2[0], pixel2[1], K, ray2))
  {
    return -1.0;
  }

  // Scale ray so it lies at the same Z-depth as intersect_pt
  double scale = intersect_pt[2] / ray2[2];
  double point2[3] =
  {
    scale* ray2[0],
    scale* ray2[1],
    scale* ray2[2]
  };

  // Euclidean distance between known point and reconstructed 3D point
  double dx = point2[0] - intersect_pt[0];
  double dy = point2[1] - intersect_pt[1];
  double dz = point2[2] - intersect_pt[2];

  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

namespace aff
{

YoloTracker::YoloTracker(const std::string& cameraName) : TrackerBase(cameraName), newYoloUpdate(false), maxAge(2.0)
{
  Mat3d_setZero(camera_matrix);
}

YoloTracker::~YoloTracker()
{
}

std::string YoloTracker::getRequestKeyword() const
{
  return "yolo";
}

void YoloTracker::parse(const nlohmann::json& jsonHeader, const nlohmann::json& jsonData, double time)
{
  // Extract the camera matrix
  if (jsonHeader.contains("camera_matrix"))
  {
    try
    {
      std::vector<std::vector<double>> camera_matrix = jsonHeader["camera_matrix"].get<std::vector<std::vector<double>>>();
      setCameraMatrix(camera_matrix);
    }
    catch (const std::exception& e)
    {
      RLOG_CPP(1, "Error parsing camera_matrix: " << e.what());
    }
  }
  else
  {
    RLOG_CPP(1, "No camera_matrix in json header" << jsonHeader.dump(2));
  }


  RLOG_CPP(2, "Received 'yolo':" << jsonData.dump(2));
  std::lock_guard<std::mutex> lock(updateMtx);
  this->newYoloUpdate = true;

  try
  {
    // Iterate over each key ("yolo_1", "yolo_2", etc.)
    // "yolo_1":          # it.key()
    // {                  # it.value() = detectJson
    //   "bounding_box": {"x1": 237, "x2": 288, "y1": 124, "y2": 180},
    //   "class_id": 47,
    //   "class_name": "apple",
    //   "confidence": 0.87,
    //   "frame_index": 0
    // },
    size_t new_data_count = 0;
    for (auto it = jsonData.begin(); it != jsonData.end(); ++it)
    {
      const auto& detectionJson = it.value();

      if (!detectionJson.contains("bounding_box") ||
          !detectionJson["bounding_box"].is_object())
      {
        continue;
      }

      YoloDetection det;
      det.lastUpdate = time;
      det.class_id = detectionJson.value("class_id", -1);
      det.class_name = detectionJson.value("class_name", "unknown");
      det.confidence = detectionJson.value("confidence", 0.0);

      const auto& bboxJson = detectionJson["bounding_box"];
      det.x1 = bboxJson.value("x1", 0);
      det.y1 = bboxJson.value("y1", 0);
      det.x2 = bboxJson.value("x2", 0);
      det.y2 = bboxJson.value("y2", 0);

      if (new_data_count >= yoloDetections.size())
      {
        yoloDetections.push_back(det);
      }
      else
      {
        YoloDetection& closest = det.findClosest(yoloDetections);
        closest = det;
      }

      new_data_count++;
    }
  }
  catch (const nlohmann::json::exception& e)
  {
    RLOG_CPP(0, "JSON Parsing Error: " << e.what());
  }

}

void YoloTracker::update(ActionScene* scene, RcsGraph* graph)
{
  if (frozen)
  {
    return;
  }

  const double t = getWallclockTime();

  std::lock_guard<std::mutex> lock(updateMtx);

  // Add RcsBody name to each detection.
  // Convention: Name is <yolo-category>_<detected_index>. If this name does
  // not exist in the graph, it will be ignored. This algorithmdoes not assume
  // any ordering in the incoming json with respect to the names and indices.
  std::map<std::string, int> class_counts;
  for (auto& detection : yoloDetections)
  {
    // Increment the count for this class and get the new count
    int& count = class_counts[detection.class_name];
    detection.yoloBdyName = detection.class_name + "_" + std::to_string(++count);
  }

  // Update the detections by deleting the old ones. We do it in every time step
  // so that we don't depend on any percepts.
  for (auto it = yoloDetections.begin(); it != yoloDetections.end();)
  {
    if (t - it->lastUpdate > getMaxAge())
    {
      RcsBody* yoloBody = RcsGraph_getBodyByName(graph, it->yoloBdyName.c_str());
      double* q_rbj = RcsBody_getStatePtr(graph, yoloBody);
      RCHECK_MSG(q_rbj, "Body not found or issues with dof: '%s'", it->yoloBdyName.c_str());
      Vec3d_set(q_rbj, 0.0, 0.0, -10.0);
      yoloDetections.erase(it);
    }
    else
    {
      ++it;
    }
  }

  // Update coordinates only after new percept has been received
  if (!this->newYoloUpdate)
  {
    return;
  }

  this->newYoloUpdate = false;


  RLOG_CPP(1, YoloDetectionsToString(yoloDetections));

  // Z points outwards from lens
  RcsBody* cam = TrackerBase::getBody(graph, cameraNamedId);
  RCHECK_MSG(cam, "Body %s with id %d", cameraNamedId.first.c_str(), cameraNamedId.second);

  // Go through detections and assign 3d coordinates
  for (const auto& detection : yoloDetections)
  {
    RcsBody* yoloBody = RcsGraph_getBodyByName(graph, detection.yoloBdyName.c_str());

    if (RcsBody_numJoints(graph, yoloBody)<3)   // nullptr or not enough dof
    {
      RLOG_CPP(1, "Could not find or found invalid " << detection.yoloBdyName);
      continue;
    }

    // Ignore the held-in-hand objects
    const bool heldInHand = RcsBody_isArticulated(graph, yoloBody);
    if (heldInHand)
    {
      continue;
    }

    // Compute camera ray in world coordinates. Coordinate y2 is the
    // lower edge of the bounding box.
    const int center_pixel_u = (detection.x1 + detection.x2) / 2;
    const int center_pixel_v = detection.y2;
    double C_ray[3];
    const bool ray_success = pixel_to_ray(center_pixel_u, center_pixel_v, camera_matrix, C_ray);
    if (!ray_success)
    {
      RLOG(1, "Could not compute ray");
      continue;
    }

    double I_ray[3];
    double* q_rbj = RcsBody_getStatePtr(graph, yoloBody);
    Vec3d_transRotate(I_ray, cam->A_BI.rot, C_ray);
    computePixelRayIntersection3D(graph, yoloBody, &cam->A_BI, I_ray, q_rbj);

    double lb[3], ub[3];
    bool hasAABB = RcsGraph_computeBodyAABB(graph, yoloBody->id, -1, lb, ub, NULL);

    if (hasAABB)
    {
      double z_offset = yoloBody->A_BI.org[2] - lb[2];
      q_rbj[2] += z_offset;
      RLOG(2, "Compensating z for %f (%f %f)", z_offset, yoloBody->A_BI.org[2], lb[2]);
    }

    RLOG(2, "q_rbj: %f %f %f", q_rbj[0], q_rbj[1], q_rbj[2]);



    // int up_pixel[2];
    // up_pixel[0] = center_pixel_u;
    // up_pixel[1] = detection.y1;
    // double diameter = computeLengthToRayPoint(up_pixel, C_ray, camera_matrix);
    // RLOG(0, "Yolo[%s] diameter: %.1f mm", yoloBody->name, 1000.0*diameter);


  }

}

void YoloTracker::setCameraMatrix(double K[3][3])
{
  Mat3d_copy(this->camera_matrix, K);
}

void YoloTracker::setCameraMatrix(const std::vector<std::vector<double>>& K)
{
  if (K.empty())
  {
    return;
  }

  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      this->camera_matrix[i][j] = K[i][j];
    }
  }
}

std::string YoloTracker::YoloDetectionsToString(const std::vector<YoloTracker::YoloDetection>& detections)
{
  std::ostringstream oss;

  oss << "Detected Objects: (" << detections.size() << " detections)\n";
  oss << "----------------------------------------------------\n";

  for (const auto& d : detections)
  {
    oss << "Class: " << d.class_name << " (ID: " << d.class_id << ")\n"
        << "  Bounding Box: [" << d.x1 << ", " << d.y1
        << "] -> [" << d.x2 << ", " << d.y2 << "]\n"
        << "  Confidence: " << d.confidence * 100 << "%\n"
        << "----------------------------------------------------\n";
  }

  return oss.str();
}

void YoloTracker::setMaxAge(double age)
{
  this->maxAge = age;
}

double YoloTracker::getMaxAge() const
{
  return this->maxAge;
}

YoloTracker::YoloDetection::YoloDetection() : class_id(-1), x1(0), y1(0), x2(0), y2(0), confidence(0.0), lastUpdate(0.0)
{
}

// Distance is sum of squared edge distances
double YoloTracker::YoloDetection::distance(const YoloTracker::YoloDetection& other) const
{
  int dx, dy, dist = 0;

  dx = x1 - other.x1;
  dy = y1 - other.y1;
  dist += dx * dx + dy * dy;

  dx = x2 - other.x2;
  dy = y1 - other.y1;
  dist += dx * dx + dy * dy;

  dx = x2 - other.x2;
  dy = y2 - other.y2;
  dist += dx * dx + dy * dy;

  dx = x1 - other.x1;
  dy = y2 - other.y2;
  dist += dx * dx + dy * dy;

  return dist;
}

YoloTracker::YoloDetection& YoloTracker::YoloDetection::findClosest(std::vector<YoloTracker::YoloDetection>& yoloDetections) const
{
  RCHECK(!yoloDetections.empty());

  int dMin = distance(yoloDetections[0]);
  size_t index_min = 0;

  for (size_t i = 1; i < yoloDetections.size(); ++i)
  {
    double di = distance(yoloDetections[i]);
    if (di < dMin)
    {
      dMin = di;
      index_min = i;
    }
  }

  return yoloDetections[index_min];
}







}   // namespace aff
