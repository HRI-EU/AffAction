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


static std::vector<double> computePixelRayIntersection3D(const RcsGraph* graph, const RcsBody* yoloBody,
                                                         const HTr* A_CI, const double I_ray[3])
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

  double pt[3];
  //double* pt = RcsBody_getStatePtr(graph, yoloBody);
  Vec3d_constMulAndAdd(pt, org, dir, s);

  return std::vector<double>(pt, pt+3);
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
  RLOG_CPP(2, "Received 'yolo':" << jsonString.dump(2));
  std::vector<YoloDetection> detections;

  try
  {
    // Iterate over each key ("yolo_1", "yolo_2", etc.)
    for (auto it = jsonString.begin(); it != jsonString.end(); ++it)
    {
      const auto& detectionJson = it.value(); // The object under "yolo_x"

      // Extract bounding box if it exists
      if (detectionJson.contains("bounding_box") && detectionJson["bounding_box"].is_object())
      {
        YoloDetection det;
        det.class_id = detectionJson.value("class_id", -1);
        det.class_name = detectionJson.value("class_name", "unknown");
        det.confidence = detectionJson.value("confidence", 0.0);

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
          RLOG(2, "Ray in cam: %f %f %f", C_ray[0], C_ray[1], C_ray[2]);
          Vec3d_transRotate(det.I_ray, A_camI.rot, C_ray);
          Vec3d_copy(det.C_ray, C_ray);
          RLOG(1, "Ray in world: %f %f %f", I_ray[0], I_ray[1], I_ray[2]);
          detections.push_back(det);
        }
        else
        {
          RLOG(1, "Could not compute ray");
        }

      }

    }
  }
  catch (const nlohmann::json::exception& e)
  {
    RLOG_CPP(0, "JSON Parsing Error: " << e.what());
  }

  RLOG_CPP(1, YoloDetectionsToString(detections));

  std::lock_guard<std::mutex> lock(updateMtx);
  this->yoloDetections = detections;
}

void YoloTracker::update(ActionScene* scene, RcsGraph* graph)
{
  RLOG_CPP(2, "YoloTracker::update()");
  std::vector<YoloDetection> detections;

  // Z points outwards from lens
  const RcsBody* cam = RcsGraph_getBodyByName(graph, "camera_0");
  RCHECK(cam);
  setCameraTransform(&cam->A_BI);

  // Thread-safe copying of detections from zmq thread
  {
    std::lock_guard<std::mutex> lock(updateMtx);
    detections = this->yoloDetections;
  }

  // Add RcsBody name to each detection.
  // Convention: Name is <yolo-category>_<detected_index>. If this name does not exist in the graph, it will be ignored
  // This algorithm looks a bit complex, but we do not enforce any ordering in the incoming json with respect to the names and indices.
  std::map<std::string,int> class_counts;
  for (auto& detection : detections)
  {
    auto it = class_counts.find(detection.class_name);
    if (it==class_counts.end())
    {
      class_counts[detection.class_name] = 0;
      it = class_counts.find(detection.class_name);
      RCHECK(it!=class_counts.end());
    }
    else
    {
      it->second++;
    }

    detection.yoloBdyName = it->first + "_" + std::to_string(it->second+1);
  }


  for (const auto& detection : detections)
  {
    RcsBody* yoloBody = RcsGraph_getBodyByName(graph, detection.yoloBdyName.c_str());

    if (RcsBody_numJoints(graph, yoloBody)<3)   // nullptr or not enough dof
    {
      RLOG_CPP(1, "Could not find or found invalid " << detection.yoloBdyName);
      continue;
    }

    RLOG_CPP(1, "Found " << detection.yoloBdyName);

    std::vector<double> pt = computePixelRayIntersection3D(graph, yoloBody, &cam->A_BI, detection.I_ray);
    double* q_rbj = RcsBody_getStatePtr(graph, yoloBody);
    Vec3d_copy(q_rbj, pt.data());
  }

}

void YoloTracker::update_hor(ActionScene* scene, RcsGraph* graph)
{
  RLOG_CPP(2, "YoloTracker::update()");
  std::vector<YoloDetection> detections;

  // Z points outwards from lens
  const RcsBody* cam = RcsGraph_getBodyByName(graph, "camera_0");
  RCHECK(cam);
  setCameraTransform(&cam->A_BI);

  // Thread-safe copying of detections from zmq thread
  {
    std::lock_guard<std::mutex> lock(updateMtx);
    detections = this->yoloDetections;
  }

  // Add RcsBody name to each detection.
  // Convention: Name is <yolo-category>_<detected_index>. If this name does not exist in the graph, it will be ignored
  // This algorithm looks a bit complex, but we do not enforce any ordering in the incoming json with respect to the names and indices.
  std::map<std::string,int> class_counts;
  for (auto& detection : detections)
  {
    auto it = class_counts.find(detection.class_name);
    if (it==class_counts.end())
    {
      class_counts[detection.class_name] = 0;
      it = class_counts.find(detection.class_name);
      RCHECK(it!=class_counts.end());
    }
    else
    {
      it->second++;
    }

    detection.yoloBdyName = it->first + "_" + std::to_string(it->second+1);
  }


  for (const auto& detection : detections)
  {
    RcsBody* yoloBody = RcsGraph_getBodyByName(graph, detection.yoloBdyName.c_str());

    if (RcsBody_numJoints(graph, yoloBody)<3)   // nullptr or not enough dof
    {
      RLOG_CPP(1, "Could not find or found invalid " << detection.yoloBdyName);
      continue;
    }

    RLOG_CPP(1, "Found " << detection.yoloBdyName);

    // Compute intersection
    const double height = 1.1;

    // org_z + s*dir_z = height => s = (height - org_z)/dir_z => intersect = org + s*dir
    const double* org = cam->A_BI.org;
    const double* dir = detection.I_ray;
    const double s = (height - org[2])/dir[2];
    double* pt = RcsBody_getStatePtr(graph, yoloBody);
    Vec3d_constMulAndAdd(pt, org, dir, s);
  }

}

void YoloTracker::setCameraTransform(const HTr* A_CI)
{
  //HTr_printComment("YOLO: Setting camera transform to:", A_CI);
  HTr_copy(&A_camI, A_CI);
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
        << "  Ray [in camera]: " << d.C_ray[0] << " " << d.C_ray[1] << " " << d.C_ray[2] << "\n"
        << "  Ray [in world]: " << d.I_ray[0] << " " << d.I_ray[1] << " " << d.I_ray[2] << "\n"
        << "----------------------------------------------------\n";
  }

  return oss.str();
}








}   // namespace aff
