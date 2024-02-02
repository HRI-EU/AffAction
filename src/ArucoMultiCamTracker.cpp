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

#include "ArucoMultiCamTracker.h"

#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_quaternion.h>

#include <tuple>


namespace aff
{

static const double default_marker_length = 1.0;



/*******************************************************************************
  We require the camera to be a child of the base marker, for instane:

  <Body name="aruco_base" rigid_body_joints="0 0 0.8 0 0 0" color="PEWTER">
    <Shape type="BOX" extents="0.14 0.14 0.001" graphics="true"
           textureFile="aruco/4x4marker_0.jpg"/>
    <Shape type="FRAME" marker="true" markerName="aruco_10" extents="0.14 0.14 0"
           scale="0.2" transform="-0.44 -1.44 0 0 0 0"/>
    </Body>

  <Body name="camera" prev="aruco_base" rigid_body_joints="0 0.5 0.48 121 -6 177">
    <Shape type="FRAME" scale="0.2" graphics="true"/>
  </Body>

 *******************************************************************************/
class ArucoMultiCamCalibrator
{
public:

  ArucoMultiCamCalibrator(const std::string& cameraBodyName,
                          const std::string& baseMarkerBdyName);

  virtual ~ArucoMultiCamCalibrator();

  // Process aruco frames. Called from control loop (100Hz or so)
  void updateCalibration(RcsGraph* graph, const std::map<std::string,ArucoMultiCamTracker::ArucoMarkerBodyData>& arucoBodies);

  void startCalibration(size_t numFrames);
  void stopCalibration();
  bool isCalibrating() const;

private:
  std::string cameraBodyName;
  std::string baseMarkerBdyName;
  int updateCameraPoseFromAruco;
  double tmc;
  size_t numCalibrationSteps;
  std::vector<std::function<void(const HTr*)>> cameraCalibrationFinishedCb;
};

ArucoMultiCamCalibrator::ArucoMultiCamCalibrator(const std::string& cameraBodyName_, const std::string& baseMarkerBdyName_) : cameraBodyName(cameraBodyName_), baseMarkerBdyName(baseMarkerBdyName_), updateCameraPoseFromAruco(-1), tmc(0.05), numCalibrationSteps(20)
{
}

ArucoMultiCamCalibrator::~ArucoMultiCamCalibrator()
{
}

void ArucoMultiCamCalibrator::updateCalibration(RcsGraph* graph, const std::map<std::string,ArucoMultiCamTracker::ArucoMarkerBodyData>& arucoBodies)
{
  if (updateCameraPoseFromAruco<0)
  {
    return;
  }

  auto it = arucoBodies.find(baseMarkerBdyName);
  if (it == arucoBodies.end())
  {
    RLOG(0, "Couldn't find base marker %s for camera %s",
         baseMarkerBdyName.c_str(), cameraBodyName.c_str());
    return;
  }

  const ArucoMultiCamTracker::ArucoMarkerBodyData& baseMarker = it->second;
  RCHECK(baseMarker.isBaseMarker);
  RCHECK(baseMarker.shapes.size()==1);

  // From here, the baseMarker is the camera's base marker
  RLOG(0, "Calibration step %d - Native base marker name is %s",
       updateCameraPoseFromAruco, baseMarker.shapes[0].markerShapeName.c_str());

  // Transform from camera to marker.
  HTr A_MC;
  HTr_copy(&A_MC, &baseMarker.shapes[0].T_MC);

  // Relative transformation of shape to body frame in case the shape marker is offset
  const RcsBody* baseMarkerBody = RcsGraph_getBodyByName(graph, baseMarkerBdyName.c_str());
  RCHECK(baseMarkerBody);
  bool shapeTrfApplied = false;

  RCSBODY_TRAVERSE_SHAPES(baseMarkerBody)
  {
    if (std::string(SHAPE->material)==baseMarker.shapes[0].markerShapeName)
    {
      HTr A_BC;
      HTr_transpose(&A_BC, &SHAPE->A_CB);
      RLOG(1, "Scaling with %f", SHAPE->extents[0]/default_marker_length);
      Vec3d_constMulSelf(A_MC.org, SHAPE->extents[0]/default_marker_length);
      HTr_transformSelf(&A_MC, &A_BC);
      shapeTrfApplied = true;
      break;
    }
  }

  RCHECK(shapeTrfApplied);
  HTr_transposeSelf(&A_MC);



  // Apply a simple 1st order LPF to the camera estimates to eliminate some
  // noise. In the first calibration step, we initialize the filter.
  HTr filt;
  const RcsBody* cam = RcsGraph_getBodyByName(graph, cameraBodyName.c_str());
  RCHECK(cam);
  int cam_qidx = RcsBody_getJointIndex(graph, cam);
  RCHECK(cam_qidx!=-1);
  double* q_cam = &graph->q->ele[cam_qidx];

  if (updateCameraPoseFromAruco == 0)
  {
    HTr_copy(&filt, &A_MC);
  }
  else
  {
    HTr_from6DVector(&filt, q_cam);
    HTr_firstOrderLPF(&filt, &A_MC, tmc);
  }

  HTr_to6DVector(q_cam, &filt);



  updateCameraPoseFromAruco++;

  if (updateCameraPoseFromAruco>numCalibrationSteps)
  {
    RLOG(0, "Calibration finished. Camera pose for xml: %.3f %.3f %.3f  %.3f %.3f %.3f",
         q_cam[0], q_cam[1], q_cam[2], RCS_RAD2DEG(q_cam[3]), RCS_RAD2DEG(q_cam[4]), RCS_RAD2DEG(q_cam[5]));
    updateCameraPoseFromAruco = -1;
    for (auto& cb : cameraCalibrationFinishedCb)
    {
      cb(&filt);
    }
  }

}

bool ArucoMultiCamCalibrator::isCalibrating() const
{
  if (updateCameraPoseFromAruco>=0 && updateCameraPoseFromAruco<=numCalibrationSteps)
  {
    return true;
  }

  return false;
}

void ArucoMultiCamCalibrator::startCalibration(size_t numFrames)
{
  RLOG(0, "Starting calibration");
  updateCameraPoseFromAruco = 0;
  numCalibrationSteps = numFrames;
}

void ArucoMultiCamCalibrator::stopCalibration()
{
  updateCameraPoseFromAruco = -1;
}




/*! \brief Given a hierarchy of transforms where I is the world frame, B is
 *         a body frame, P is the bodie's parent frame and M is a marker frame
 *         that is a child of the body, and knowing the transforms T_MI, T_MB
 *         and T_PI, we compute the unknown transform T_BP:
 *
 *         P_r_PB = A_PI (I_r_M - I_r_P - A_IB B_r_BM)   with A_IB = A_IM A_MB
 *
 *         A_BP = A_BM A_MI A_IP = A_BI A_IP
 */
static void getRigidBodyDofs(HTr* T_BP,
                             const HTr* T_MI,
                             const HTr* T_MB,
                             const HTr* T_PI)
{
  const double* B_r_BM = T_MB->org;
  const double* I_r_M = T_MI->org;
  const double* I_r_P = T_PI->org;

  double A_IB[3][3];
  Mat3d_transposeMul(A_IB, (double(*)[3])T_MI->rot, (double(*)[3])T_MB->rot);

  double I_r_BM[3];   // I_r_BM = A_IB B_r_BM
  Vec3d_rotate(I_r_BM, A_IB, B_r_BM);

  double I_r_PB[3];   // I_r_PB = I_r_M - I_r_P - I_r_BM
  Vec3d_sub(I_r_PB, I_r_M, I_r_P);
  Vec3d_subSelf(I_r_PB, I_r_BM);

  double P_r_PB[3];
  Vec3d_rotate(P_r_PB, (double(*)[3])T_PI->rot, I_r_PB);

  // Rotation matrices: A_BP = A_BI A_IP = A_BI A_PI^T
  double A_BI[3][3];
  Mat3d_transpose(A_BI, A_IB);

  double A_BP[3][3];
  Mat3d_mulTranspose(A_BP, A_BI, (double(*)[3])T_PI->rot);

  // Result
  Vec3d_copy(T_BP->org, P_r_PB);
  Mat3d_copy(T_BP->rot, A_BP);
}

ArucoMultiCamTracker::ArucoMultiCamTracker() : newArucoUpdate(false)
{
}

ArucoMultiCamTracker::~ArucoMultiCamTracker()
{
}

std::string ArucoMultiCamTracker::getRequestKeyword() const
{
  return "aruco";
}

// Process aruco frames. Called from control loop (100Hz or so)
void ArucoMultiCamTracker::updateGraph(RcsGraph* graph)
{
  // We have to call computeDofsFromAruco() here, since it depends on the graph.
  if (newArucoUpdate)
  {
    const double currTime = getWallclockTime();

    // Just the camera transform and the arucoMap can be written from
    // different threads. We protect them here.
    parseMtx.lock();
    auto localArucoShapes = arucoShapes;
    parseMtx.unlock();

    // Update each body data structure with the incoming aruco shapes. This loop
    // goes over the cameras that are the key of the arucoShapes data structure.
    for (const auto& a : localArucoShapes)
    {
      // This traverses the complete graph and updates all body transforms
      const RcsBody* cameraBdy = RcsGraph_getBodyByName(graph, a.first.c_str());
      const double maxAge = 0.5;
      RCHECK_MSG(cameraBdy, "%s", a.first.c_str());
      updateMarkerBodyTransforms(graph, cameraBdy, a.second, arucoBodies, currTime, maxAge);

      // Here we should eliminate all "outdated" percepts.

      // Instantiate calibrator if not existing
      auto calib_it = calibrators.find(a.first);
      if (calib_it == calibrators.end())
      {
        const RcsBody* baseMarkerBdy = RCSBODY_BY_ID(graph, cameraBdy->parentId);
        RCHECK_MSG(baseMarkerBdy, "Camera %s has no base marker (parent)", cameraBdy->name);
        calibrators[a.first] = std::make_unique<ArucoMultiCamCalibrator>(a.first, baseMarkerBdy->name);
      }

    }

    // Average all transforms
    for (auto& arucoBdy : arucoBodies)
    {
      ArucoMarkerBodyData& bData = arucoBdy.second;
      bData.q_rbj = std::vector<double>(6, 0.0);

      if (bData.A_BP.size() > 1)
      {
        std::vector<double> eulRot(3 * bData.A_BP.size(), 0.0);
        double* avgPos = bData.q_rbj.data();
        double* avgEul = bData.q_rbj.data()+3;

        for (size_t i = 0; i < bData.A_BP.size(); ++i)
        {
          Vec3d_addSelf(avgPos, bData.A_BP[i].org);
          Mat3d_toEulerAngles(eulRot.data() + 3*i, bData.A_BP[i].rot);
        }

        Math_weightedMeanEulerAngles(avgEul, (double(*)[3])eulRot.data(), NULL, bData.A_BP.size());
        Vec3d_constMulSelf(avgPos, 1.0 / bData.A_BP.size());
      }
      else if (bData.A_BP.size() == 1)
      {
        HTr_to6DVector(bData.q_rbj.data(), &bData.A_BP[0]);
      }
      else
      {
        RFATAL("Must never happen: ArucoMarkerBodyData with empty transform list");
      }

    }

    // Perform a calibration step
    for (auto& c : calibrators)
    {
      c.second->updateCalibration(graph, arucoBodies);
    }

    newArucoUpdate = false;
  }

  // Computation in each step for filtering etc.
  for (const auto& arucoBdy : arucoBodies)
  {
    std::string bdyName = arucoBdy.first;

    // Ignore the base marker
    if (arucoBdy.second.isBaseMarker)
    {
      continue;
    }

    const int qIdx = arucoBdy.second.qIdx;
    const double* q6 = arucoBdy.second.q_rbj.data();
    NLOG_CPP(2, "Body " << bdyName <<" at index " << qIdx << ": Applying pose "
             << q6[0] << " " << q6[1] << " " << q6[2] << " "
             << RCS_RAD2DEG(q6[3]) << " " << RCS_RAD2DEG(q6[4]) << " "
             << RCS_RAD2DEG(q6[5]));

    // Apply low pass filter to aruco estimates.
    const double tmc = 0.01;
    HTr raw, filt;
    HTr_from6DVector(&raw, q6);
    HTr_from6DVector(&filt, &graph->q->ele[qIdx]);
    HTr_firstOrderLPF(&filt, &raw, tmc);
    HTr_to6DVector(&graph->q->ele[qIdx], &filt);

    // That's without filtering (for reference)
    //VecNd_copy(&graph->q->ele[qIdx], q6, 6);

  }   // for (auto const& marker : markers)


}

void ArucoMultiCamTracker::calibrate(size_t numFrames)
{
  for (auto& c : calibrators)
  {
    c.second->startCalibration(numFrames);
  }
}

static std::vector<double> parsePose(const nlohmann::json& json)
{
  std::vector<double> pose
  {
    double(json["position"]["x"]),
    double(json["position"]["y"]),
    double(json["position"]["z"]),
    double(json["orientation"]["w"]),
    double(json["orientation"]["x"]),
    double(json["orientation"]["y"]),
    double(json["orientation"]["z"])
  };
  return pose;
}

void ArucoMultiCamTracker::parse(const nlohmann::json& json, double time, const std::string& cameraFrame)
{
  std::map<std::string,std::vector<ArucoMarkerShapeData>> inputData;

  for (auto& entry : json.items())
  {
    if (entry.value().size() != 1)
    {
      RLOG_CPP(0, "Cannot handle multiple detections of '" << entry.key() << "'. Skipping.");
      continue;
    }

    ArucoMarkerShapeData ams;
    ams.markerShapeName = entry.key();
    ams.lastUpdateTime = time;

    auto pose = parsePose(entry.value()[0]);
    const double* ptr = pose.data();
    const double* pos = &ptr[0];
    const double* quat = &ptr[3];   // w-x-y-z order
    Vec3d_copy(ams.T_MC.org, pos);
    Quat_toRotationMatrix(ams.T_MC.rot, quat);

    inputData[cameraFrame].push_back(ams);
  }

  std::lock_guard<std::mutex> lock(parseMtx);
  if (!inputData.empty())
  {
    newArucoUpdate = true;
  }
  arucoShapes = inputData;
}

void ArucoMultiCamTracker::setCameraTransform(const HTr* A_camI)
{
}

ArucoMultiCamTracker::ArucoMarkerBodyData
ArucoMultiCamTracker::updateMarkerBodyTransform(const RcsGraph* graph,
                                                const RcsBody* body,
                                                const RcsBody* camera,
                                                const std::vector<ArucoMultiCamTracker::ArucoMarkerShapeData>& arucoShapes)
{
  std::vector<ArucoMultiCamTracker::ArucoMarkerShapeData> bdyShapes;
  std::vector<HTr> bdyA_BP;
  double t_mostRecent = 0.0;
  const RcsBody* baseMarker = RCSBODY_BY_ID(graph, camera->parentId);
  RCHECK(baseMarker);

  RCSBODY_TRAVERSE_SHAPES(body)
  {
    if (!RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_MARKER))
    {
      continue;
    }

    for (const auto& aruco : arucoShapes)
    {
      // Don't do anything if shape's marker name (in RcsShape::material) is
      // not the aruco marker name
      if (std::string(SHAPE->material) != aruco.markerShapeName)
      {
        continue;
      }

      // From here on, the RcsShape corresponds to the aruco marker shape.

      // Compute marker in camera coordinates. We divide it by the marker
      // extents so that different-sized markers are handled correctly.
      HTr T_MC = aruco.T_MC;
      Vec3d_constMulSelf(T_MC.org, SHAPE->extents[0]/default_marker_length);

      HTr T_MI;   // Marker in world coordinates
      const HTr* T_CI = &camera->A_BI;
      HTr_transform(&T_MI, T_CI, &T_MC);

      // Body -> marker shape (static offset from xml)
      const HTr* T_MB = &SHAPE->A_CB;

      // World transform of bodie's parent body.
      const HTr* T_PI = (body->parentId == -1) ? HTr_identity() : &graph->bodies[body->parentId].A_BI;

      HTr T_BP;   // That's what we need: The transformation of the body to the parent.
      getRigidBodyDofs(&T_BP, &T_MI, T_MB, T_PI);
      bdyShapes.push_back(aruco);
      bdyA_BP.push_back(T_BP);

      // Update latest update time
      t_mostRecent = std::max(aruco.lastUpdateTime, t_mostRecent);
    }

  }   // RCSBODY_TRAVERSE_SHAPES(body)

  ArucoMultiCamTracker::ArucoMarkerBodyData bdyData;
  bdyData.shapes = bdyShapes;
  bdyData.A_BP = bdyA_BP;
  bdyData.lastUpdateTime = t_mostRecent;
  bdyData.isBaseMarker = (baseMarker->id==body->id) ? true : false;
  bdyData.qIdx = RcsBody_getJointIndex(graph, body);

  return bdyData;
}

void ArucoMultiCamTracker::updateMarkerBodyTransforms(const RcsGraph* graph,
                                                      const RcsBody* cameraBdy,
                                                      const std::vector<ArucoMultiCamTracker::ArucoMarkerShapeData>& arucoShapes,
                                                      std::map<std::string,ArucoMultiCamTracker::ArucoMarkerBodyData>& arucoBodies,
                                                      double currTime, double maxAge) // Everything older than this is erased from the data
{

  RCSGRAPH_FOREACH_BODY(graph)
  {
    if (BODY->id == -1)
    {
      continue;
    }

    auto bdyData = updateMarkerBodyTransform(graph, cameraBdy, BODY, arucoShapes);

    if (!bdyData.shapes.empty())
    {
      // This overwrites everything
      // arucoBodies[BODY->name] = bdyData;

      // But we should just append it
      auto bdydat_it = arucoBodies.find(BODY->name);
      if (bdydat_it == arucoBodies.end())
      {
        arucoBodies[BODY->name] = bdyData;
      }
      else
      {
        bdydat_it->second.shapes.insert(bdydat_it->second.shapes.end(), bdyData.shapes.begin(), bdyData.shapes.end());
        bdydat_it->second.A_BP.insert(bdydat_it->second.A_BP.end(), bdyData.A_BP.begin(), bdyData.A_BP.end());
        bdydat_it->second.lastUpdateTime = std::max(bdydat_it->second.lastUpdateTime, bdyData.lastUpdateTime);
        RCHECK(bdydat_it->second.qIdx==bdyData.qIdx);
        RCHECK(bdydat_it->second.isBaseMarker==bdyData.isBaseMarker);
      }


      // Here we delete all old shapes and update the lastTimeSeen
      bdydat_it = arucoBodies.find(BODY->name);
      auto shp_it = bdydat_it->second.shapes.begin();
      auto trf_it = bdydat_it->second.A_BP.begin();
      while (shp_it != bdydat_it->second.shapes.end())
      {
        // If shape is old then delete it
        if (shp_it->lastUpdateTime<currTime-maxAge)
        {
          // Reset the iterator to next item.
          shp_it = bdydat_it->second.shapes.erase(shp_it);
          trf_it = bdydat_it->second.A_BP.erase(trf_it);
        }
        else
        {
          shp_it++;
          trf_it++;
        }
      }

      // Here we update the most recen percept time since this might have
      // changed after deletion of old percepts.
      bdydat_it->second.lastUpdateTime = 0.0;
      for (const auto& sh : bdydat_it->second.shapes)
      {
        bdydat_it->second.lastUpdateTime = std::max(bdydat_it->second.lastUpdateTime, sh.lastUpdateTime);
      }
    }

  }   // RCSGRAPH_FOREACH_BODY(graph)

}

}   // namespace
