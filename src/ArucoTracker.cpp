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

#include "ArucoTracker.h"

#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_quaternion.h>

#include <tuple>


static const double default_marker_length = 1.0;


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

static size_t computeBodyDofsFromAruco(double q_rbj[6],
                                       const RcsGraph* graph,
                                       const RcsBody* body,
                                       const HTr* T_camI,
                                       const std::map<std::string, std::vector<double>>& markerMap,
                                       std::vector<std::string>& MARKER_NAMES)
{
  // We allow several marker shapes per body and average them after their
  // collection.
  std::vector<HTr> T_BP_array;
  const double t_now = Timer_getSystemTime();
  double t_latest = 0.0;

  RCSBODY_TRAVERSE_SHAPES(body)
  {
    if (!RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_MARKER))
    {
      continue;
    }

    // From here, we have a shape with computeType marker. We look through
    // the currently received markers to find if we have a matching
    // perception. There is at maximum one match, since the std::map key is
    // guaranteed to be unique.
    // \todo: Use find for this
    const HTr* T_MC = NULL;   // Marker in camera coordinates
    HTr T_MC_buf;

    for (auto const& marker : markerMap)
    {
      // The marker name is stored in the material string.
      if (std::string(SHAPE->material) == marker.first)
      {
        MARKER_NAMES.push_back(marker.first);

        // The marker.second.data() has actually 13 elements, the last one
        // being the update time.
        T_MC = &T_MC_buf;
        HTr_fromVector(&T_MC_buf, marker.second.data());// In Camera coords
        t_latest = std::max(t_latest, marker.second[12]);

        // Divide by marker extents so that different-sized markers are handled correctly
        Vec3d_constMulSelf(T_MC_buf.org, SHAPE->extents[0]/default_marker_length);

        break;
      }
    }

    if (T_MC)   // We found one marker
    {
      HTr T_MI;   // Marker in world coordinates
      HTr_transform(&T_MI, T_camI, T_MC);

      // Body -> marker shape (static offset from xml)
      const HTr* T_MB = &SHAPE->A_CB;

      // World transform of bodie's parent body.
      const HTr* T_PI = (body->parentId == -1) ? HTr_identity() : &graph->bodies[body->parentId].A_BI;

      HTr T_BP;   // That's what we need
      getRigidBodyDofs(&T_BP, &T_MI, T_MB, T_PI);
      T_BP_array.push_back(T_BP);
    }
  }   // RCSBODY_TRAVERSE_SHAPES(BODY)

  // Here we average the estimates and apply it to the bodie's rigid body joints
  if (T_BP_array.size() == 1)
  {
    HTr_to6DVector(q_rbj, &T_BP_array[0]);
  }
  else if (T_BP_array.size() > 1)
  {
    RLOG(1, "Averaging pose of body %s with %zu estimates", body->name, T_BP_array.size());
    // Centroid for frame origins
    double avgPos[3];
    Vec3d_setZero(avgPos);
    for (size_t i = 0; i < T_BP_array.size(); ++i)
    {
      RLOG(1, "pos %zu: %.3f %.3f %.3f", i, T_BP_array[i].org[0], T_BP_array[i].org[1], T_BP_array[i].org[2]);
      Vec3d_addSelf(avgPos, T_BP_array[i].org);
    }
    Vec3d_constMulSelf(avgPos, 1.0 / T_BP_array.size());

    // Rotation average for frame orientations
    std::vector<double> eulRot(3 * T_BP_array.size(), 0.0);
    for (size_t i = 0; i < T_BP_array.size(); ++i)
    {
      Mat3d_toEulerAngles(eulRot.data() + 3 * i, T_BP_array[i].rot);
    }
    double avgEul[3];
    Math_weightedMeanEulerAngles(avgEul, (double(*)[3])eulRot.data(),
                                 NULL, T_BP_array.size());

    // Apply to body joints
    Vec3d_copy(q_rbj, avgPos);
    Vec3d_copy(q_rbj + 3, avgEul);
  }
  else
  {
    // T_BP_array.empty(): Nothing to do
  }

  // Compute confidence based on time difference
  double conf = 1.0/(1.0 + t_now - t_latest);
  RcsBody* hack = (RcsBody*)body;
  hack->confidence = conf;
  return T_BP_array.size();
}

static std::vector<std::tuple<std::string,std::vector<std::string>,int,std::vector<double>>>
computeDofsFromAruco(const RcsGraph* graph,
                     const HTr* T_camI,
                     const std::map<std::string, std::vector<double>>& markerMap)
{
  // Tuple: 1. body name, 2. q-index, 3. vector with 6 joint values
  std::vector<std::tuple<std::string,std::vector<std::string>,int,std::vector<double>>> res;

  RCSGRAPH_FOREACH_BODY(graph)
  {
    if (BODY->id == -1)
    {
      continue;
    }

    // Apply to body joints
    double q_rbj[6];
    std::vector<std::string> MARKER_NAMES;
    size_t nMarkers = computeBodyDofsFromAruco(q_rbj, graph, BODY, T_camI, markerMap, MARKER_NAMES);

    if (nMarkers>0 && !RcsBody_isArticulated(graph, BODY))
    {
      RCHECK_MSG(BODY->rigid_body_joints, "Body \"%s\" is expected to have rigid body joints",
                 BODY->name);
      const RcsJoint* jnt = RCSJOINT_BY_ID(graph, BODY->jntId);
      RCHECK_MSG(jnt, "Body \"%s\" has no joints", BODY->name);
      res.push_back(std::tuple<std::string,std::vector<std::string>,int,std::vector<double>>(std::string(BODY->name), MARKER_NAMES, jnt->jointIndex, std::vector<double>(q_rbj,q_rbj+6)));
    }
  }

  return res;
}

static std::map<std::string,ArucoTracker::ArucoMarkerData>
computeDofsFromAruco_new(const RcsGraph* graph,
                         const HTr* T_camI,
                         const std::map<std::string, std::vector<double>>& arucoMap)
{
  // Tuple: 1. body name, 2. q-index, 3. vector with 6 joint values
  std::map<std::string,ArucoTracker::ArucoMarkerData> res;

  RCSGRAPH_FOREACH_BODY(graph)
  {
    if (BODY->id == -1)
    {
      continue;
    }


    // Apply to body joints
    double q_rbj[6];
    std::vector<std::string> MARKER_NAMES;
    size_t nMarkers = computeBodyDofsFromAruco(q_rbj, graph, BODY, T_camI, arucoMap, MARKER_NAMES);

    if (nMarkers>0)
    {
      RCHECK_MSG(BODY->rigid_body_joints, "Body \"%s\" is expected to have rigid body joints",
                 BODY->name);
      const RcsJoint* jnt = RCSJOINT_BY_ID(graph, BODY->jntId);
      RCHECK_MSG(jnt, "Body \"%s\" has no joints", BODY->name);

      ArucoTracker::ArucoMarkerData arucoData;
      arucoData.markerShapes = MARKER_NAMES;
      arucoData.qIdx = jnt->jointIndex;
      arucoData.q_rbj = std::vector<double>(q_rbj,q_rbj+6);

      // This is wrong. We need to determine the last time from the latest one of all shapes in
      // computeBodyDofsFromAruco. Also, this function should not return a new structure, but update the old
      // one in case new data came in, so that we can persistently handle time.
      auto it = arucoMap.find(BODY->name);
      RCHECK(it != arucoMap.end());
      arucoData.lastUpdateTime = it->second[12];

      res[BODY->name] = arucoData;
    }
  }

  return res;
}


/*******************************************************************************
  We require the camera to be a child of the base marker, for instane:

  <Body name="aruco_base" rigid_body_joints="0 0 0.8 0 0 0" color="PEWTER">
    <Shape type="BOX" extents="0.14 0.14 0.001" graphics="true" textureFile="aruco/4x4marker_0.jpg"/>
    <Shape type="FRAME" marker="true" markerName="aruco_10" extents="0.14 0.14 0" scale="0.2" transform="-0.44 -1.44 0 0 0 0"/>
    </Body>

   Body name="camera" prev="aruco_base" rigid_body_joints="0 0.5 0.48 121 -6 177">
    <Shape type="FRAME" scale="0.2" graphics="true"/>
  </Body>

 *******************************************************************************/
class ArucoCalibrator
{
public:

  ArucoCalibrator(const std::string& cameraBodyName, const std::string& baseMarkerBdyName);

  virtual ~ArucoCalibrator();

  // Process aruco frames. Called from control loop (100Hz or so)
  void updateCalibration(RcsGraph* graph,
                         const std::map<std::string,std::vector<double>>& arucoMap,
                         const std::tuple<std::string,std::vector<std::string>,int,std::vector<double>>& marker);

  void updateCameraTransform(RcsGraph* graph);

  void startCalibration(size_t numFrames);
  void stopCalibration();
  void registerCallback(std::function<void(const HTr*)> callback);
  std::string getBaseMarkerName() const;
  std::string getCameraName() const;
  bool isCalibrating() const;

private:
  std::string cameraBodyName;
  std::string baseMarkerBdyName;
  HTr A_ArucoCam;   // Offeset transform between Aruco base marker and camera
  int updateCameraPoseFromAruco;
  double tmc;
  size_t numCalibrationSteps;
  std::vector<std::function<void(const HTr*)>> cameraCalibrationFinishedCb;
};

ArucoCalibrator::ArucoCalibrator(const std::string& cameraBodyName_, const std::string& baseMarkerBdyName_) : cameraBodyName(cameraBodyName_), baseMarkerBdyName(baseMarkerBdyName_), updateCameraPoseFromAruco(-1), tmc(0.05), numCalibrationSteps(20)
{
  HTr_setIdentity(&A_ArucoCam);
}

ArucoCalibrator::~ArucoCalibrator()
{
}

std::string ArucoCalibrator::getBaseMarkerName() const
{
  return baseMarkerBdyName;
}

std::string ArucoCalibrator::getCameraName() const
{
  return cameraBodyName;
}

void ArucoCalibrator::registerCallback(std::function<void(const HTr*)> callback)
{
  cameraCalibrationFinishedCb.push_back(callback);
}

// Process aruco frames. Called from control loop (100Hz or so)
void ArucoCalibrator::updateCalibration(RcsGraph* graph,
                                        const std::map<std::string,std::vector<double>>& arucoMap,
                                        const std::tuple<std::string,std::vector<std::string>,int,std::vector<double>>& marker)
{
  // marker: 1. body name, 2. marker shapes 3. q-index, 4. vector with 6 joint values
  std::string bdyName = std::get<0>(marker);

  NLOG(1, "updateCameraPoseFromAruco = %d", updateCameraPoseFromAruco);
  NLOG_CPP(1, "bdyName: " << bdyName << " baseMarkerBdyName " << baseMarkerBdyName);

  if ((updateCameraPoseFromAruco<0) || (bdyName!=baseMarkerBdyName))
  {
    return;
  }

  // For the base marker body, we assume only one marker shape.
  std::vector<std::string> markerShapeNames = std::get<1>(marker);
  RCHECK(markerShapeNames.size()==1);
  RLOG(0, "Calibration step %d - Native base marker name is %s",
       updateCameraPoseFromAruco, markerShapeNames[0].c_str());

  auto it = arucoMap.find(markerShapeNames[0]);
  RCHECK(it != arucoMap.end());

  // Transform from camera to marker, from aruco processing. The
  // it->second.data() pointer has actually 13 elements, the last
  // one being the last update time.
  HTr A_MC;
  HTr_fromVector(&A_MC, it->second.data());

  // Relative transformation of shape to body frame in case the shape marker is offset
  const RcsBody* baseMarkerBody = RcsGraph_getBodyByName(graph, baseMarkerBdyName.c_str());
  RCHECK(baseMarkerBody);
  RCSBODY_TRAVERSE_SHAPES(baseMarkerBody)
  {
    if (std::string(SHAPE->material)==markerShapeNames[0])
    {
      HTr A_BC;
      HTr_transpose(&A_BC, &SHAPE->A_CB);
      RLOG(1, "Scaling with %f", SHAPE->extents[0]/default_marker_length);
      Vec3d_constMulSelf(A_MC.org, SHAPE->extents[0]/default_marker_length);
      HTr_transformSelf(&A_MC, &A_BC);
      break;
    }
  }

  HTr_transposeSelf(&A_MC);

  // Apply a simple 1st order LPF to the camera estimates to eliminate some noise.
  if (updateCameraPoseFromAruco == 0)
  {
    HTr_copy(&A_ArucoCam, &A_MC);
  }
  else
  {
    HTr_firstOrderLPF(&A_ArucoCam, &A_MC, tmc);
  }

  updateCameraPoseFromAruco++;

  if (updateCameraPoseFromAruco>numCalibrationSteps)
  {
    RLOG(0, "Calibration finished");
    double x[6];
    HTr_to6DVector(x, &A_ArucoCam);
    RLOG(0, "Camera pose for xml: %.3f %.3f %.3f  %.3f %.3f %.3f",
         x[0], x[1], x[2], RCS_RAD2DEG(x[3]), RCS_RAD2DEG(x[4]), RCS_RAD2DEG(x[5]));
    updateCameraPoseFromAruco = -1;
    for (auto& cb : cameraCalibrationFinishedCb)
    {
      cb(&A_ArucoCam);
    }
    updateCameraTransform(graph);
  }

}

bool ArucoCalibrator::isCalibrating() const
{
  if (updateCameraPoseFromAruco>=0 && updateCameraPoseFromAruco<=numCalibrationSteps)
  {
    return true;
  }

  return false;
}

void ArucoCalibrator::updateCameraTransform(RcsGraph* graph)
{
  const RcsBody* cam = RcsGraph_getBodyByName(graph, cameraBodyName.c_str());
  RCHECK(cam && cam->rigid_body_joints);
  const RcsJoint* camJnt = RCSJOINT_BY_ID(graph, cam->jntId);
  RCHECK(camJnt);
  double* q_cam = &graph->q->ele[camJnt->jointIndex];
  HTr_to6DVector(q_cam, &A_ArucoCam);

  RLOG(1, "[%d] Setting camera pose to %.3f %.3f %.3f %.3f %.3f %.3f",
       updateCameraPoseFromAruco, q_cam[0], q_cam[1], q_cam[2],
       RCS_RAD2DEG(q_cam[3]), RCS_RAD2DEG(q_cam[4]), RCS_RAD2DEG(q_cam[5]));
}

void ArucoCalibrator::startCalibration(size_t numFrames)
{
  RLOG(0, "Starting calibration");
  updateCameraPoseFromAruco = 0;
  numCalibrationSteps = numFrames;
}

void ArucoCalibrator::stopCalibration()
{
  updateCameraPoseFromAruco = -1;
}




ArucoTracker::ArucoTracker(const std::string& cameraBodyName, const std::string& baseMarkerBdyName) : newArucoUpdate(false)
{
  calibration = std::make_unique<ArucoCalibrator>(cameraBodyName, baseMarkerBdyName);
  HTr_setIdentity(&A_CI);
}

ArucoTracker::~ArucoTracker()
{
}

std::string ArucoTracker::getRequestKeyword() const
{
  return "aruco";
}

// Process aruco frames. Called from control loop (100Hz or so)
void ArucoTracker::updateGraph(RcsGraph* graph)
{
  // We have to call computeDofsFromAruco() here, since it depends on the graph.
  if (newArucoUpdate)
  {
    // Just the camera transform and the arucoMap can be written from
    // different threads. We protect them here.
    arucoMapMtx.lock();
    auto localArucoMap = arucoMap;
    HTr A_camI = this->A_CI;
    arucoMapMtx.unlock();

    // We have to call computeDofsFromAruco() here, since it depends on the graph.
    markers = computeDofsFromAruco(graph, &A_camI, localArucoMap);
    NLOG_CPP(1, "Found " << markers.size() << " bodies with markers.");

    // Calibration update happens only if new update has been received.
    for (auto const& marker : markers)
    {
      calibration->updateCalibration(graph, localArucoMap, marker);
    }
  }

  // Computation in each step for filtering etc.
  for (auto const& marker : markers)
  {
    // marker: 1. body name, 2. marker shapes 3. q-index, 4. vector with 6 joint values
    std::string bdyName = std::get<0>(marker);

    // Ignore the base marker
    if (bdyName==calibration->getBaseMarkerName())
    {
      continue;
    }

    const int qIdx = std::get<2>(marker);
    std::vector<double> q_rbj = std::get<3>(marker);
    NLOG_CPP(2, "Body " << bdyName <<" at index " << qIdx << ": Applying pose "
             << q_rbj[0] << " " << q_rbj[1] << " " << q_rbj[2] << " "
             << RCS_RAD2DEG(q_rbj[3]) << " " << RCS_RAD2DEG(q_rbj[4]) << " "
             << RCS_RAD2DEG(q_rbj[5]));

    // Apply low pass filter to aruco estimates.
    const double tmc = 0.5;
    HTr raw, filt;
    HTr_from6DVector(&raw, q_rbj.data());
    HTr_from6DVector(&filt, &graph->q->ele[qIdx]);
    HTr_firstOrderLPF(&filt, &raw, tmc);
    HTr_to6DVector(&graph->q->ele[qIdx], &filt);

    // That's without filtering (for reference)
    //VecNd_copy(&graph->q->ele[qIdx], q_rbj.data(), 6);

  }   // for (auto const& marker : markers)


  newArucoUpdate = false;
}

void ArucoTracker::calibrate(size_t numFrames)
{
  calibration->startCalibration(numFrames);
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

void ArucoTracker::parse(const nlohmann::json& json, double time, const std::string& cameraFrame)
{
  for (auto& entry : json.items())
  {
    if (entry.value().size() != 1)
    {
      RLOG_CPP(0, "Cannot handle multiple detections of '" << entry.key() << "'. Taking first.");
    }

    auto pose = parsePose(entry.value()[0]);
    RCHECK_MSG(pose.size()==7, "%zu", pose.size());

    const double* ptr = pose.data();
    const double* pos = &ptr[0];
    const double* quat = &ptr[3];   // w-x-y-z, see k4a_quaternion_t
    HTr frm_i;
    Vec3d_copy(frm_i.org, pos);
    Quat_toRotationMatrix(frm_i.rot, quat);

    std::vector<double> pose13(13);
    for (size_t i=0; i<3; ++i)
    {
      pose13[i] = pose[i];
    }

    const double* rm = (const double*) frm_i.rot;
    for (size_t i=3; i<12; ++i)
    {
      pose13[i] = rm[i-3];
    }
    pose13[12] = time;

    std::lock_guard<std::mutex> lock(arucoMapMtx);
    arucoMap[entry.key()] = pose13;
    newArucoUpdate = true;
  }
}

void ArucoTracker::setCameraTransform(const HTr* A_camI)
{
  REXEC(0)
  {
    double x[6];
    HTr_to6DVector(x, A_camI);
    RLOG(0, "ArucoTracker::Callback: Camera pose for xml: %.3f %.3f %.3f  %.3f %.3f %.3f",
         x[0], x[1], x[2], RCS_RAD2DEG(x[3]), RCS_RAD2DEG(x[4]), RCS_RAD2DEG(x[5]));
  }

  std::lock_guard<std::mutex> lock(arucoMapMtx);
  HTr_copy(&A_CI, A_camI);
}

void ArucoTracker::addCalibrationFinishedCallback(std::function<void(const HTr*)> callback)
{
  calibration->registerCallback(callback);
}

std::string ArucoTracker::getBaseMarkerName() const
{
  return calibration ? calibration->getBaseMarkerName() : std::string();
}

std::string ArucoTracker::getCameraName() const
{
  return calibration ? calibration->getCameraName() : std::string();
}

bool ArucoTracker::isCalibrating() const
{
  return calibration ? calibration->isCalibrating() : false;
}
