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



namespace aff
{
/*******************************************************************************
  MarkerBodyData class implementation
 *******************************************************************************/
ArucoTracker::MarkerBodyData::MarkerBodyData() : bodyId(-1), jointIndex(-1), t_latest(0.0), frozen(false)
{
}

bool ArucoTracker::MarkerBodyData::hasUpdate() const
{
  return q_rbj.size()==6 ? true : false;
}

void ArucoTracker::MarkerBodyData::print(const RcsGraph* graph) const
{
  std::string bdyName = graph ? std::string(RCSBODY_NAME_BY_ID(graph, bodyId)) : std::to_string(bodyId);
  std::cout << "Body " << bdyName << " at index " << jointIndex << ": "
            << "Frozen is " << (frozen ? "true" : "false");

  if (hasUpdate())
  {
    std::cout << "\n\tPose [m, deg]: "
              << q_rbj[0] << " " << q_rbj[1] << " " << q_rbj[2] << " "
              << RCS_RAD2DEG(q_rbj[3]) << " "
              << RCS_RAD2DEG(q_rbj[4]) << " "
              << RCS_RAD2DEG(q_rbj[5]);
  }

  std::cout << std::endl;
}

const RcsBody* ArucoTracker::MarkerBodyData::body(const RcsGraph* graph)
{
  return RCSBODY_BY_ID(graph, bodyId);
}

/*******************************************************************************
*! \brief Given a hierarchy of transforms where I is the world frame, B is
 *         a body frame, P is the bodie's parent frame and M is a marker frame
 *         that is a child of the body, and knowing the transforms T_MI, T_MB
 *         and T_PI, we compute the unknown transform T_BP:
 *
 *         P_r_PB = A_PI (I_r_M - I_r_P - A_IB B_r_BM)   with A_IB = A_IM A_MB
 *
 *         A_BP = A_BM A_MI A_IP = A_BI A_IP
 *******************************************************************************/
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

ArucoTracker::MarkerBodyData ArucoTracker::computeBodyDofsFromAruco(const RcsGraph* graph,
                                                                    const RcsBody* body,
                                                                    const HTr* T_camI,
                                                                    const std::map<std::string, std::vector<double>>& arucoMap)
{
  ArucoTracker::MarkerBodyData mData;
  mData.bodyId = body->id;
  mData.jointIndex = graph->joints[body->jntId].jointIndex;
  mData.frozen = true;

  // We allow several marker shapes per body and average them after their collection.
  std::vector<HTr> T_BP_array;

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

    // Search for the marker in the map
    auto it = std::find_if(arucoMap.begin(), arucoMap.end(),
                           [&](const std::pair<std::string, std::vector<double>>& marker)
    {
      return std::string(SHAPE->material) == marker.first;
    });

    if (it == arucoMap.end())  // Check if a marker was found
    {
      continue;
    }

    const auto& arucoMapItem = *it;
    mData.markerNames.push_back(arucoMapItem.first);

    // The marker.second.data() has actually 13 elements, the last one
    // being the update time.
    HTr T_MC;
    HTr_fromVector(&T_MC, arucoMapItem.second.data());  // In Camera coords
    mData.t_latest = std::max(mData.t_latest, arucoMapItem.second[12]);
    mData.frozen = false;

    // Divide by marker extents so that different-sized markers are handled correctly
    Vec3d_constMulSelf(T_MC.org, SHAPE->extents[0]/default_marker_length);

    HTr T_MI;   // Marker in world coordinates
    HTr_transform(&T_MI, T_camI, &T_MC);

    // Body -> marker shape (static offset from xml)
    const HTr* T_MB = &SHAPE->A_CB;

    // World transform of bodie's parent body.
    const HTr* T_PI = (body->parentId == -1) ? HTr_identity() : &graph->bodies[body->parentId].A_BI;

    HTr T_BP;   // That's what we need
    getRigidBodyDofs(&T_BP, &T_MI, T_MB, T_PI);
    T_BP_array.push_back(T_BP);

  }   // RCSBODY_TRAVERSE_SHAPES(BODY)



  // Here we average the estimates and apply it to the bodie's rigid body joints and assign it to
  // the rigid body joints q_rbj. In case there are no found markers, the q_rbj vector is empty.
  if (T_BP_array.size() == 1)
  {
    double q_rbj[6];
    HTr_to6DVector(q_rbj, &T_BP_array[0]);
    mData.q_rbj = std::vector<double>(q_rbj, q_rbj + 6);
  }
  else if (T_BP_array.size() > 1)
  {
    NLOG(5, "Averaging pose of body %s with %zu estimates", body->name, T_BP_array.size());

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
    double q_rbj[6];
    Vec3d_copy(q_rbj, avgPos);
    Vec3d_copy(q_rbj + 3, avgEul);
    mData.q_rbj = std::vector<double>(q_rbj, q_rbj + 6);
  }

  return mData;
}

/*******************************************************************************
  We require the camera to be a child of the base marker, for instance:

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
                         std::string bdyName,
                         std::vector<std::string> markerShapeNames);

  void updateCameraTransform(RcsGraph* graph, const HTr* A_CI);

  void startCalibration(size_t numFrames);
  void stopCalibration();
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
};

ArucoCalibrator::ArucoCalibrator(const std::string& cameraBodyName_,
                                 const std::string& baseMarkerBdyName_) :
  cameraBodyName(cameraBodyName_),
  baseMarkerBdyName(baseMarkerBdyName_),
  updateCameraPoseFromAruco(-1),
  tmc(0.05),
  numCalibrationSteps(20)
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

// Process aruco frames. Called from control loop (100Hz or so)
void ArucoCalibrator::updateCalibration(RcsGraph* graph,
                                        const std::map<std::string,std::vector<double>>& arucoMap,
                                        std::string bdyName,
                                        std::vector<std::string> markerShapeNames)
{

  if ((updateCameraPoseFromAruco<0) ||
      (bdyName!=baseMarkerBdyName) ||
      arucoMap.empty() ||
      markerShapeNames.empty())
  {
    return;
  }

  // For the base marker body, we assume only one marker shape.
  RCHECK_MSG(markerShapeNames.size()==1, "%zu markers for body %s",
             markerShapeNames.size(), bdyName.c_str());
  RLOG(0, "Calibration step %d - Native base marker name is %s",
       updateCameraPoseFromAruco, markerShapeNames[0].c_str());

  auto it = arucoMap.find(markerShapeNames[0]);
  RCHECK_MSG(it != arucoMap.end(), "%s", markerShapeNames[0].c_str());

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
      RLOG(0, "Scaling with %f", SHAPE->extents[0]/default_marker_length);
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
    updateCameraPoseFromAruco = -1;
    updateCameraTransform(graph, &A_ArucoCam);
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

void ArucoCalibrator::updateCameraTransform(RcsGraph* graph, const HTr* A_AC)
{
  RLOG(0, "Calibration finished");
  double x[6];
  HTr_to6DVector(x, A_AC);
  RLOG(0, "Camera pose for xml: %.3f %.3f %.3f  %.3f %.3f %.3f",
       x[0], x[1], x[2], RCS_RAD2DEG(x[3]), RCS_RAD2DEG(x[4]), RCS_RAD2DEG(x[5]));

  const RcsBody* cam = RcsGraph_getBodyByName(graph, cameraBodyName.c_str());
  RCHECK_MSG(cam && cam->rigid_body_joints, "%s", cameraBodyName.c_str());
  const RcsJoint* camJnt = RCSJOINT_BY_ID(graph, cam->jntId);
  RCHECK(camJnt);
  double* q_cam = &graph->q->ele[camJnt->jointIndex];
  HTr_to6DVector(q_cam, A_AC);

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




/*******************************************************************************
  ArucoTracker class implementation
 *******************************************************************************/
ArucoTracker::ArucoTracker(const std::string& camera, const std::string& baseMarkerBdyName) : TrackerBase(camera), newArucoUpdate(false)
{
  calibration = std::make_unique<ArucoCalibrator>(camera, baseMarkerBdyName);
}

ArucoTracker::~ArucoTracker()
{
}

std::string ArucoTracker::getRequestKeyword() const
{
  return "aruco";
}

// Process aruco frames. Called from control loop (100Hz or so)
void ArucoTracker::update(ActionScene* scene, RcsGraph* graph)
{
  // Just the camera transform and the arucoMap can be written from different threads. We protect them here.
  //HTr A_camI;
  bool newupdate = false;
  std::map<std::string, std::vector<double>> localArucoMap;
  if (!frozen)
  {
    std::lock_guard<std::mutex> lock(arucoMapMtx);
    localArucoMap = this->arucoMap;
    //HTr_copy(&A_camI, &this->A_CI);
    //A_camI = getCameraTransform(graph);
    newupdate = this->newArucoUpdate;
    if (this->newArucoUpdate)
    {
      this->arucoMap.clear();
      this->newArucoUpdate = false;
    }
  }

  // markers contain only updates from the previous input, not including
  // transforms of objects that are held in any hand.
  if (newupdate)
  {
    HTr A_camI = getCameraTransform(graph);

    RCSGRAPH_FOREACH_BODY(graph)
    {
      // We only add entries that correspond to valid rigid bodies that have at least one marker.
      if ((!BODY->rigid_body_joints) || (BODY->id==-1) ||
          (RcsBody_numShapesOfType(BODY, RCSSHAPE_COMPUTE_MARKER)==0))
      {
        continue;
      }

      auto markerItem = computeBodyDofsFromAruco(graph, BODY, &A_camI, localArucoMap);
      auto it = markerMap.find(BODY->name);

      // Marker body never seen before: initilize
      if (it == markerMap.end())
      {
        markerMap[BODY->name] = markerItem;
      }
      // Update marker data
      else
      {
        if (markerItem.hasUpdate())
        {
          it->second.t_latest = markerItem.t_latest;
          it->second.q_rbj = markerItem.q_rbj;
          it->second.markerNames = markerItem.markerNames;
          it->second.frozen = markerItem.frozen;
        }

        it->second.bodyId = markerItem.bodyId;
        it->second.jointIndex = markerItem.jointIndex;

        // Calibration update happens only if new update has been received.
        calibration->updateCalibration(graph, localArucoMap, it->first, it->second.markerNames);
      }
    }   // RCSGRAPH_FOREACH_BODY(graph)
  }

  // Computation in each step for filtering etc.
  for (auto& marker : markerMap)
  {
    marker.second.frozen |= this->frozen;   // This leaves it be frozen until there is an update
    const bool heldInHand = RcsBody_isArticulated(graph, marker.second.body(graph));

    // Ignore the base marker and invalid (unseen, held-in-hand ...) entries
    if ((marker.first==calibration->getBaseMarkerName()) || heldInHand || marker.second.frozen)
    {
      continue;
    }

    // Apply low pass filter on valid estimates.
    const double tmc = 0.5;
    HTr raw, filt;
    HTr_from6DVector(&raw, marker.second.q_rbj.data());
    HTr_from6DVector(&filt, &graph->q->ele[marker.second.jointIndex]);
    HTr_firstOrderLPF(&filt, &raw, tmc);
    HTr_to6DVector(&graph->q->ele[marker.second.jointIndex], &filt);
  }

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

/*

 {
     "aruco_10": [      # <-- entry.key
         {              # <-- entry.value[0]
             "id": 10,
             "orientation": {
                 "w": 0.1829875629933815,
                 "x": 0.028557308472796805,
                 "y": 0.8989100483042847,
                 "z": 0.39706517976287165
             },
             "position": {
                 "x": -5.260215610158375,
                 "y": 1.28047442505508,
                 "z": 16.066453031456977
             },
             "reprojection_error": 0.15712533543963342,
         }
     ],  # end entry (item 0)

     "aruco_3": [
         {
             "id": 3,
             "orientation": {
                 "w": 0.022150579825356786,
                 "x": 0.6953875885958379,
                 "y": 0.49476124675710376,
                 "z": 0.5207271475039713
             },
             "position": {
                 "x": -5.878074040926496,
                 "y": 1.6805983774038338,
                 "z": 13.107461503432832
             },
             "reprojection_error": 0.13311420570088922,
         }
     ]
 }

 */
void ArucoTracker::parse(const nlohmann::json& jsonHeader, const nlohmann::json& jsonData, double time)
{
  std::map<std::string,std::vector<double>> localArucoMap;
  RLOG_CPP(2, "Received 'aruco':" << jsonData.dump(2));

  for (auto& entry : jsonData.items())
  {
    if (entry.value().size() != 1)
    {
      RLOG_CPP(0, "Cannot handle multiple detections of '" << entry.key()
               << "'. Taking first.");
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

    localArucoMap[entry.key()] = pose13;
  }

  std::lock_guard<std::mutex> lock(arucoMapMtx);
  arucoMap = localArucoMap;
  newArucoUpdate = true;
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

bool ArucoTracker::initDebugGraphics(Rcs::Viewer* viewer, const RcsGraph* graph)
{
  return true;
}

}   // namespace
