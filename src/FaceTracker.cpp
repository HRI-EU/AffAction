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

#include "FaceTracker.h"
#include "SceneHelpers.h"

#include <Rcs_graphicsUtils.h>
#include <BodyNode.h>

#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_geometry.h>
#include <Rcs_resourcePath.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>

#include <string>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <fstream>

#define DISTANCE_FACE_TO_CAM         (0.0)
//#define FACEMESH_SIMPLE_NUM_VERTICES (468)
#define FACEMESH_IRIS_NUM_VERTICES   (478)
#define DEFAULT_MAX_AGE (2.0)

static const size_t foreheadTop      = 3 * 10;
static const size_t chinCenter       = 3 * 152;



namespace aff
{

static void lpFiltTrf(double filtVec[6], const HTr* raw, double tmc)
{
  HTr filt;
  HTr_from6DVector(&filt, filtVec);
  HTr_firstOrderLPF(&filt, raw, tmc);
  HTr_to6DVector(filtVec, &filt);
  // HTr_to6DVector(filtVec, raw);   // Uncomment this for no filtering
}

// In case the iris is estimated, there are 10 more landmarks
FaceTracker::FaceTracker(const std::string& nameOfFaceBody, const std::string& nameOfCamera, const std::string& nameOfAgent) :
  TrackerBase(nameOfFaceBody), initialFaceHeight(0.0), newFaceUpdate(false), wasVisible(false), isVisible(false),
  lastUpdateTime(0.0), mesh(nullptr), landmarks(nullptr), viewer(nullptr),
  faceName(nameOfFaceBody), cameraName(nameOfCamera), agentName(nameOfAgent)
{
  std::vector<std::string> candidates =
  {
    "hri_scitos_description/FaceMesh-holes-478.obj",
    "hri_description/meshes/FaceMesh-holes-478.obj",
    "meshes/hri_description/meshes/FaceMesh-holes-478.obj"
  };

  std::string meshFile = Rcs::getAbsoluteFileName(candidates);
  this->mesh = RcsMesh_createFromFile(meshFile.c_str());
  if (!this->mesh)
  {
    Rcs_printResourcePath();
  }

  RCHECK(this->mesh);
  RCHECK(mesh->nVertices == FACEMESH_IRIS_NUM_VERTICES);
  RLOG(5, "Face mesh has %d vertices and %d faces",
       mesh->nVertices, mesh->nFaces);
  this->landmarks = MatNd_create(FACEMESH_IRIS_NUM_VERTICES, 3);

  // For face size normalization
  this->initialFaceHeight = Vec3d_distance(&mesh->vertices[foreheadTop], &mesh->vertices[chinCenter]);
  RLOG(5, "Initial face height is %f", initialFaceHeight);

  HTr_setIdentity(&this->faceTrf);
}

FaceTracker::~FaceTracker()
{
  RcsMesh_destroy(this->mesh);
  MatNd_destroy(this->landmarks);
}

std::string FaceTracker::getRequestKeyword() const
{
  return "mediapipe";
}

// Does not depend on this->mesh
void FaceTracker::parse(const nlohmann::json& jsonHeader, const nlohmann::json& jsonData, double time)
{
  std::string json_camera_name = jsonHeader.value("frame_id", "None");
  if (this->cameraName != json_camera_name)
  {
    //RLOG_CPP(1, "Wrong cam: expecting '" << this->cameraName << "' but received '" << json_camera_name << "'");
    return;
  }

  std::lock_guard<std::mutex> lock(landmarksMtx);
  newFaceUpdate = true;
  lastUpdateTime = getWallclockTime();

  NLOG_CPP(1, "FaceTracker::parse" << jsonData.dump());
  size_t nFaceLandmarks = 0;
  for (auto& entry : jsonData.items())
  {
    const std::string& key = entry.key();
    //RLOG_CPP(1, "json[" << entry.key() << "]" << nlohmann::to_string(entry.value()));

    if (STRNEQ(key.c_str(), "face", 4))
    {
      nFaceLandmarks++;
      auto keyStrings = Rcs::String_split(key, "_");
      RCHECK(keyStrings.size()==3);
      //size_t faceId = atoi(keyStrings[1].c_str());
      size_t landmarkIdx = atoi(keyStrings[2].c_str());

      if (landmarkIdx<landmarks->m)
      {
        double* rowPtr = MatNd_getRowPtr(this->landmarks, landmarkIdx);
        const std::vector<double>& dataPoint = entry.value();

        // We center it by substracting 0.5 from the x and y ranges [0...1]
        rowPtr[0] = dataPoint[0]-0.5;
        rowPtr[1] = dataPoint[1]-0.5;
        rowPtr[2] = dataPoint[2];
      }

    }
  }

  // We assume that the mesh vertices are contained within the landmarks
  // from the beginning. There might be more landmarks than mesh vertices
  // in case we estimate the iris parameters.
  this->faceTrf = estimateFaceTransform(landmarks);

  // Update debug graphics. Locking the viewer mutex might lead to waiting
  // the whole step() cycle. In parse(), it is acceptable, since it is a slow
  // and non-deterministic loop.
  if (sw.valid() && sw->isVisible())
  {
    HTr C_leftIris, C_rightIris;
    estimateIrisTransform(landmarks, &faceTrf, &C_leftIris, &C_rightIris);

    //viewer->lock();
    faceMeshNode->update(this->mesh);
    faceFrameNode->setTransformation(&faceTrf);
    leftIrisNode->setOrigin(C_leftIris.org);
    leftIrisNode->setDirection(Vec3d_ex());
    leftIrisNode->setArrowLength(0.3);
    rightIrisNode->setOrigin(C_rightIris.org);
    rightIrisNode->setDirection(Vec3d_ex());
    rightIrisNode->setArrowLength(0.3);
    //viewer->unlock();
  }

}

void FaceTracker::update(ActionScene* scene, RcsGraph* graph)
{
  const double age = getWallclockTime() - lastUpdateTime;

  this->wasVisible = isVisible;
  this->isVisible = (age <= DEFAULT_MAX_AGE) ? true : false;

  if ((!wasVisible) && isVisible)
  {
    for (const auto& cb : agentAppearDisappearCb)
    {
      cb(agentName, true);
    }

  }
  else if (wasVisible && (!isVisible))
  {
    for (const auto& cb : agentAppearDisappearCb)
    {
      cb(agentName, false);
    }
  }

  std::lock_guard<std::mutex> lock(landmarksMtx);
  if (!newFaceUpdate)
  {
    return;
  }

  newFaceUpdate = false;

  const double currentFaceHeight = Vec3d_distance(&this->landmarks->ele[foreheadTop], &this->landmarks->ele[chinCenter]);
  const double faceScaling = this->initialFaceHeight/currentFaceHeight;

  RcsBody* faceBdy = RcsGraph_getBodyByName(graph, faceName.c_str());
  RCHECK_MSG(faceBdy, "Face body with name '%s' not found - please make sure it exists in the xml file.", faceName.c_str());

  for (unsigned int i = 0; i < faceBdy->nShapes; ++i)
  {
    RcsShape* sh = &faceBdy->shapes[i];
    if ((sh->type!=RCSSHAPE_MESH) || (sh->mesh->nVertices!=landmarks->m))
    {
      continue;
    }

    if (!RcsShape_isOfComputeType(sh, RCSSHAPE_COMPUTE_RESIZEABLE))
    {
      RLOG(1, "Face mesh not resizeable - skipping vertices updating.");
      continue;
    }

    // Transform camera vertices into face frame
    for (unsigned int i = 0; i < landmarks->m; ++i)
    {
      double* dst = &sh->mesh->vertices[3 * i];
      Vec3d_invTransform(dst, &this->faceTrf, MatNd_getRowPtr(landmarks, i));
      Vec3d_constMulSelf(dst, faceScaling);
    }

  }

  // That's the debug mesh, we transform it to world coordinates to test
  if (sw.valid() && sw->isVisible())
  {
    HTr A_CI = getCameraTransform(graph);
    for (unsigned int i = 0; i < mesh->nVertices; ++i)
    {
      double* dst = &mesh->vertices[3 * i];
      Vec3d_transform(dst, &A_CI, MatNd_getRowPtr(landmarks, i));
      dst[0] += DISTANCE_FACE_TO_CAM;
    }
  }

}

bool FaceTracker::addGraphics(Rcs::Viewer* viewer_, const HTr* cameraFrame)
{
  if (!viewer_)
  {
    RLOG(1, "Can't add graphics - passed viewer is NULL");
    return false;
  }

  if (viewer)
  {
    RLOG(1, "Graphics has already been added - skipping");
    return false;
  }

  if (!cameraFrame)
  {
    RLOG(1, "Camera frame is NULL - skipping");
    return false;
  }

  RLOG(5, "FaceTracker: Adding debug graphics");
  viewer = viewer_;


  // Hide graphics from the beginning
  this->sw = new Rcs::NodeBase();
  sw->hide();
  HTr A_cam;
  HTr_copy(&A_cam, cameraFrame);
  A_cam.org[0] += DISTANCE_FACE_TO_CAM;
  sw->setTransformation(&A_cam);

  this->landmarksNode = new Rcs::VertexArrayNode(this->landmarks, osg::PrimitiveSet::POINTS, "RED");
  landmarksNode->setPointSize(3.0);
  sw->addChild(landmarksNode.get());

  this->faceFrameNode = new Rcs::COSNode(0.4);
  sw->addChild(faceFrameNode.get());

  this->leftIrisNode = new Rcs::ArrowNode();
  this->rightIrisNode = new Rcs::ArrowNode();
  sw->addChild(leftIrisNode.get());
  sw->addChild(rightIrisNode.get());

  viewer->add(sw.get());

  // We show the debug face in world coordinates
  // Fair skin tone #FFDBAC, light skin tone: #EFD0B9, medium-light: #E0C19F
  this->faceMeshNode = new Rcs::MeshNode(this->mesh);
  Rcs::setNodeMaterial("#EFD0B9", faceMeshNode);
  faceMeshNode->clearMesh();
  faceMeshNode->makeDynamic();
  viewer->add(faceMeshNode.get());

  RLOG(1, "Added debug graphics");

  return true;
}

// This method estimates the frame of reference for the face landmarks.
// x points forward, z points upward, y points from right eye to left eye.
// The frame center lies between the eyes. The face transform is represented
// in the frame of the Mediapipe camera.
/*static*/ HTr FaceTracker::estimateFaceTransform(const MatNd* faceLandMarks)
{
  HTr A_FC;   // Camera -> Face
  HTr_setIdentity(&A_FC);

  // Face only: 468, face + iris: 478
  if (faceLandMarks->m < FACEMESH_IRIS_NUM_VERTICES)
  {
    NLOG(0, "Wrong number of face landmarks: %d - should be >= %d",
         faceLandMarks->m, FACEMESH_IRIS_NUM_VERTICES);
    return A_FC;
  }

  const double* lm6 = MatNd_getRowPtr(faceLandMarks, 6);      // Between the eyes
  const double* lm10 = MatNd_getRowPtr(faceLandMarks, 10);    // Top center of forehead
  const double* lm33 = MatNd_getRowPtr(faceLandMarks, 33);    // Right eye
  const double* lm152 = MatNd_getRowPtr(faceLandMarks, 152);  // Chin center
  const double* lm263 = MatNd_getRowPtr(faceLandMarks, 263);  // Left eye

  // z-axis is 152 - 10
  Vec3d_sub(A_FC.rot[2], lm10, lm152);
  Vec3d_normalizeSelf(A_FC.rot[2]);

  // TODO: Are these orthogonal?
  // y-axis is 33 - 263
  Vec3d_sub(A_FC.rot[1], lm263, lm33);
  Vec3d_normalizeSelf(A_FC.rot[1]);

  // x-axis is right-handed
  Vec3d_crossProduct(A_FC.rot[0], A_FC.rot[1], A_FC.rot[2]);

  // Origin is landmark 6
  Vec3d_copy(A_FC.org, lm6);
  return A_FC;
}



bool FaceTracker::estimateIrisTransform(const MatNd* faceLandMarks, const HTr* A_FC, HTr* C_leftIris, HTr* C_rightIris)
{
  if (faceLandMarks->m < FACEMESH_IRIS_NUM_VERTICES)
  {
    NLOG(0, "Wrong number of face landmarks: %d - should be >= %d",
         faceLandMarks->m, FACEMESH_IRIS_NUM_VERTICES);
    return false;
  }

  std::vector<int> left_eye_indices {362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398};
  std::vector<int> right_eye_indices {33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246};
  std::vector<int> left_iris_indices {474, 475, 476, 477};
  std::vector<int> right_iris_indices {469, 470, 471, 472};
  double left_eye_center[3] = {0.0, 0.0, 0.0}, right_eye_center[3] = {0.0, 0.0, 0.0};
  double left_iris_center[3] = {0.0, 0.0, 0.0}, right_iris_center[3] = {0.0, 0.0, 0.0};

  for (size_t i=0; i<left_eye_indices.size(); ++i)
  {
    Vec3d_addSelf(left_eye_center, MatNd_getRowPtr(faceLandMarks, left_eye_indices[i]));
  }
  Vec3d_constMulSelf(left_eye_center, 1.0/left_eye_indices.size());

  for (size_t i=0; i<right_eye_indices.size(); ++i)
  {
    Vec3d_addSelf(right_eye_center, MatNd_getRowPtr(faceLandMarks, right_eye_indices[i]));
  }
  Vec3d_constMulSelf(right_eye_center, 1.0/right_eye_indices.size());

  for (size_t i=0; i<left_iris_indices.size(); ++i)
  {
    Vec3d_addSelf(left_iris_center, MatNd_getRowPtr(faceLandMarks, left_iris_indices[i]));
  }
  Vec3d_constMulSelf(left_iris_center, 1.0/left_iris_indices.size());

  for (size_t i=0; i<right_iris_indices.size(); ++i)
  {
    Vec3d_addSelf(right_iris_center, MatNd_getRowPtr(faceLandMarks, right_iris_indices[i]));
  }
  Vec3d_constMulSelf(right_iris_center, 1.0/right_iris_indices.size());

  // Transform into face frame
  double F_left_eye_center[3], F_right_eye_center[3];
  double F_left_iris_center[3], F_right_iris_center[3];
  Vec3d_invTransform(F_left_eye_center, A_FC, left_eye_center);
  Vec3d_invTransform(F_right_eye_center, A_FC, right_eye_center);
  Vec3d_invTransform(F_left_iris_center, A_FC, left_iris_center);
  Vec3d_invTransform(F_right_iris_center, A_FC, right_iris_center);

  HTr_setIdentity(C_leftIris);
  HTr_setIdentity(C_rightIris);
  Vec3d_copy(C_leftIris->org, left_iris_center);
  Vec3d_copy(C_rightIris->org, right_iris_center);

  return true;
}

/*static*/ const std::string& FaceTracker::getFaceMeshDebugString(const std::string& fileName)
{
  static std::string debugStr;

  if (debugStr.empty())
  {
    RLOG(0, "Reading face mesh from file");
    std::ifstream t(fileName);
    std::stringstream buffer;
    buffer << t.rdbuf();
    debugStr = std::move(buffer.str());
    RLOG_CPP(0, debugStr);
  }

  return debugStr;
}

void FaceTracker::enableDebugGraphics(bool enable)
{
  if (!sw.valid())
  {
    RLOG(0, "Debug graphics not created - skipping enableDebugGraphics");
    return;
  }

  RLOG(0, "Setting FaceTracker visibility to %s", enable ? "TRUE" : "FALSE");
  if (enable)
  {
    sw->show();
  }
  else
  {
    sw->hide();
  }
}

bool FaceTracker::initDebugGraphics(Rcs::Viewer* viewer, const RcsGraph* graph)
{
  if (!viewer)
  {
    return false;
  }

  HTr A_CI = getCameraTransform(graph);
  bool success = addGraphics(viewer, &A_CI);

  if (!success)
  {
    RLOG_CPP(1, "Couldn't add debug graphics with camera '" << getCameraName() <<"'");
  }

  return success;
}

void FaceTracker::registerAgentAppearDisappearCallback(std::function<void(std::string agentName, bool appear)> callback)
{
  agentAppearDisappearCb.push_back(callback);
}

std::string FaceTracker::findFaceOfAgent(const ActionScene* scene,
                                         const RcsGraph* graph,
                                         const std::string& agentName)
{
  const Agent* a = scene->getAgent(agentName);
  if (!dynamic_cast<const HumanAgent*>(a))
  {
    RLOG_CPP(0, "Couldn't find HumanAgent with name '" << agentName
             << "' - skipping face tracker");
    return std::string();
  }

  RCSBODY_TRAVERSE_BODIES(graph, (RcsBody*)a->body(graph))
  {
    for (unsigned int i = 0; i < BODY->nShapes; ++i)
    {
      RcsShape* sh = &BODY->shapes[i];
      if ((sh->type == RCSSHAPE_MESH) && (sh->mesh) &&
          (sh->mesh->nVertices == FACEMESH_IRIS_NUM_VERTICES))
      {
        return std::string(BODY->name);
      }
    }
  }

  return std::string();
}

void FaceTracker::onRenameAgent(std::string from, std::string to)
{
  if (from == this->agentName)
  {
    this->agentName = to;
  }

}




}   // namespace aff
