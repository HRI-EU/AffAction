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

#define DISTANCE_FACE_TO_CAM (1.0)

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
FaceTracker::FaceTracker(const std::string& nameOfFaceBody) :
  scene(nullptr), mesh(NULL), landmarks(NULL), viewer(nullptr), faceName(nameOfFaceBody)
{
  std::string meshFile = Rcs::getAbsoluteFileName("FaceMesh-holes-468.obj");
  this->mesh = RcsMesh_createFromFile(meshFile.c_str());
  RCHECK(this->mesh);
  RCHECK(mesh->nVertices >= 468);
  RLOG(5, "Face mesh has %d vertices and %d faces",
       mesh->nVertices, mesh->nFaces);
  this->landmarks = MatNd_create(468, 3);

  HTr_setIdentity(&this->faceTrf);
  HTr_setIdentity(&this->A_camI);
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

void FaceTracker::setCameraName(const std::string& cameraFrame)
{
  this->cameraName = cameraFrame;
}

std::string FaceTracker::getCameraName() const
{
  return this->cameraName;
}

void FaceTracker::parse(const nlohmann::json& json, double time, const std::string& cameraFrame)
{
  std::lock_guard<std::mutex> lock(landmarksMtx);

  //RLOG_CPP(5, "parse" << json.dump());
  for (auto& entry : json.items())
  {
    const std::string& key = entry.key();
    //RLOG_CPP(1, "json[" << entry.key() << "]" << nlohmann::to_string(entry.value()));

    if (STRNEQ(key.c_str(), "face", 4))
    {
      auto keyStrings = Rcs::String_split(key, "_");
      RCHECK(keyStrings.size()==3);
      //size_t faceId = atoi(keyStrings[1].c_str());
      size_t landmarkIdx = atoi(keyStrings[2].c_str());

      if (landmarkIdx<mesh->nVertices)
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
}

void FaceTracker::updateGraph(RcsGraph* graph)
{
  std::lock_guard<std::mutex> lock(landmarksMtx);

  // The landmarks array is in camera coordinates. We transform it into the
  // face frame to display this coherently
  HTr* A_FC = &this->faceTrf;   // Camera -> Face

  // World -> face
  HTr A_FI;
  HTr_transform(&A_FI, &this->A_camI, A_FC);

  // Amplify the face rotations
  double tmp6[6];
  HTr_to6DVector(tmp6, &A_FI);
  tmp6[5] = Math_fmodAngle(tmp6[5]+M_PI);
  tmp6[3] *= 2.0;
  tmp6[4] *= 4.0;
  tmp6[5] *= 2.0;
  tmp6[5] += M_PI;
  HTr_from6DVector(&A_FI, tmp6);

  // Shift the face away from the camera. \todo: Improve this.
  //A_FI.org[0] += 1.75 * DISTANCE_FACE_TO_CAM;
  A_FI.org[0] += 0.95 * DISTANCE_FACE_TO_CAM;

  // Look down
  Mat3d_rotateSelfAboutXYZAxis(A_FI.rot, 1, RCS_DEG2RAD(10.0));

  double* q6 = RcsBody_getStatePtr(graph, RcsGraph_getBodyByName(graph, faceName.c_str()));
  RCHECK_MSG(q6, "Body with name '%s' and six rigid body joints not found - please make sure it exists in the xml file.", faceName.c_str());
  lpFiltTrf(q6, &A_FI, 0.05);
  //RLOG(1, "ea: %f %f %f", q6[3], q6[4], q6[5]);

  for (unsigned int i=0; i<mesh->nVertices; ++i)
  {
    double* dst = &mesh->vertices[3*i];
    Vec3d_transform(dst, &this->A_camI, MatNd_getRowPtr(landmarks, i));
    dst[0] += DISTANCE_FACE_TO_CAM;
  }

  RcsBody* faceBdy = RcsGraph_getBodyByName(graph, faceName.c_str());
  RCHECK_MSG(faceBdy, "Face body with name '%s' not found - please make sure it exists in the xml file.", faceName.c_str());
  for (unsigned int i = 0; i < faceBdy->nShapes; ++i)
  {
    RcsShape* sh = &faceBdy->shapes[i];
    if (sh->type == RCSSHAPE_MESH)
    {
      //RLOG(0, "Found mesh at index %d", i);
      if ((sh->mesh->nVertices==mesh->nVertices) && (sh->mesh->nFaces==mesh->nFaces))
      {
        if (RcsShape_isOfComputeType(sh, RCSSHAPE_COMPUTE_RESIZEABLE))
        {
          //RLOG(0, "Copying mesh with %d vertices and %d faces", mesh->nVertices, mesh->nFaces);
          for (unsigned int i = 0; i < mesh->nVertices; ++i)
          {
            double* dst = &sh->mesh->vertices[3 * i];
            Vec3d_invTransform(dst, A_FC, MatNd_getRowPtr(landmarks, i));
          }
        }
        else
        {
          RLOG(1, "Face mesh not resizeable - skipping vertices updating. Please fix in xml file!");
        }


      }
    }
  }


  // Update debug graphics
  if (sw.valid())
  {
    viewer->lock();
    faceMeshNode->update(mesh);
    faceFrameNode->setTransformation(&faceTrf);
    viewer->unlock();
  }

}


void FaceTracker::setScene(aff::ActionScene* scene_)
{
  this->scene = scene_;
}

void FaceTracker::updateAgents(RcsGraph* graph)
{
  if (!this->scene)
  {
    return;
  }

}

const RcsMeshData* FaceTracker::getMesh() const
{
  return this->mesh;
}

bool FaceTracker::addGraphics(Rcs::Viewer* viewer_, const RcsBody* cameraFrame)
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
  HTr A_cam = cameraFrame->A_BI;
  A_cam.org[0] += DISTANCE_FACE_TO_CAM;
  sw->setTransformation(&A_cam);

  this->landmarksNode = new Rcs::VertexArrayNode(this->landmarks, osg::PrimitiveSet::POINTS, "RED");
  landmarksNode->setPointSize(3.0);
  sw->addChild(landmarksNode.get());

  this->faceFrameNode = new Rcs::COSNode(0.4);
  sw->addChild(faceFrameNode.get());

  viewer->add(sw.get());

  // We show the debug face in world coordinates
  this->faceMeshNode = new Rcs::MeshNode(this->mesh);
  faceMeshNode->clearMesh();
  viewer->add(faceMeshNode.get());

  return true;
}

void FaceTracker::setCameraTransform(const HTr* A_CI)
{
  HTr_copy(&A_camI, A_CI);
}

// This method estimates the frame of reference for the face landmarks.
// x points forward, z points upward, y points from right eye to left eye.
// The frame center lies between the eyes. The face transform is represented
// in the frame of the Mediapipe camera.
/*static*/ HTr FaceTracker::estimateFaceTransform(const MatNd* faceLandMarks)
{
  HTr trf;
  HTr_setIdentity(&trf);

  // Face only: 468, face + iris: 478
  if (faceLandMarks->m < 468)
  {
    NLOG(0, "Wrong number of face landmarks: %d - should be 468", faceLandMarks->m);
    return trf;
  }

  const double* lm6 = MatNd_getRowPtr(faceLandMarks, 6);      // Between the eyes
  const double* lm10 = MatNd_getRowPtr(faceLandMarks, 10);    // Top center of forehead
  const double* lm33 = MatNd_getRowPtr(faceLandMarks, 33);    // Right eye
  const double* lm152 = MatNd_getRowPtr(faceLandMarks, 152);  // Chin center
  const double* lm263 = MatNd_getRowPtr(faceLandMarks, 263);  // Left eye

  // z-axis is 152 - 10
  Vec3d_sub(trf.rot[2], lm10, lm152);
  Vec3d_normalizeSelf(trf.rot[2]);

  // TODO: Are these orthogonal?
  // y-axis is 33 - 263
  Vec3d_sub(trf.rot[1], lm263, lm33);
  Vec3d_normalizeSelf(trf.rot[1]);

  // x-axis is right-handed
  Vec3d_crossProduct(trf.rot[0], trf.rot[1], trf.rot[2]);

  // Origin is landmark 6
  Vec3d_copy(trf.org, lm6);

  return trf;
}

bool FaceTracker::isVisible() const
{
  return (landmarks->m>0) ? true : false;
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

}   // namespace aff
