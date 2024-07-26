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

#ifndef AFF_FACETRACKER_H
#define AFF_FACETRACKER_H

#include "TrackerBase.h"
#include "ActionScene.h"

#include <RcsViewer.h>
#include <MeshNode.h>
#include <VertexArrayNode.h>
#include <COSNode.h>

#include <mutex>


namespace aff
{

/*******************************************************************************
   Landmark-based estimation of face mesh and face transform based on Mediapipe
   landmarks. The face is represented in a camera frame with z-axis pointing
   outwards, and y-axis pointing downwards (x-axis points right to complete a
   right-handed frame). Mediapipe has no real 3d model, therefore the face
   distance to camera (z) will remain constant (about 1.6m or so). The
   x (left-right) and  y (up-down) dimensions are bewen 0 and 1.
 ******************************************************************************/
class FaceTracker : public TrackerBase
{
public:

  FaceTracker(const std::string& nameOfFaceBody);
  virtual ~FaceTracker();

  // Inherited methods
  std::string getRequestKeyword() const;
  void parse(const nlohmann::json& json, double time, const std::string& cameraFrame);
  void updateGraph(RcsGraph* graph);
  void setCameraTransform(const HTr* A_CI);



  bool addGraphics(Rcs::Viewer* viewer, const RcsBody* cameraFrame);
  void setScene(aff::ActionScene* scene);
  void updateAgents(RcsGraph* graph);
  void setCameraName(const std::string& cameraFrame);
  std::string getCameraName() const;
  const RcsMeshData* getMesh() const;


  bool isVisible() const;
  void enableDebugGraphics(bool enable);

  static const std::string& getFaceMeshDebugString(const std::string& fileName = "FaceMesh.txt");

private:

  FaceTracker(const FaceTracker& other) = delete;

  // This method estimates the frame of reference for the face landmarks.
  // x points forward, z points upward, y points from right eye to left eye.
  // The frame center lies between the eyes.
  static HTr estimateFaceTransform(const MatNd* faceLandMarks);

  ActionScene* scene;
  RcsMeshData* mesh;
  MatNd* landmarks;
  Rcs::Viewer* viewer;
  HTr A_camI;
  HTr faceTrf;
  std::mutex mtx;
  std::string cameraName;
  std::string faceName;
  osg::ref_ptr<Rcs::COSNode> faceFrameNode;
  osg::ref_ptr<Rcs::MeshNode> faceMeshNode;
  osg::ref_ptr<Rcs::VertexArrayNode> landmarksNode;
  osg::ref_ptr<Rcs::NodeBase> sw;
};

}   // namespace aff

#endif   // AFF_FACETRACKER_H
