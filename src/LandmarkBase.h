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

#ifndef LANDMARKBASE_H
#define LANDMARKBASE_H

#include "TrackerBase.h"
#include "ActionScene.h"


namespace aff
{

class LandmarkBase
{
public:

  LandmarkBase();

  virtual ~LandmarkBase();

  void setJsonInput(const nlohmann::json& json);
  std::string getTrackerState() const;

  void updateGraph(RcsGraph* graph);
  void addTracker(std::unique_ptr<TrackerBase> tracker);
  void setCameraTransform(const HTr* A_CI);
  void addArucoTracker(const std::string& camera="camera",
                       const std::string& baseMarker="aruco_base");
  TrackerBase* addSkeletonTracker(size_t numSkeletons);
  int addSkeletonTrackerForAgents(double defaultRadius);
  void setSkeletonTrackerDefaultRadius(double r);
  void setSkeletonTrackerDefaultPosition(size_t skeletonIndex, double x, double y, double z);

  void startCalibration(const std::string& camera, size_t numFrames);
  bool isCalibrating(const std::string& camera) const;
  void setScenePtr(RcsGraph* graph, ActionScene* scene);

  void onFreezePerception(bool freeze);
  bool isFrozen() const;
  const RcsGraph* getGraph() const;
  const ActionScene* getScene() const;
  virtual void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);
  std::vector<std::unique_ptr<TrackerBase>>& getTrackers();

protected:

  std::vector<std::unique_ptr<TrackerBase>> trackers;
  ActionScene* scene;
  RcsGraph* graph;
  bool frozen;
};

}   // namespace

#endif // LANDMARKBASE_H
