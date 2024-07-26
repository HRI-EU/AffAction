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

#include "LandmarkBase.h"
#include "ArucoTracker.h"
#include "AzureSkeletonTracker.h"
#include "FaceTracker.h"
#include "SceneJsonHelpers.h"

#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_typedef.h>



namespace aff
{


LandmarkBase::LandmarkBase() : scene(NULL), graph(NULL), frozen(false),
  syncInputJsonWithWallclockTime(false)
{
}

LandmarkBase::~LandmarkBase()
{
}

void LandmarkBase::setScenePtr(RcsGraph* graph_, ActionScene* scene_)
{
  graph = graph_;
  scene = scene_;
}

void LandmarkBase::updateGraph(RcsGraph* g)
{
  for (const auto& tracker : trackers)
  {
    tracker->updateGraph(g);
  }
}

std::string LandmarkBase::getTrackerState() const
{
  if ((!graph) || (!scene))
  {
    RLOG(1, "Can't update scene - no graph has been added");
    return std::string();
  }

  nlohmann::json json;
  // getSceneState(json, scene, graph);

  // Add skeleton information
  for (const auto& tracker : trackers)
  {
    AzureSkeletonTracker* st = dynamic_cast<AzureSkeletonTracker*>(tracker.get());
    if (st)
    {
      //jsonFromSkeletons(json, st);
      st->jsonFromSkeletons(json);
    }
  }

  return json.dump();
}

void LandmarkBase::addTracker(std::unique_ptr<TrackerBase> tracker)
{
  trackers.push_back(std::move(tracker));
}

void LandmarkBase::setJsonInput(const nlohmann::json& json)
{
  double time;

  if (syncInputJsonWithWallclockTime)
  {
    time = TrackerBase::getWallclockTime();
  }
  else
  {
    time = json["header"]["timestamp"];
  }

  std::string cameraFrame = json["header"]["frame_id"];

  for (auto& entry : json["data"].items())
  {
    //RLOG_CPP(1, entry.key());

    for (const auto& tracker : trackers)
    {
      if (entry.key() == tracker->getRequestKeyword())
      {
        tracker->parse(entry.value(), time, cameraFrame);
      }
    }
  }
}

void LandmarkBase::setCameraTransform(const HTr* A_camI)
{
  double x[6];
  HTr_to6DVector(x, A_camI);
  RLOG(0, "Callback: Camera pose for xml: %.3f %.3f %.3f  %.3f %.3f %.3f",
       x[0], x[1], x[2], RCS_RAD2DEG(x[3]), RCS_RAD2DEG(x[4]), RCS_RAD2DEG(x[5]));
  for (const auto& tracker : trackers)
  {
    tracker->setCameraTransform(A_camI);
  }
}

void LandmarkBase::addArucoTracker(const std::string& camera, const std::string& baseMarker)
{
  auto tracker = new ArucoTracker(camera, baseMarker);
  tracker->addCalibrationFinishedCallback([this](const HTr* A_CI) -> void
  {
    setCameraTransform(A_CI);
  });
  addTracker(std::unique_ptr<ArucoTracker>(tracker));
}

TrackerBase* LandmarkBase::addSkeletonTracker(size_t numSkeletons)
{
  auto tracker = new AzureSkeletonTracker(numSkeletons);
  addTracker(std::unique_ptr<AzureSkeletonTracker>(tracker));
  return tracker;
}

int LandmarkBase::addSkeletonTrackerForAgents(double r)
{
  if (!scene)
  {
    RLOG(0, "Can't add skeleton tracker for agents - scene has not been set");
    return 0;
  }

  size_t numHumanAgents = 0;
  for (const auto& agent : scene->agents)
  {
    if (dynamic_cast<HumanAgent*>(agent))
    {
      numHumanAgents++;
    }
  }

  if (numHumanAgents==0)
  {
    RLOG(0, "Can't add skeleton tracker for agents - no human agent found");
    return 0;
  }

  auto tracker = new AzureSkeletonTracker(numHumanAgents);
  tracker->setScene(this->scene);
  addTracker(std::unique_ptr<AzureSkeletonTracker>(tracker));
  tracker->addAgents();
  tracker->setSkeletonDefaultPositionRadius(r);

  return numHumanAgents;
}

void LandmarkBase::setSkeletonTrackerDefaultRadius(double r)
{
  for (auto& tracker : trackers)
  {
    AzureSkeletonTracker* st = dynamic_cast<AzureSkeletonTracker*>(tracker.get());

    if (st)
    {
      st->setSkeletonDefaultPositionRadius(r);
    }
  }
}

void LandmarkBase::setSkeletonTrackerDefaultPosition(size_t skeletonIndex, double x, double y, double z)
{
  for (auto& tracker : trackers)
  {
    AzureSkeletonTracker* st = dynamic_cast<AzureSkeletonTracker*>(tracker.get());

    if (st)
    {
      st->setSkeletonDefaultPosition(skeletonIndex, x, y, z);
    }
  }
}

TrackerBase* LandmarkBase::addFaceTracker(const std::string& agent, const std::string& camera)
{
  if (!scene)
  {
    RLOG(0, "Can't add face tracker - scene has not been set");
    return nullptr;
  }

  FaceTracker* tracker = new FaceTracker();
  tracker->setScene(this->scene);
  tracker->setCameraName(camera);
  addTracker(std::unique_ptr<FaceTracker>(tracker));

  return tracker;
}

void LandmarkBase::startCalibration(const std::string& camera, size_t numFrames)
{
  for (auto& tracker : trackers)
  {
    ArucoTracker* arucoTracker = dynamic_cast<ArucoTracker*>(tracker.get());

    if (arucoTracker && (arucoTracker->getCameraName()==camera))
    {
      arucoTracker->calibrate(numFrames);
    }
  }
}

bool LandmarkBase::isCalibrating(const std::string& camera) const
{
  for (auto& tracker : trackers)
  {
    ArucoTracker* arucoTracker = dynamic_cast<ArucoTracker*>(tracker.get());

    if (arucoTracker && (arucoTracker->getCameraName()==camera))
    {
      return arucoTracker->isCalibrating();
    }
  }

  RLOG_CPP(0, "Couldn't find camera with name '" << camera << "'");

  return false;
}

void LandmarkBase::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  const double wallClockTime = TrackerBase::getWallclockTime();
  for (const auto& tracker : trackers)
  {
    tracker->setCurrentTime(wallClockTime);
  }

  if (!frozen)
  {
    updateGraph(desired);
  }
}

bool LandmarkBase::isFrozen() const
{
  return this->frozen;
}

void LandmarkBase::onFreezePerception(bool freeze)
{
  this->frozen = freeze;
}

void LandmarkBase::setSyncInputWithWallclock(bool enable)
{
  syncInputJsonWithWallclockTime = enable;
}

bool LandmarkBase::getSyncInputWithWallclock() const
{
  return syncInputJsonWithWallclockTime;
}

const RcsGraph* LandmarkBase::getGraph() const
{
  return this->graph;
}

const ActionScene* LandmarkBase::getScene() const
{
  return this->scene;
}

ActionScene* LandmarkBase::getScene()
{
  return this->scene;
}

std::vector<std::unique_ptr<TrackerBase>>& LandmarkBase::getTrackers()
{
  return this->trackers;
}

void LandmarkBase::createDebugGraphics(Rcs::Viewer* viewer)
{
  for (auto& tracker : getTrackers())
  {
    // Add skeleton graphics
    aff::AzureSkeletonTracker* st = dynamic_cast<aff::AzureSkeletonTracker*>(tracker.get());
    if (st)
    {
      st->initGraphics(getGraph(), viewer);
    }

    // Add facemesh graphics
    aff::FaceTracker* ft = dynamic_cast<aff::FaceTracker*>(tracker.get());
    if (ft)
    {
      const RcsBody* cam = RcsGraph_getBodyByName(getGraph(), ft->getCameraName().c_str());
      ft->addGraphics(viewer, cam);
    }

  }

}

void LandmarkBase::enableDebugGraphics(bool enable)
{
  for (auto& tracker : getTrackers())
  {


    // Handle facemesh graphics
    aff::FaceTracker* ft = dynamic_cast<aff::FaceTracker*>(tracker.get());
    if (ft)
    {
      RLOG(0, "Setting FaceTracker visibility to %s", enable ? "TRUE" : "FALSE");
      ft->enableDebugGraphics(enable);
    }

  }

}

}   // namespace
