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
#include "YoloTracker.h"
#include "SceneHelpers.h"
#include "SceneJsonHelpers.h"

#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_typedef.h>


namespace aff
{

LandmarkBase::LandmarkBase() : frozen(false), syncInputJsonWithWallclockTime(false)
{
}

LandmarkBase::~LandmarkBase()
{
}

void LandmarkBase::addTracker(std::unique_ptr<TrackerBase> tracker)
{
  trackers.push_back(std::move(tracker));
}

void LandmarkBase::setJsonInput(const nlohmann::json& json_data)
{
  if (!json_data.contains("header"))
  {
    RLOG(2, "No 'header' found in json - returning");
    RLOG_CPP(3, "This is the json:\n" << json_data.dump(2));
    return;
  }

  double time = 0.0;
  const nlohmann::json& json_header = json_data["header"];
  // RLOG_CPP(1, "This is the header:\n" << json_header.dump(2));
  // RLOG_CPP(3, "This is the data:\n" << json_data.dump(2));
  // REXEC(3)
  // {
  //   RLOG_CPP(0, trackers.size() << " trackers found:");
  //   for (const auto& tracker : trackers)
  //   {
  //     RLOG_CPP(0, "[" << tracker->getRequestKeyword() << "]");
  //   }
  // }

  if (syncInputJsonWithWallclockTime)
  {
    time = getWallclockTime();
  }
  else
  {
    time = json_header["timestamp"];
  }

  // Delegate parsing of data to added trackers
  if (json_data.contains("data"))
  {
    for (auto& entry : json_data["data"].items())
    {
      for (const auto& tracker : trackers)
      {
        const std::string& key = entry.key();
        const std::string keyword = tracker->getRequestKeyword();
        //RLOG_CPP(0, "[" << key << "]: " << keyword);

        //if (entry.key() == tracker->getRequestKeyword())
        if (key.size() >= keyword.size() &&
            key.compare(0, keyword.size(), keyword) == 0)
        {
          //RLOG_CPP(1, "[" << entry.key() << "]: " << entry.value());
          tracker->parse(json_header, entry.value(), time);
        }
      }
    }
  }
}

void LandmarkBase::addArucoTracker(const std::string& camera, const std::string& baseMarker)
{
  auto tracker = new ArucoTracker(camera, baseMarker);
  addTracker(std::unique_ptr<ArucoTracker>(tracker));
  RLOG(0, "Added ArucoTracker");
}

TrackerBase* LandmarkBase::addSkeletonTracker(size_t numSkeletons)
{
  auto tracker = new AzureSkeletonTracker(numSkeletons);
  addTracker(std::unique_ptr<AzureSkeletonTracker>(tracker));
  return tracker;
}

int LandmarkBase::addSkeletonTrackerForAgents(const ActionScene* scene, double r)
{
  if (!scene)
  {
    RLOG(1, "Can't add skeleton tracker for agents - scene has not been set");
    return 0;
  }

  const size_t numHumanAgents = scene->getAgents<HumanAgent>().size();

  if (numHumanAgents==0)
  {
    RLOG(1, "Can't add skeleton tracker for agents - no human agent found");
    return 0;
  }

  auto tracker = std::make_unique<AzureSkeletonTracker>(numHumanAgents);
  tracker->addAgents(scene);
  tracker->setSkeletonDefaultPositionRadius(r);
  addTracker(std::move(tracker));

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

TrackerBase* LandmarkBase::addFaceTracker(const std::string& faceBodyName, const std::string& camera, const std::string& agent)
{
  FaceTracker* tracker = new FaceTracker(faceBodyName, camera, agent);
  addTracker(std::unique_ptr<FaceTracker>(tracker));

  return tracker;
}

TrackerBase* LandmarkBase::addYoloTracker(const std::string& camera)
{
  YoloTracker* tracker = new YoloTracker(camera);
  addTracker(std::unique_ptr<YoloTracker>(tracker));
  return tracker;
}

void LandmarkBase::estimateCameraPose(int numFrames)
{
  for (auto& t : trackers)
  {
    ArucoTracker* at = dynamic_cast<ArucoTracker*>(t.get());

    if (at)
    {
      RLOG(0, "Calibrating Aruco camera");
      at->calibrate(numFrames);
    }
  }
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

void LandmarkBase::onUpdateScene(RcsGraph* desired, RcsGraph* current, ActionScene* scene)
{
  const double wallClockTime = getCurrentTime();

  for (auto& tracker : trackers)
  {
    tracker->setCurrentTime(wallClockTime);
    tracker->update(scene, desired);
  }

}

double LandmarkBase::getCurrentTime() const
{
  return getWallclockTime();
}

bool LandmarkBase::isFrozen() const
{
  return this->frozen;
}

void LandmarkBase::onFreezePerception(bool freeze)
{
  this->frozen = freeze;
  for (auto& tracker : getTrackers())
  {
    tracker->setFrozen(freeze);
  }
}

void LandmarkBase::setSyncInputWithWallclock(bool enable)
{
  syncInputJsonWithWallclockTime = enable;
}

bool LandmarkBase::getSyncInputWithWallclock() const
{
  return syncInputJsonWithWallclockTime;
}

std::vector<std::unique_ptr<TrackerBase>>& LandmarkBase::getTrackers()
{
  return this->trackers;
}

void LandmarkBase::createDebugGraphics(Rcs::Viewer* viewer, const RcsGraph* graph)
{
  for (auto& tracker : getTrackers())
  {
    bool success = tracker->initDebugGraphics(viewer, graph);
    if (!success)
    {
      RLOG_CPP(1, "Failed to add tracker for '" << tracker->getRequestKeyword() <<"'");
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
