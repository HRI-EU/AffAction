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
#include "SceneJsonHelpers.h"

#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_typedef.h>



namespace aff
{


LandmarkBase::LandmarkBase() : graph(NULL), ownsScene(false)
{
}

LandmarkBase::LandmarkBase(const std::string& configFile) : graph(NULL), ownsScene(true)
{
  graph = RcsGraph_create(configFile.c_str());
  RCHECK(graph);
  scene = new aff::ActionScene(graph->cfgFile);
  RCHECK(scene);
}

LandmarkBase::~LandmarkBase()
{
  if (ownsScene)
  {
    delete graph;
    delete scene;
  }
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

void LandmarkBase::updateScene(double currentTime)
{
  if (!graph)
  {
    RLOG(1, "Can't update scene - no scene has been added");
    return;
  }

  for (const auto& tracker : trackers)
  {
    tracker->setCurrentTime(currentTime);
  }

  updateGraph(graph);
}

/* Json format for (skeleton) agents

"agent":
{
  { 1,
    {"type" : "human",
    "visible" : true / false,
    "links" : [{"link_id": 0, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] },
                {"link_id": 1, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] },
                ...
                {"link_id": 31, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] }]
    }
  },
  { 2,
    {"type" : "human",
    "visible" : true / false,
    "links" : [{"link_id": 0, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] },
                {"link_id": 1, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] },
                ...
                {"link_id": 31, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] }]
    }
  }

}

*/
void LandmarkBase::jsonFromSkeletons(nlohmann::json& json, TrackerBase* skeletonTracker) const
{
  AzureSkeletonTracker* st = dynamic_cast<AzureSkeletonTracker*>(skeletonTracker);
  RCHECK(st);

  // Here we have a valid skeleton tracker
  size_t skeletonId = 0;

  for (const auto& skeleton : st->skeletons)
  {
    nlohmann::json skeletonJson;
    skeletonJson["skeleton_id"] = skeletonId;
    skeletonJson["type"] = "human";
    skeletonJson["visible"] = skeleton->isVisible ? true : false;

    size_t linkId = 0;

    for (auto& link : skeleton->markers)
    {
      double ea[3];
      Mat3d_toEulerAngles(ea, link.rot);

      nlohmann::json linkJson =
      {
        {"link_id", linkId},
        {"position", std::vector<double>(link.org, link.org+3)},
        {"euler_xyzr", std::vector<double>(ea, ea+3)}
      };
      skeletonJson["links"] += linkJson;
      linkId++;
    }   // for (const auto& link : skeleton->markers)

    //RLOG_CPP(0, "skeletonJson: '" << skeletonJson.dump() << "'");
    json["agent"][skeletonId] = skeletonJson;
    skeletonId++;
  }   // for (const auto& skeleton : st->skeletons)

}

std::string LandmarkBase::getState() const
{
  if ((!graph) || (!scene))
  {
    RLOG(1, "Can't update scene - no graph has been added");
    return std::string();
  }

  nlohmann::json json;
  getSceneState(json, scene, graph);

  // Add skeleton information
  for (const auto& tracker : trackers)
  {
    AzureSkeletonTracker* st = dynamic_cast<AzureSkeletonTracker*>(tracker.get());
    if (st)
    {
      jsonFromSkeletons(json, st);
    }
  }

  return json.dump();
}

void LandmarkBase::addTracker(std::unique_ptr<TrackerBase> tracker)
{
  trackers.push_back(std::move(tracker));
}

void LandmarkBase::updateFromJson(const nlohmann::json& json)
{
  // RLOG_CPP(0, "\nHeader: " << json["header"]["timestamp"]);
  // RLOG_CPP(0, "\nHeader: " << typeid(json["header"]["timestamp"]).name());

  const double time = json["header"]["timestamp"];
  std::string cameraFrame = json["header"]["frame_id"];

  for (auto& entry : json["data"].items())
  {
    NLOG_CPP(1, entry.key());

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

TrackerBase* LandmarkBase::addSkeletonTracker(size_t numSkeletons=3)
{
  auto tracker = new AzureSkeletonTracker(numSkeletons);
  tracker->lmc = this;
  addTracker(std::unique_ptr<AzureSkeletonTracker>(tracker));
  return tracker;
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

}   // namespace
