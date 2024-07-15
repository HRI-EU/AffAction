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

#ifndef AZURESKELETONTRACKER_H
#define AZURESKELETONTRACKER_H

#include "TrackerBase.h"
#include "ActionScene.h"

#include <Rcs_HTr.h>
#include <RcsViewer.h>

#include <map>
#include <memory>
#include <functional>



namespace aff
{

struct Skeleton;

class AzureSkeletonTracker : public TrackerBase
{
public:

  AzureSkeletonTracker(size_t numSkeletons);

  virtual ~AzureSkeletonTracker();

  void initGraphics(const RcsGraph* graph, Rcs::Viewer* viewer);

  // Process aruco frames. Called from control loop (100Hz or so)
  void updateGraph(RcsGraph* graph);

  std::string getRequestKeyword() const;

  void setCameraTransform(const HTr* A_CI);

  bool isSkeletonVisible(size_t idx) const;

  void setSkeletonDefaultPosition(size_t skeletonIdx, double x, double y, double z);

  void setSkeletonDefaultPositionRadius(double r);

  void setSkeletonName(size_t skeletonIdx, const std::string& name);

  void addAgent(const std::string& agentName);

  void addAgents();

  void setScene(aff::ActionScene* scene);

  void jsonFromSkeletons(nlohmann::json& json) const;

  void registerAgentAppearDisappearCallback(std::function<void(const std::string& agentName, bool appear)> callback);

  // bool setParameter(const std::string& parameterName, void* ptr);

private:

  // Process aruco frames. Called from perception thread (30Hz or so)
  void parse(const nlohmann::json& json, double time, const std::string& cameraFrame);

  void updateAgents(RcsGraph* graph);

  void updateSkeletons(RcsGraph* graph);

  std::vector<int> findCorrespondences(std::map<int, std::vector<HTr>> markerMap) const;
  std::vector<std::unique_ptr<Skeleton>> skeletons;
  std::vector<std::function<void(const std::string&, bool)>> agentAppearDisappearCb;
  bool newAzureUpdate;
  double defaultPosRadius;
  HTr A_CI;
  size_t skeletonIndex = 0;
  ActionScene* scene;
};

} // namespace aff

#endif // AZURESKELETONTRACKER_H
