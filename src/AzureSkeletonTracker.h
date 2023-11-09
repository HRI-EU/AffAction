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

#include "LandmarkBase.h"
#include "TrackerBase.h"
#include "Agent.h"
#include "ActionScene.h"

#include <Rcs_HTr.h>
#include <RcsViewer.h>
#include <SphereNode.h>
#include <VertexArrayNode.h>
#include <COSNode.h>

#include <map>
#include <memory>

namespace aff
{

struct Skeleton
{
  Skeleton();
  ~Skeleton();
  void initGraphics(Rcs::Viewer* viewer,
                    const std::string& color);
  void updateGraphics();
  void setExpectedInitialPose(const HTr* pose);
  void setAgent(Agent* a);
  int trackerId;
  double lastUpdate;
  double maxAge;
  bool wasVisible;
  bool isVisible;
  bool becameVisible;
  bool becameInvisible;
  bool hasAgent;
  std::vector<HTr> markers;
  HTr expectedInitialPose;
  Agent* agent;
  std::string name;

  // Only graphics from here
  osg::ref_ptr<osg::Switch> sw;
  osg::ref_ptr<Rcs::VertexArrayNode> lmConnectionsNode;
  std::vector<std::pair<int, int>> connection_idx;
  MatNd* lmConnections;
};



class AzureSkeletonTracker : public TrackerBase
{
public:

  AzureSkeletonTracker(size_t numSkeletons);

  virtual ~AzureSkeletonTracker();

  // Process aruco frames. Called from perception thread (30Hz or so)
  void parse(const nlohmann::json& json, double time, const std::string& cameraFrame);

  // Process aruco frames. Called from control loop (100Hz or so)
  void updateGraph(RcsGraph* graph);

  void updateScene(RcsGraph* graph);

  virtual std::string getRequestKeyword() const;

  void initGraphics(Rcs::Viewer* viewer);

  void setCameraTransform(const HTr* A_CI);

  bool isSkeletonVisible(size_t idx) const;

  void setSkeletonDefaultPosition(size_t skeletonIdx, double x, double y, double z);

  void setSkeletonName(size_t skeletonIdx, const std::string& name);

  void setSkeletonDefaultPositionRadius(double r);

  void addAgent(const std::string& agentName);

  void addAgents();

  const Skeleton* getSkeleton(size_t idx) const;

  void setScene(aff::ActionScene* scene);

  //private:

  std::vector<int> findCorrespondences(std::map<int, std::vector<HTr>> markerMap) const;
  std::vector<std::unique_ptr<Skeleton>> skeletons;
  bool newAzureUpdate;
  double defaultPosRadius;
  HTr A_CI;
  LandmarkBase* lmc;
  size_t skeletonIndex = 0;
  ActionScene* scene;
};

} // namespace aff

#endif // AZURESKELETONTRACKER_H
