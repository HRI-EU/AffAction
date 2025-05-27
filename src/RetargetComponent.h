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

#ifndef AFF_RETARGETCOMPONENT_H
#define AFF_RETARGETCOMPONENT_H

#include "ComponentBase.h"

#include <IkSolverRMR.h>

#include <mutex>


namespace aff
{

class RetargetLogger : public ComponentBase
{
public:
  RetargetLogger(EntityBase* parent, const std::string& fileName);
  virtual ~RetargetLogger();

  void startRecording();
  void stopRecording();

private:
  void onRetarget(std::map<int, std::vector<HTr>> poses);
  FILE* fd;
  bool record;
  double lastSampleTime;
  std::string fileName;
};

class RetargetPlayer : public ComponentBase
{
public:
  RetargetPlayer(EntityBase* parent, const std::string& fileName);
  virtual ~RetargetPlayer();

  void startPlaying();
  void stopPlaying();
  double getDt() const;

  void onRender();
private:
  MatNd* data;
  bool play;
  size_t rowIdx;
  size_t loopCount;
  size_t everyNth;
};

class RetargetComponent : public ComponentBase
{
public:

  enum class BodyType
  {
    OpenSim, BVH, DexBot
  };

  enum class PoseState
  {
    Visible, Hidden
  };

  struct Pose : public ComponentBase
  {
    Pose(EntityBase* parent, BodyType bType, int poseId);
    ~Pose();

    void onRender();
    void onToggleKinetics();
    void onGraphicsWindowFeedback(std::string feedbackType, std::string graphId);

    PoseState getPoseState() const;
    void setPoseState(PoseState newState);
    bool isValid() const;
    void makeValid();
    void initialize(double seconds);
    void retarget(const std::vector<HTr>& frames);
    void retarget_init(const std::vector<HTr>& frames);
    void retarget_IK(const std::vector<HTr>& frames);
    void setCameraTransform(const HTr* A_CI);
    bool setCameraTransform(const std::string& bdyName);
    void filterVisualGraph(double tmc);
    std::string getGraphIdStr() const;
    double computeHeight() const;

    static std::map<std::string, int> getNameIdMap(BodyType bType);
    static std::string getConfigFileName(BodyType bType);

    int poseId;
    Rcs::ControllerBase controller;
    Rcs::IkSolverRMR ikSolver;
    RcsGraph* visGraph;
    std::map<std::string, int> nameIdMap;
    std::vector<HTr> frames, rawFrames;
    MatNd* a_des;
    MatNd* x_des;
    HTr A_CI;
    double countDown;
    double lastUpdate;
    bool graphicsInitialized;
    bool kineticsEnabled;
    PoseState poseState;
    BodyType bType;
    size_t numIkIterationsPerStep;
    double bodyHeight;
  };

  RetargetComponent(EntityBase* parent, BodyType bType, size_t maxPeople=5);
  virtual ~RetargetComponent();

  // Camera convention: z points forward, y points down, x points left
  virtual bool setCameraTransform(const std::string& bdyName);
  virtual void toggleThreading();

  //private:

  void onSetCameraTransform(HTr A_CI);
  void onRetarget(std::map<int, std::vector<HTr>> poses);
  std::vector<int> findCorrespondences(std::map<int, std::vector<HTr>> frames) const;

  HTr A_CI;   // To be stored here for newly created poses
  std::vector<Pose*> poses;
  bool threadedRetarget;
  mutable std::mutex threadMtx;
  double dt_retarget;
};

}  // namespace aff

#endif   // AFF_RETARGETCOMPONENT_H
