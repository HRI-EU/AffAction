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

#ifndef AFF_EXAMPLEACTIONSECS_H
#define AFF_EXAMPLEACTIONSECS_H

#include "EntityBase.h"
#include "ConcurrentSceneQuery.h"

#include <GraphComponent.h>
#include <GraphicsWindow.h>
#include <TextEditComponent.h>
#include <ActionComponent.h>
#include <TrajectoryComponent.h>
#include <IKComponent.h>
#include <VirtualCamera.h>

#include <ExampleBase.h>
#include <IkSolverRMR.h>
#include <ActionBase.h>
#include <BodyPointDragger.h>

#include "GazeComponent.h"
#include "SceneTransformationDataRecorder.h"
#include "SceneTransformationDataPlayer.h"

#include <atomic>



extern "C" {
  void AffActionExampleInfo();
}

namespace aff
{

class ExampleActionsECS : public Rcs::ExampleBase
{
public:

  ExampleActionsECS();
  ExampleActionsECS(int argc, char** argv);
  virtual ~ExampleActionsECS();

  // From ExampleBase
  virtual bool initParameters();
  virtual bool parseArgs(Rcs::CmdLineParser* parser);
  virtual bool initAlgo();
  virtual bool initGraphics();
  virtual bool initGuis();
  virtual void run();
  virtual void step();
  virtual std::string help();
  virtual void updateUI();
  virtual void setSyncMode(std::string syncMode);
  virtual void startThreaded();

  // Accessors
  ActionScene* getScene();
  const ActionScene* getScene() const;
  RcsGraph* getGraph();
  const RcsGraph* getGraph() const;
  RcsGraph* getCurrentGraph();
  const RcsGraph* getCurrentGraph() const;
  RcsBroadPhase* getBroadPhase();
  const RcsBroadPhase* getBroadPhase() const;
  const RcsCollisionMdl* getSelfCollisionModel() const;
  std::shared_ptr<ConcurrentSceneQuery> getQuery();
  GraphicsWindow* getViewer();
  bool eraseViewer();
  const EntityBase& getEntity() const;
  EntityBase& getEntity();
  const VirtualCamera* getVirtualCamera(int idx=-1) const;
  VirtualCamera* getVirtualCamera(int idx=-1);
  std::vector<std::pair<std::string,VirtualCamera*>> getVirtualCameras();
  bool addVirtualCamera(std::string camera_name, std::string camera_type, int width, int height);
  void addComponentArgument(const std::string& arg);
  bool eraseComponent(ComponentBase* component);   // Remove and delete
  std::string getComponentArguments() const;
  const std::vector<ComponentBase*>& getComponentsRef() const;
  void addComponent(ComponentBase* component);
  void addHardwareComponent(ComponentBase* component);
  bool isFinalPoseRunning() const;
  size_t getNumFailedActions() const;

  std::vector<std::pair<std::string,std::string>> getCompletedActionStack() const;
  void clearCompletedActionStack();
  void lockStepMtx() const;
  void unlockStepMtx() const;

  bool isProcessingAction() const;
  void setProcessingAction(bool isProcessing);

  std::string xmlFileName;
  std::string configDirectory;
  std::vector<ActionResult> lastActionResult;
  std::vector<std::string> eventQueue;

  unsigned int virtualCameraWidth, virtualCameraHeight;
  bool virtualCameraEnabled, virtualCameraWindowEnabled;
  bool gazeComponentEnabled;
  bool eyeIkEnabled;
  bool usersGazeComponentEnabled;
  bool sceneTransformationDataRecorderEnabled;
  bool sceneTransformationDataPlayerEnabled;
  std::string virtualCameraBodyName;
  unsigned int speedUp;
  int maxNumThreads;
  int numSceneQueries;
  bool noLimits, noViewer, noTextGui, earlyExitAction;
  bool unittest, verbose, turbo;
  bool noSpeedCheck, noJointCheck, noCollCheck, noTrajCheck;
  bool hasBeenStopped;
  bool blockingMainThread;
  double dt;
  bool enableWireframeToggle;
  bool enableRealGraphVisualization;


  /*! \brief Retrieves the gaze data in JSON format.
  * This methods aggregates the gaze data from all gaze components and returns it in JSON format.
  * \return JSON object containing the gaze data from all the agents.
  */
  nlohmann::json getUsersGazeData() const;

  nlohmann::json getRecordedTransformations(double start_time, double end_time) const;
  void loadTransformationDataFromFile(const std::string& filename) const;
  void startPlaybackTransformationData() const;

protected:

  EntityBase entity;
  double trajTime, trajTimeScaling;
  std::string sequenceCommand;
  std::string componentArgs;
  std::string physicsEngine;
  std::string landmarksCamera;
  std::vector<std::string> actionStack;
  IKComponent::IkSolverType ikType;
  double dt_max, dt_max2, alpha, lambda, dtProcess, dtEvents;
  bool plot, valgrind, withRobot, pause, withEventGui;
  bool zigzag, singleThreaded;
  unsigned int loopCount;
  std::atomic<bool> processingAction;

  GraphicsWindow* viewer;
  ActionComponent* actionC;
  GraphComponent* graphC;
  TrajectoryComponent* trajC;
  IKComponent* ikc;
  TextEditComponent* textGui;

  std::vector<std::pair<std::string,std::unique_ptr<VirtualCamera>>> virtualCameras;
  std::unique_ptr<Rcs::ControllerBase> controller;
  std::unique_ptr<SceneQueryPool> sceneQuery;


  osg::ref_ptr<Rcs::BodyPointDragger> dragger;


  void setEnableRobot(bool enable);
  bool getRobotEnabled() const;
  void addToCompletedActionStack(std::string action, std::string result);
  void printCompletedActionStack() const;

  // Subscribed callbacks
  void onQuit();
  void onPrint();
  void onActionSequence(std::string text);
  void onPlanActionSequenceBFS(std::string text);
  void onPlanActionSequenceDFS(std::string text);
  void onPlanActionSequenceDFSEE(std::string text);
  void onTrajectoryMoving(bool isMoving);
  void onTextCommand(std::string text);
  void onChangeBackgroundColorFreeze(bool freeze);
  void onActionResult(bool success, double quality, std::vector<ActionResult> results);
  void onProcess();
  void onSetTurboMode(bool enable);
  void onClearTrajectory();
  void onSetPupilSpeedWeight(double weight);
  void onPause();
  void onResume();
  void onEventReceived(std::string event);

  ES::SubscriberCollectionDecay<RcsGraph*>* updateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*>* postUpdateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*, ActionScene*>* updateScene;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeKinematics;
  ES::SubscriberCollectionDecay<double>* computeTrajectory;
  ES::SubscriberCollectionDecay<const MatNd*, const MatNd*>* setTaskCommand;
  ES::SubscriberCollectionDecay<const MatNd*>* setJointCommand;
  ES::SubscriberCollectionDecay<>* setRenderCommand;

  size_t failCount;
  std::vector<std::pair<std::string,std::string>> completedActionStack;
  mutable std::mutex actionStackMtx;
  mutable std::mutex stepMtx;
  std::vector<ComponentBase*> hwc;
  std::vector<ComponentBase*> components;
  RcsGraph* graphToInitializeWith;

  /*! \brief List of gaze components.
  *
  * This vector contains pointers to instances of GazeComponent,
  * each of which is responsible for tracking the gaze of a specific agent.
  *
  */
  std::vector<GazeComponent*> gazeComponents;
  SceneTransformationDataRecorder* sceneTransformationDataRecorder;
  SceneTransformationDataPlayer* sceneTransformationDataPlayer;
};

}   // namespace aff

#endif   // AFF_EXAMPLEACTIONSECS_H
