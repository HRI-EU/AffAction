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
#include <RcsViewer.h>
#include <KeyCatcher.h>
#include <GraphNode.h>
#include <HUD.h>
#include <BodyPointDragger.h>
#include <VertexArrayNode.h>
#include <ControllerWidgetBase.h>
#include <JointWidget.h>
#include <MatNdWidget.h>
#include <ActionBase.h>

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
  std::shared_ptr<ConcurrentSceneQuery> getQuery();
  GraphicsWindow* getViewer();
  const EntityBase& getEntity() const;
  EntityBase& getEntity();
  const VirtualCamera* getVirtualCamera() const;
  VirtualCamera* getVirtualCamera();

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
  unsigned int virtualCameraWidth, virtualCameraHeight;
  bool virtualCameraEnabled, virtualCameraWindowEnabled;
  std::string virtualCameraBodyName;
  unsigned int speedUp;
  int maxNumThreads;
  bool noLimits, noViewer, noTextGui, earlyExitAction;
  bool unittest, verbose, turbo;
  bool noSpeedCheck, noJointCheck, noCollCheck, noTrajCheck;

  std::unique_ptr<GraphicsWindow> viewer;

protected:

  EntityBase entity;
  double trajTime;
  std::string sequenceCommand;
  std::string componentArgs;
  std::string physicsEngine;
  std::vector<std::string> actionStack;
  IKComponent::IkSolverType ikType;
  double dt, dt_max, dt_max2, alpha, lambda, dtProcess, dtEvents;
  bool plot, valgrind, withRobot, pause, withEventGui;
  bool zigzag, singleThreaded;
  unsigned int loopCount;
  std::atomic<bool> processingAction;

  std::unique_ptr<GraphComponent> graphC;
  std::unique_ptr<Rcs::ControllerBase> controller;
  std::unique_ptr<SceneQueryPool> sceneQuery;
  std::unique_ptr<ActionComponent> actionC;
  std::unique_ptr<TextEditComponent> textGui;
  std::unique_ptr<TrajectoryComponent> trajC;
  std::unique_ptr<IKComponent> ikc;
  std::unique_ptr<VirtualCamera> virtualCamera;

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

  ES::SubscriberCollectionDecay<RcsGraph*>* updateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*>* postUpdateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeKinematics;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeTrajectory;
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
};

}   // namespace aff

#endif   // AFF_EXAMPLEACTIONSECS_H
