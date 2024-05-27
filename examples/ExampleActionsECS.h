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
#include <TaskGuiComponent.h>
#include <TextEditComponent.h>
#include <ActionComponent.h>
#include <TrajectoryComponent.h>
#include <IKComponent.h>

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

  void addComponent(ComponentBase* component);
  void addHardwareComponent(ComponentBase* component);
  bool isFinalPoseRunning() const;
  size_t getNumFailedActions() const;

  std::vector<std::pair<std::string,std::string>> getCompletedActionStack() const;
  void clearCompletedActionStack();
  void lockStepMtx() const;
  void unlockStepMtx() const;

  std::string xmlFileName;
  std::string config_directory;
  std::string sequenceCommand;
  std::string lastResultMsg;
  std::vector<std::string> lastFeedbackMsg;
  unsigned int speedUp;
  int maxNumThreads;
  bool noLimits, noViewer, noTextGui, earlyExitAction;
  bool unittest, verbose, processingAction, turbo;
  bool noSpeedCheck, noJointCheck, noCollCheck, noTrajCheck;

  std::unique_ptr<GraphicsWindow> viewer;

private:

  EntityBase entity;
  double trajTime;
  std::vector<std::string> actionStack;
  IKComponent::IkSolverType ikType;
  double dt, dt_max, dt_max2, alpha, lambda, dtProcess, dtEvents;
  bool plot, valgrind, withRobot, pause, withTaskGui, withEventGui;
  bool zigzag, singleThreaded;
  unsigned int loopCount;

  std::unique_ptr<GraphComponent> graphC;
  std::unique_ptr<Rcs::ControllerBase> controller;
  std::unique_ptr<SceneQueryPool> sceneQuery;
  std::unique_ptr<ActionComponent> actionC;
  std::unique_ptr<TextEditComponent> textGui;
  std::unique_ptr<TrajectoryComponent> trajC;
  std::unique_ptr<IKComponent> ikc;
  std::unique_ptr<TaskGuiComponent> taskGui;

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
  void onActionResult(bool success, double quality, std::vector<std::string> resMsg);
  void onProcess();
  void onSetTurboMode(bool enable);

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
