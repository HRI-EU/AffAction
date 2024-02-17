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

  EntityBase entity;
  double trajTime;
  std::string xmlFileName;
  std::string config_directory;
  std::string sequenceCommand;
  std::vector<std::string> actionStack;
  IKComponent::IkSolverType ikType;
  double dt, dt_max, dt_max2, alpha, lambda;
  unsigned int speedUp, loopCount, lookaheadCount;
  bool pause, noSpeedCheck, noJointCheck, noCollCheck, noTrajCheck;
  bool noLimits, zigzag, withEventGui, withTaskGui, noViewer, noTextGui;
  bool plot, valgrind, unittest, withRobot;
  bool singleThreaded, verbose, processingAction, lookahead;
  double dtProcess, dtEvents;
  size_t failCount;

  std::unique_ptr<Rcs::ControllerBase> controller;
  std::unique_ptr<TextEditComponent> textGui;
  std::unique_ptr<ActionComponent> actionC;
  std::unique_ptr<GraphComponent> graphC;
  std::unique_ptr<TrajectoryComponent> trajC;
  std::unique_ptr<IKComponent> ikc;
  std::unique_ptr<GraphicsWindow> viewer;
  std::unique_ptr<TaskGuiComponent> taskGui;
  std::unique_ptr<ConcurrentSceneQuery> sceneQuery;
  std::unique_ptr<ConcurrentSceneQuery> sceneQuery2;
  std::unique_ptr<ConcurrentSceneQuery> panTiltQuery;

  ExampleActionsECS(int argc, char** argv);
  virtual ~ExampleActionsECS();
  virtual bool initParameters();
  virtual bool parseArgs(Rcs::CmdLineParser* parser);
  virtual bool initAlgo();
  virtual bool initGraphics();
  virtual bool initGuis();
  virtual void run();
  virtual void startThreaded();
  virtual void step();
  virtual std::string help();
  ActionScene* getScene();
  const ActionScene* getScene() const;
  void addComponent(ComponentBase* component);
  void addHardwareComponent(ComponentBase* component);
  bool isFinalPoseRunning() const;

  std::vector<std::pair<std::string,std::string>> getCompletedActionStack() const;

  void clearCompletedActionStack();
  mutable std::mutex stepMtx;

protected:

  void setEnableRobot(bool enable);
  bool getRobotEnabled() const;
  void addToCompletedActionStack(std::string action, std::string result);
  void printCompletedActionStack() const;
  std::vector<std::pair<std::string,std::string>> completedActionStack;
  mutable std::mutex actionStackMtx;
  std::vector<ComponentBase*> hwc;
  std::vector<ComponentBase*> components;

private:

  void onQuit();
  void onPrint();
  void onActionSequence(std::string text);
  void onPlanActionSequence(std::string text);
  void onTrajectoryMoving(bool isMoving);
  void onTextCommand(std::string text);
  void onChangeBackgroundColorFreeze(bool freeze);
  void process();

  ES::SubscriberCollectionDecay<RcsGraph*>* updateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*>* postUpdateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeKinematics;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeTrajectory;
  ES::SubscriberCollectionDecay<const MatNd*, const MatNd*>* setTaskCommand;
  ES::SubscriberCollectionDecay<const MatNd*>* setJointCommand;
  ES::SubscriberCollectionDecay<>* setRenderCommand;

  RcsGraph* graphToInitializeWith;
};

}   // namespace aff

#endif   // AFF_EXAMPLEACTIONSECS_H
