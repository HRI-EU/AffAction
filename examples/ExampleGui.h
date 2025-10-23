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

#ifndef AFF_EXAMPLEGUI_H
#define AFF_EXAMPLEGUI_H

#include "EntityBase.h"
#include "ConcurrentSceneQuery.h"

#include <GraphComponent.h>
#include <GraphicsWindow.h>

#include <ExampleBase.h>

#include <atomic>



extern "C" {
  void AffActionExampleInfo();
}

namespace aff
{

class ExampleGui : public Rcs::ExampleBase
{
public:

  ExampleGui();
  ExampleGui(int argc, char** argv);
  virtual ~ExampleGui();

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
  RcsGraph* getGraph();
  const RcsGraph* getGraph() const;
  RcsGraph* getCurrentGraph();
  const RcsGraph* getCurrentGraph() const;
  GraphicsWindow* getViewer();
  bool eraseViewer();
  const EntityBase& getEntity() const;
  EntityBase& getEntity();
  void addComponentArgument(const std::string& arg);
  bool eraseComponent(ComponentBase* component);   // Remove and delete
  std::string getComponentArguments() const;
  const std::vector<ComponentBase*>& getComponentsRef() const;
  void addComponent(ComponentBase* component);
  void addHardwareComponent(ComponentBase* component);

  void lockStepMtx() const;
  void unlockStepMtx() const;

  std::string xmlFileName;
  std::string configDirectory;

  bool blockingMainThread;
  unsigned int speedUp;
  double dt;
  bool enableWireframeToggle;
  bool enableRealGraphVisualization;




protected:

  EntityBase entity;
  std::string componentArgs;
  double dt_max, dt_max2, dtProcess, dtEvents;
  bool withRobot, pause;
  unsigned int loopCount;
  std::atomic<bool> processingAction;

  GraphicsWindow* viewer;
  GraphComponent* graphC;

  std::unique_ptr<Rcs::ControllerBase> controller;


  void setEnableRobot(bool enable);
  bool getRobotEnabled() const;

  // Subscribed callbacks
  void onQuit();
  void onPrint();
  void onProcess();

  ES::SubscriberCollectionDecay<RcsGraph*>* updateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*>* postUpdateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeKinematics;
  ES::SubscriberCollectionDecay<const MatNd*>* setJointCommand;
  ES::SubscriberCollectionDecay<>* setRenderCommand;

  mutable std::mutex stepMtx;
  std::vector<ComponentBase*> hwc;
  std::vector<ComponentBase*> components;
  RcsGraph* graphToInitializeWith;
};

}   // namespace aff

#endif   // AFF_EXAMPLEGUI_H
