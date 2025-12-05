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

#ifndef AFF_EXAMPLEJOINTCONTROL_H
#define AFF_EXAMPLEJOINTCONTROL_H

#include "EntityBase.h"

#include <GraphComponent.h>
#include <GraphicsWindow.h>
#include <JointGuiComponent.h>

#include <ExampleBase.h>
#include <ControllerBase.h>



namespace aff
{

class ExampleJointControl : public Rcs::ExampleBase
{
public:

  ExampleJointControl();
  ExampleJointControl(int argc, char** argv);
  virtual ~ExampleJointControl();

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
  void addComponentArgument(const std::string& arg);

protected:

  // Subscribed callbacks
  void onQuit();
  void onPrint();

  EntityBase entity;
  std::string xmlFileName;
  std::string configDirectory;
  std::string componentArgs;
  std::string renderStringHUD;

  bool noGraphics = false;
  bool blockingMainThread = false;
  double dt = 0.01;
  double tmc = 0.1;

  double dt_max = 0.0, dtProcess = 0.0;
  unsigned int loopCount = 0;

  GraphicsWindow* viewer = nullptr;
  GraphComponent* graphC = nullptr;
  JointGuiComponent* jguiC = nullptr;

  std::unique_ptr<Rcs::ControllerBase> controller;

  ES::SubscriberCollectionDecay<RcsGraph*>* updateGraph = nullptr;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*>* postUpdateGraph = nullptr;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeKinematics = nullptr;
  ES::SubscriberCollectionDecay<const MatNd*>* setJointCommand = nullptr;
  ES::SubscriberCollectionDecay<>* setRenderCommand = nullptr;

  std::vector<ComponentBase*> hwc;
  std::vector<ComponentBase*> components;
};

}   // namespace aff

#endif   // AFF_EXAMPLEJOINTCONTROL_H
