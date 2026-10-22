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

#ifndef AFF_EXAMPLETELEOP_H
#define AFF_EXAMPLETELEOP_H

#include "EntityBase.h"

#include <GraphComponent.h>
#include <GraphicsWindow.h>
#include <IKTeleOp.h>
#include <ActionScene.h>
#include <VirtualCamera.h>
#include <LeRobotComponent.h>

#include <ExampleBase.h>
#include <ControllerBase.h>

#include <memory>
#include <mutex>
#include <iostream>


namespace aff
{

class ExampleTeleOp : public Rcs::ExampleBase
{
public:

  ExampleTeleOp();
  ExampleTeleOp(int argc, char** argv);
  virtual ~ExampleTeleOp();
  void cleanup();

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
  const EntityBase& getEntity() const;
  EntityBase& getEntity();
  RcsGraph* getGraph();
  const RcsGraph* getGraph() const;
  RcsGraph* getCurrentGraph();
  const RcsGraph* getCurrentGraph() const;
  ActionScene* getScene();
  const ActionScene* getScene() const;
  void addComponentArgument(const std::string& arg);
  void setBuildPath(const std::string& path);
  std::vector<double> getBodyPose(std::string bodyName, bool fromCurrentGraph) const;
  std::vector<std::pair<std::string,VirtualCamera*>> getVirtualCameras();
  bool addVirtualCamera(std::string camera_name, std::string camera_type, int width, int height);
  std::vector<std::pair<std::string,std::unique_ptr<VirtualCamera>>> virtualCameras;

  std::map<std::string, double> getObservation(bool current);
  bool sendAction(const nlohmann::json& command);

  bool withScene = false;
  std::string xmlFileName;
  std::string dataCollectionXmlFileName;
  std::string configDirectory;
  std::string inputType;   // see IKTeleOp::ActionType
  std::string eye_ik_camera;
  bool noLimits = false;
  bool enableRealGraphVisualization = true;
  double dt = 0.01;
  unsigned int loopCount = 0;

protected:

  // Subscribed callbacks
  void onQuit();
  void onPrint();
  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);

  EntityBase entity;
  std::string build_path;
  std::string componentArgs;
  std::string renderStringHUD;

  bool noGraphics = false;
  bool blockingMainThread = true;
  bool runFunctionRunning = false;
  bool withRobo = false;
  double lambda = 1.0e-4;
  double alpha = 0.1;

  double dt_max = 0.0, dtProcess = 0.0;

  GraphicsWindow* viewer = nullptr;
  GraphComponent* graphC = nullptr;
  IKTeleOp* ikc = nullptr;
  LeRobotComponent* lerobotC = nullptr;

  std::unique_ptr<Rcs::ControllerBase> controller;
  std::unique_ptr<ActionScene> scene;
  std::vector<ComponentBase*> hwc;
  std::vector<ComponentBase*> components;
};


}   // namespace aff

#endif   // AFF_EXAMPLETELEOP_H
