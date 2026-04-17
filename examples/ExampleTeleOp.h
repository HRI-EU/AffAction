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
  void addComponentArgument(const std::string& arg);
  void onSetTwist(double vel_x, double vel_y, double vel_z,
                  double vel_thx, double vel_thy, double vel_thz,
                  bool inWorldFrame);
  void onSetWrench(double pos_x, double pos_y, double pos_z,
                   double eul_thx, double eul_thy, double eul_thz,
                   bool inWorldFrame);
  void onSetFingerPose(std::string modelStateName);
  void setBuildPath(const std::string& path);

  std::vector<std::vector<double>> getCollectedData() const;
  void printCollectedData() const;
  void cleanup();
  std::vector<double> getEndEffectorWrench() const;

  bool withScene = false;
  std::string xmlFileName;
  std::string configDirectory;
  std::string inputType;   // "Twist", "Wrench", "Retarget Polar"
  bool noLimits = false;
  bool enableRealGraphVisualization = true;





  std::vector<std::pair<std::string,VirtualCamera*>> getVirtualCameras();
  bool addVirtualCamera(std::string camera_name, std::string camera_type, int width, int height);
  std::vector<std::pair<std::string,std::unique_ptr<VirtualCamera>>> virtualCameras;

protected:

  // Subscribed callbacks
  void onQuit();
  void onPrint();
  void onCollectData(RcsGraph* desired, RcsGraph* current);


  EntityBase entity;
  std::string build_path;
  std::string componentArgs;
  std::string renderStringHUD;

  bool noGraphics = false;
  bool blockingMainThread = true;
  bool runFunctionRunning = false;
  bool withRobo = false;
  double dt = 0.01;
  double lambda = 1.0e-4;
  double alpha = 0.1;

  double dt_max = 0.0, dtProcess = 0.0;
  unsigned int loopCount = 0;

  GraphicsWindow* viewer = nullptr;
  GraphComponent* graphC = nullptr;
  IKTeleOp* ikc = nullptr;

  std::unique_ptr<Rcs::ControllerBase> controller;
  std::unique_ptr<ActionScene> scene;


  ES::SubscriberCollectionDecay<RcsGraph*>* updateGraph = nullptr;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*>* postUpdateGraph = nullptr;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeKinematics = nullptr;
  ES::SubscriberCollectionDecay<double,double,double,double,double,double,bool>* setTwist;
  ES::SubscriberCollectionDecay<std::array<double, 6>, bool>* setTwistCommand;
  ES::SubscriberCollectionDecay<const MatNd*>* setJointCommand = nullptr;
  ES::SubscriberCollectionDecay<>* setRenderCommand = nullptr;
  ES::SubscriberCollectionDecay<RcsGraph*, RcsGraph*, ActionScene*>* updateScene = nullptr;

  std::vector<ComponentBase*> hwc;
  std::vector<ComponentBase*> components;

  std::array<double, 6> twist_des{};
  bool twist_in_world = true;
  std::array<double, 6> wrench_des{};
  bool wrench_in_world = true;
  std::array<double, 16> fingers_des{};
  std::string fingerPoseName;
  std::map<std::string, std::vector<std::pair<int, double>>> modelStates;

  mutable std::mutex eeMtx;
  std::string endEffectorName;
  HTr eeTrf;


  struct CollectedData
  {

    CollectedData()
    {
      data.resize(8);
      data[0].resize(3);
      data[1].resize(9);
      data[2].resize(6);
      data[3].resize(6);
      data[6].resize(6);
      data[7].resize(6);
    }

    CollectedData(size_t dof) : CollectedData()
    {
      data[4].resize(dof);
      data[5].resize(dof);
    }

    void set(const std::vector<double>& pos_in_world,
             const std::vector<double>& rotmat_world_to_endeffector,
             const std::vector<double>& twist_in_world,
             const std::vector<double>& twist_in_endeffector,
             const std::vector<double>& q_curr,
             const std::vector<double>& q_des,
             const std::vector<double>& fts_base,
             const std::vector<double>& fts_ee)
    {
      std::lock_guard<std::mutex> lock(mtx);
      data[0] = pos_in_world;                 // 0-2
      data[1] = rotmat_world_to_endeffector;  // 3-11
      data[2] = twist_in_world;               // 12-17
      data[3] = twist_in_endeffector;         // 18-23
      data[4] = q_curr;                       // 24-75: 53, 24-112: 66
      data[5] = q_des;                        // 76-127: 105, 113-201: 155
      data[6] = fts_base;
      data[7] = fts_ee;
    }

    std::vector<std::vector<double>> get() const
    {
      std::lock_guard<std::mutex> lock(mtx);
      return data;
    }

    void print() const
    {
      auto print_data = get();   // Thread-safe
      std::cout << "Collected data:" << std::endl;

      for (size_t i=0; i<print_data.size(); ++i)
      {
        std::cout << "Data " << i << " has dimension " << print_data[i].size() << std::endl;
        for (size_t j=0; j<print_data[i].size(); ++j)
        {
          std::cout << print_data[i][j] << " ";
        }
        std::cout << std::endl;
      }

    }

    std::vector<std::vector<double>> data;
    mutable std::mutex mtx;
  };

  CollectedData collectedData;
};



class ExampleTeleOpFrankaRight : public ExampleTeleOp
{
public:

  ExampleTeleOpFrankaRight();
  ExampleTeleOpFrankaRight(int argc, char** argv);
  virtual ~ExampleTeleOpFrankaRight() = default;
};


}   // namespace aff

#endif   // AFF_EXAMPLETELEOP_H
