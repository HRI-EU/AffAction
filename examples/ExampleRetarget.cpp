/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "ExampleRetarget.h"
#include "EventGui.h"

#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graph.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_math.h>
#include <Rcs_typedef.h>

#include <fstream>
#include <iostream>
#include <csignal>


namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleRetarget, "Azure", "Retarget");

ExampleRetarget::ExampleRetarget(int argc, char** argv) :
  Rcs::ExampleBase(argc, argv), entity()
{
  pause = false;
  noLimits = false;
  noViewer = false;
  logToFile = false;
  noRetarget = false;
  dt = 0.01;
  dt_max = 0.0;
  loopCount = 0;
  maxPeople = 5;
  dataSource = "AzureCPP";

  entity.subscribe("Quit", &ExampleRetarget::stop, this);
}

ExampleRetarget::~ExampleRetarget()
{
  Rcs_removeResourcePath(cfgDir.c_str());
}

bool ExampleRetarget::initParameters()
{
  cfgDir = "config/data";
  avatarName ="OpenSim";
  cameraBodyName = "Camera";
  return true;
}

bool ExampleRetarget::initAlgo()
{
  Rcs_addResourcePath(cfgDir.c_str());
  entity.setDt(dt);

  if (pause)
  {
    entity.call("TogglePause");
  }


  if (dataSource=="AzurePython")
  {
    // Socket interface to Python Mediapipe
    mpC = std::make_unique<LandmarkZmqComponent>(&entity);
    //mpC->setSocketTimeout(100.0);
  }
  else if (dataSource=="AzureCPP")
  {
    azC = std::make_unique<AzureBodyTrackingComponent>(&entity);
  }
  else if (dataSource=="FromFile")
  {
    player = std::make_unique<RetargetPlayer>(&entity, cfgDir+std::string("/frames-win.dat"));
  }

  // Retargetting component using Azure Kinect data
  if (!noRetarget)
  {
    RetargetComponent::BodyType bType = RetargetComponent::BodyType::OpenSim;
    if (avatarName == "DexBot")
    {
      bType = RetargetComponent::BodyType::DexBot;
    }
    else if (avatarName == "BVH")
    {
      bType = RetargetComponent::BodyType::BVH;
    }
    else if (avatarName == "Smile")
    {
      bType = RetargetComponent::BodyType::SmileJaco2;
    }
    retargetC = std::make_unique<RetargetComponent>(&entity, bType, maxPeople);
    retargetC->setCameraTransform(cameraBodyName);
  }

  if (logToFile)
  {
    logger = std::make_unique<RetargetLogger>(&entity, "frames.dat");
  }


  return true;
}

bool ExampleRetarget::initGraphics()
{
  if (noViewer)
  {
    return true;
  }

  // Optinal graphics window. We don't use a static instance since this will
  // not be cleaned up after exiting main.
  viewer = std::make_unique<GraphicsWindow>(&entity);
  viewer->setTitle("ExampleRetarget");

  viewer->setKeyCallback('e', [this](char k)
  {
    RLOG(0, "Launching event gui");
    new aff::EventGui(&entity);
  }, "Launch event gui");

  viewer->setKeyCallback('k', [this](char k)
  {
    RLOG(0, "Toggling kinetics calculation");
    entity.publish("RetargetToggleKinetics");
  }, "Toggle kinetics calculation");

  viewer->setKeyCallback(' ', [this](char k)
  {
    entity.call("TogglePause");

  }, "Toggle pause modus");

  viewer->setKeyCallback('C', [this](char k)
  {
    RLOG(0, "Calibrating camera with floor plane estimation");
    entity.publish("EstimateCameraPose");

  }, "Calibrate depth camera with floor plane estimation");

  viewer->setKeyCallback('l', [this](char k)
  {
    if (logger)
    {
      RLOG(0, "Start logging");
      logger->startRecording();
    }
    else
    {
      RMSG("Logger not active");
    }

  }, "Start logging azure transforms");

  viewer->setKeyCallback('L', [this](char k)
  {
    if (logger)
    {
      RLOG(0, "Stop logging");
      logger->stopRecording();
    }
    else
    {
      RMSG("Logger not active");
    }

  }, "Stop logging azure transforms");

  viewer->setKeyCallback('t', [this](char k)
  {
    if (retargetC)
    {
      retargetC->toggleThreading();
      RLOG(0, "Threaded retargetting is %s", retargetC->threadedRetarget ? "ON" : "OFF");
    }

  }, "Toggle threaded retragetting calculation");
  viewer->setKeyCallback('X', [this](char k)
  {
    RLOG(0, "Initializing all retargetting instances");
    entity.publish("RetargetInitialize", 10.0);

  }, "Initialize retargetting");

  viewer->setKeyCallback('V', [this](char k)
  {
    RLOG(0, "Scaling graphs with factor 0.7");
    if (retargetC)
    {
      const double scale = 0.70;
      //for (size_t i = 0; i < retargetC2->poses.size(); ++i)
      size_t i=0;
      {
        RcsGraph_scale(retargetC->poses[i]->controller.getGraph(), scale);
        RcsGraph_scale(retargetC->poses[i]->visGraph, scale);
        entity.publish("RenderCommand", retargetC->poses[i]->getGraphIdStr(), std::string("erase"));
      }
    }

  }, "Test scale");

  viewer->start();

  // We disble the MeshFactory so that meshes can be scaled individually. This allows us to
  // adjust the body sizes per retargetting model.
  entity.publish("RenderCommand", std::string("SetEnableMeshFactory"), std::string("false"));

  return true;
}

bool ExampleRetarget::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-pause", &pause,  "Hit enter for step");
  parser->getArgument("-noLimits", &noLimits, "No kinematic limits considered in IK");
  parser->getArgument("-noViewer", &noViewer, "Don't launch graphics");
  parser->getArgument("-dataSource", &dataSource, "Data source: AzurePython, AzureCPP, FromFile, None (default: %s)", dataSource.c_str());
  parser->getArgument("-dir", &cfgDir, "Configuration file directory "
                      "(default is %s)", cfgDir.c_str());
  parser->getArgument("-dt", &dt, "Time step (default is %f)", dt);
  parser->getArgument("-avatar", &avatarName, "Avatar to retarget data to: OpenSim, BHV, DexBot"
                      "(default is %s)", avatarName.c_str());
  parser->getArgument("-camBody", &cameraBodyName, "Body to which camera is attached "
                      "(default is %s)", cameraBodyName.c_str());
  parser->getArgument("-log", &logToFile, "Log marker transforms to file frames.xml");
  parser->getArgument("-noRetarget", &noRetarget, "Skip IK retargetting");
  parser->getArgument("-maxPeople", &maxPeople, "Maximum number of people to be tracked (Default is %zu)", maxPeople);

  return true;
}

void ExampleRetarget::start()
{
  entity.publish("Start");
  entity.process();
  runLoop = true;
  run();
}

void ExampleRetarget::stop()
{
  entity.publish("Stop");
  entity.processUntilEmpty(100);
  runLoop = false;
}

void ExampleRetarget::step()
{
  double dt_max2 = 0.0;
  double dtProcess = Timer_getSystemTime();
  entity.publish("Render");
  entity.process();
  entity.stepTime();
  dtProcess = Timer_getSystemTime() - dtProcess;

  if (entity.getTime() > 3.0)
  {
    dt_max = std::max(dt_max, dtProcess);
    dt_max2 = std::max(dt_max2, dtProcess);
  }

  loopCount++;

  char timeStr[256];
  snprintf(timeStr, 256, "Time: %.3f   dt: %.1f dt_max: %.1f %.1f msec\n"
           "max. queue: %zu",
           entity.getTime(), dtProcess * 1.0e3, dt_max * 1.0e3, dt_max2 * 1.0e3,
           entity.getMaxQueueSize());
  entity.publish("SetTextLine", std::string(timeStr), 0);

  Timer_waitDT(entity.getDt() - dtProcess);
}

/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleRetargetLog, "Azure", "Record data");

ExampleRetargetLog::ExampleRetargetLog(int argc, char** argv) :
  ExampleRetarget(argc, argv)
{
}

ExampleRetargetLog::~ExampleRetargetLog()
{
}

bool ExampleRetargetLog::initParameters()
{
  ExampleRetarget::initParameters();
  logToFile = true;
  noRetarget = true;
  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleRetargetPlay, "Azure", "Play data from file");

ExampleRetargetPlay::ExampleRetargetPlay(int argc, char** argv) :
  ExampleRetarget(argc, argv)
{
}

ExampleRetargetPlay::~ExampleRetargetPlay()
{
}

bool ExampleRetargetPlay::initParameters()
{
  ExampleRetarget::initParameters();
  dataSource = "FromFile";
  return true;
}

}   // namespace
