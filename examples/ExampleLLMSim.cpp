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

#include "ExampleLLMSim.h"
#include "ActionFactory.h"
#include "SceneJsonHelpers.h"

#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_body.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_shape.h>
#include <Rcs_timer.h>

#include <unordered_set>



namespace aff
{

RCS_REGISTER_EXAMPLE(ExampleLLMSim, "Actions", "LLM Simulator");


ExampleLLMSim::ExampleLLMSim() : ExampleActionsECS(0, NULL)
{
}

ExampleLLMSim::ExampleLLMSim(int argc, char** argv) :
  ExampleActionsECS(argc, argv)
{
}

ExampleLLMSim::~ExampleLLMSim()
{
  onStopWebSocket();
}

bool ExampleLLMSim::initParameters()
{
  ExampleActionsECS::initParameters();
  config_directory = "config/xml/SmileSimulator";
  xmlFileName = "g_scenario_pizza.xml";
  noTrajCheck = false;
  noLimits = false;
  noTextGui = false;
  speedUp = 3;

  return true;
}

bool ExampleLLMSim::initGraphics()
{
  ExampleActionsECS::initGraphics();

  if (noViewer || valgrind)
  {
    RLOG(0, "Running without graphics");
    return true;
  }

  // Check if there is a body named 'initial_camera_view'.
  double q_cam[6];
  VecNd_set6(q_cam, -2.726404, -2.515165, 3.447799,   -0.390124, 0.543041, 0.795294);

  const RcsBody* camera_body = RcsGraph_getBodyByName(graphC->getGraph(), "default_camera_view");
  if (camera_body)
  {
    RLOG(1, "Setting initial view based on body 'initial_camera_view'.");
    HTr_to6DVector(q_cam, &camera_body->A_BI);
  }

  viewer->setCameraTransform(q_cam[0], q_cam[1], q_cam[2], q_cam[3], q_cam[4], q_cam[5]);

  if (!hwc.empty())
  {
    entity.publish("BackgroundColor", std::string("PEWTER"));

    viewer->setKeyCallback('l', [this](char k)
    {
      RLOG(0, "Toggle talk flag");
      entity.publish("ToggleASR");

    }, "Toggle talk flag");

  }

  viewer->setKeyCallback('a', [this](char k)
  {
    RLOG(0, "Test pan tilt angle calculation");
    Agent* robo_ = getScene()->getAgent("Robo");
    RCHECK(robo_);

    double panTilt[2], err[2];
    size_t maxIter = 100;
    double eps = 1.0e-8;
    RobotAgent* robo = dynamic_cast<RobotAgent*>(robo_);

    double t_calc = Timer_getSystemTime();
    int iter = robo->getPanTilt(controller->getGraph(), "table", panTilt, maxIter, eps, err);
    t_calc = Timer_getSystemTime() - t_calc;

    RLOG(0, "pan=%.1f tilt=%.1f err=%f %f took %d iterations and %.1f msec",
         RCS_RAD2DEG(panTilt[0]), RCS_RAD2DEG(panTilt[1]), err[0], err[1], iter, 1.0e3*t_calc);

  }, "Test pan tilt angle calculation");

  return true;
}

bool ExampleLLMSim::initAlgo()
{
  ExampleActionsECS::initAlgo();

  if (unittest)
  {
    useWebsocket = false;
    entity.subscribe<bool, double, std::string>("ActionResult", &ExampleLLMSim::onActionResultUnittest, this);
  }
  else
  {
    entity.subscribe<bool, double, std::string>("ActionResult", &ExampleLLMSim::onActionResult, this);
  }

  entity.subscribe("TextCommand", &ExampleLLMSim::onTextCommand, this);
  entity.subscribe("Stop", &ExampleLLMSim::onStopWebSocket, this);
  entity.subscribe("Start", &ExampleLLMSim::onStartWebSocket, this);
  entity.subscribe("Process", &ExampleLLMSim::process, this);

  if (useWebsocket)
  {
    // configure ws server
    RLOG(0, "Configuring websocket server");
    this->server.clear_access_channels(websocketpp::log::alevel::all);
    this->server.clear_error_channels(websocketpp::log::elevel::all);
    this->server.set_error_channels(websocketpp::log::elevel::warn |
                                    websocketpp::log::elevel::rerror |
                                    websocketpp::log::elevel::fatal);
    this->server.set_message_handler([this](websocketpp::connection_hdl hdl, WebsocketServer::message_ptr msg)
    {
      RLOG(0, "Received msg: '%s'", msg->get_payload().c_str());
      entity.publish("ActionSequence", msg->get_payload());
    });
    this->server.set_fail_handler([this](websocketpp::connection_hdl hdl)
    {
      RLOG(0, "Websocket connection failed!");
    });
    this->server.set_open_handler([this](websocketpp::connection_hdl hdl)
    {
      this->hdl = hdl;
      this->connected = true;
      RLOG(0, "Connected to %s", this->server.get_con_from_hdl(hdl)->get_host().c_str());
    });
    this->server.set_close_handler([this](websocketpp::connection_hdl hdl)
    {
      this->connected = false;
      RLOG(0, "Connection to %s closed", this->server.get_con_from_hdl(hdl)->get_host().c_str());
      this->hdl.reset();
    });



    // init websocket server
    RLOG(0, "Initializing websocket server");
    this->server.init_asio();
    this->server.set_reuse_addr(true);
    this->server.listen(asio::ip::tcp::v4(), this->port);
    this->server.start_accept();
  }

  // Create static graph copies at this point
  collectFeedback();

  return true;
}

std::string ExampleLLMSim::help()
{
  std::stringstream s;

  s << ExampleBase::help();

  std::string usage = "Websocket action server \n"
                      "   This example requires to start a websocket server\n";

  s << usage;

  return s.str();
}

bool ExampleLLMSim::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-port", &port, "Websocket port (default: %d)", port);
  parser->getArgument("-useWebsocket", &useWebsocket, "Use websocket (default: %s)", useWebsocket ? "true" : "false");
  bool res = ExampleActionsECS::parseArgs(parser);

  if (parser->hasArgument("-h"))
  {
    std::cout << help() << std::endl;
    res = false;
  }

  return res;
}

void ExampleLLMSim::process()
{
  entity.process();
}

void ExampleLLMSim::onStartWebSocket()
{
  if (started)
  {
    return;
  }

  started = true;

  if (useWebsocket)
  {
    bgThread = std::thread(&ExampleLLMSim::runThread, this, 12.0);
  }

}

void ExampleLLMSim::onStopWebSocket()
{

  if (useWebsocket)
  {
    if (connected)
    {
      this->connected = false;
      this->server.stop_listening();
      this->server.close(this->hdl, websocketpp::close::status::going_away, "Robot shut down");
      this->server.poll();   // must process the stop events submitted above
    }

    if (started)
    {
      started = false;
      bgThread.join();
    }
  }
  else
  {
    started = false;
  }

}

void ExampleLLMSim::onActionResult(bool success, double quality, std::string resMsg)
{
  RMSG_CPP(resMsg);
  entity.publish("SetTextLine", resMsg, 1);

  // This needs to be done independent of failure or success
  if (!actionStack.empty())
  {
    addToCompletedActionStack(actionStack[0], resMsg);
    printCompletedActionStack();
    actionStack.erase(actionStack.begin());
  }

  if (!success)
  {
    numFailedActions++;
    actionStack.clear();
    RLOG_CPP(0, "Clearing action stack, number of failed actions: " << numFailedActions);
  }

  // After the last action command of a sequence has been finished (the
  // trajectory has ended), or we face a failure, we "unfreeze" the perception
  // and accept updates from the perceived scene.
  if (actionStack.empty())
  {
    RLOG(0, "ActionStack is empty, unfreezing perception");
    entity.publish("FreezePerception", false);
    entity.publish<std::string, std::string>("RenderCommand", "BackgroundColor", "");
  }

  // In valgrind mode, we only perform one action, since this might run with valgrind
  if (valgrind)
  {
    ExampleBase::stop();
  }
  else if (connected && useWebsocket)
  {
    this->server.send(hdl, resMsg, websocketpp::frame::opcode::TEXT);
  }

  // HACK \todo(MG): Unify this
  failCount = numFailedActions;
}

void ExampleLLMSim::onActionResultUnittest(bool success, double quality, std::string resMsg)
{
  RMSG_CPP("Unittest: " + resMsg);
  entity.publish("SetTextLine", resMsg, 1);

  lastResultMsg = resMsg;

  if (!success)
  {
    numFailedActions++;

    if (!actionStack.empty())
    {
      entity.publish("TextCommand", std::string("reset"));
    }
  }

  // In valgrind mode, we only perform one action, since this might run with valgrind
  if (valgrind)
  {
    ExampleBase::stop();
  }

  // HACK \todo(MG): Unify this
  failCount = numFailedActions;
}

// This only handles the "get_state" keyword
void ExampleLLMSim::onTextCommand(std::string text)
{
  RMSG_CPP("ExampleLLMSim::onTextCommand RECEIVED: " << text);
  if (text=="get_state")
  {
    std::thread feedbackThread([this]()
    {
      std::string fb = collectFeedback();
      if (connected && useWebsocket)
      {
        this->server.send(hdl, fb, websocketpp::frame::opcode::TEXT);
      }
    });
    feedbackThread.detach();
  }
}

void ExampleLLMSim::runThread(double freq)
{
  while (started)
  {
    this->server.poll();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / freq));
  }
}

size_t ExampleLLMSim::getNumFailedActions() const
{
  return numFailedActions;
}

std::string ExampleLLMSim::getSceneEntities() const
{
  std::string res = "These entities are in the scene: ";
  const ActionScene* scene = getScene();

  // Only add each item once in case of duplicate names.
  std::unordered_set<std::string> ntts;

  for (const auto& e : scene->entities)
  {
    ntts.insert(e.name);
  }

  for (const auto& n : ntts)
  {
    res += n + ", ";
  }

  return res;
}

std::string ExampleLLMSim::collectFeedback() const
{

  double t0 = Timer_getSystemTime();
  static RcsGraph* staticGraph = NULL;
  static ActionScene staticScene;

  stepMtx.lock();
  if (!staticGraph)
  {
    staticGraph = RcsGraph_clone(controller->getGraph());
}
  else
  {
    RcsGraph_copy(staticGraph, controller->getGraph());
  }
  double t1 = Timer_getSystemTime();

  staticScene = *getScene();
  double t2 = Timer_getSystemTime();
  stepMtx.unlock();

  NLOG(1, "get_state took %.3f  %.3f msec (graph, actions)",
       (t1-t0)*1.0e3, (t2-t1)*1.0e3);

  nlohmann::json stateJson;
  getSceneState(stateJson, &staticScene, staticGraph);

  return stateJson.dump();
}

void ExampleLLMSim::setUseWebsocket(bool enable)
{
  useWebsocket = enable;
}

bool ExampleLLMSim::getUseWebsocket() const
{
  return useWebsocket;
}


}   // namespace aff
