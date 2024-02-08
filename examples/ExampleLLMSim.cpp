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
  }

  return true;
}

bool ExampleLLMSim::initAlgo()
{
  ExampleActionsECS::initAlgo();

  if (unittest)
  {
    useWebsocket = false;
  }

  entity.subscribe<bool, double, std::string>("ActionResult", &ExampleLLMSim::onActionResult, this);
  entity.subscribe("TextCommand", &ExampleLLMSim::onTextCommand, this);
  entity.subscribe("Stop", &ExampleLLMSim::onStopWebSocket, this);
  entity.subscribe("Start", &ExampleLLMSim::onStartWebSocket, this);

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

  return true;
}

std::string ExampleLLMSim::help()
{
  std::stringstream s;

  s << ExampleActionsECS::help();

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
  if (verbose)
  {
    RMSG_CPP(resMsg);
  }
  entity.publish("SetTextLine", resMsg, 1);

  lastResultMsg = resMsg;

  // This needs to be done independent of failure or success
  if (!actionStack.empty())
  {
    addToCompletedActionStack(actionStack[0], resMsg);

    if (verbose)
    {
      printCompletedActionStack();
    }

    actionStack.erase(actionStack.begin());
  }

  if (!success)
  {
    numFailedActions++;

    if (unittest)
    {
      if (!actionStack.empty())
      {
        // \todo: Check why this works
        //RPAUSE_MSG("Calling reset");
        entity.publish("TextCommand", std::string("reset"));
        actionStack.insert(actionStack.begin(), "reset");
      }
    }
    else
    {
      actionStack.clear();
    }
    RLOG_CPP(0, "Clearing action stack, number of failed actions: " << numFailedActions);
  }

  // After the last action command of a sequence has been finished (the
  // trajectory has ended), or we face a failure, we "unfreeze" the perception
  // and accept updates from the perceived scene.
  if (actionStack.empty())
  {
    RLOG(0, "ActionStack is empty, unfreezing perception");
    entity.publish("FreezePerception", false);
    processingAction = false;
    actionC->setFinalPoseRunning(false);
  }

  if ((actionStack.size()==1) && STRNEQ(actionStack[0].c_str(), "pose", 4))
  {
    actionC->setFinalPoseRunning(true);
  }

  // In valgrind mode, we only perform one action, since this might run with valgrind
  if (valgrind || (unittest && actionStack.empty()))
  {
    ExampleBase::stop();
  }
  else if (connected && useWebsocket)
  {
    this->server.send(hdl, resMsg, websocketpp::frame::opcode::TEXT);
  }

  // HACK \todo(MG): Unify this
  failCount = numFailedActions;
  //processingAction = false;
}

// This only handles the "get_state" keyword
void ExampleLLMSim::onTextCommand(std::string text)
{
  if (verbose)
  {
    RMSG_CPP("ExampleLLMSim::onTextCommand RECEIVED: " << text);
  }

  if (text=="get_state")
  {
    std::thread feedbackThread([this]()
    {
      std::string fb = sceneQuery->getSceneState().dump();
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

#if 1
  // Add agents \todo(MG): Names not resolved properly
  if (!scene->agents.empty())
  {
    res += " These agents are in the scene: ";

    for (const auto& a : scene->agents)
    {
      if (!a->types.empty())
      {
        res += a->types[0] + ", ";
      }
    }

  }

  // Add agent's reachability
  if (!scene->agents.empty())
  {

    for (const auto& a : scene->agents)
    {
      if (!a->types.empty())
      {
        auto ntts = a->getObjectsInReach(scene, controller->getGraph());
        if (ntts.empty())
        {
          res += a->name + " can not reach any object ";
        }
        else
        {
          res += a->name + " can reach these objects: ";
          for (const auto& n : ntts)
          {
            res += n->name + ", ";
          }
        }
      }
    }
  }
#endif

  return res;
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
