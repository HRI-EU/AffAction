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

#ifndef AFF_EXAMPLELLMSIM_H
#define AFF_EXAMPLELLMSIM_H

#include "ExampleActionsECS.h"

#define ASIO_STANDALONE

#if defined (_MSC_VER)
#define _WEBSOCKETPP_CPP11_TYPE_TRAITS_
#endif
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <thread>


namespace aff
{

typedef websocketpp::server<websocketpp::config::asio> WebsocketServer;

class ExampleLLMSim : public ExampleActionsECS
{
public:

  ExampleLLMSim();
  ExampleLLMSim(int argc, char** argv);
  virtual ~ExampleLLMSim();
  virtual void onStartWebSocket();
  virtual void onStopWebSocket();
  virtual std::string getSceneEntities() const;
  virtual std::string help();
  virtual bool initAlgo();
  virtual bool initGraphics();
  size_t getNumFailedActions() const;
  void setUseWebsocket(bool enable);
  bool getUseWebsocket() const;

  virtual bool parseArgs(Rcs::CmdLineParser* parser);
  virtual void onActionResult(bool success, double quality, std::string resMsg);
  virtual void onTextCommand(std::string text);

  void runThread(double freq);

  WebsocketServer server;
  websocketpp::connection_hdl hdl; /// TODO: Support multiple connections?
  bool connected = false;
  bool started = false;
  bool useWebsocket = true;
  unsigned int port = 35000;
  size_t numFailedActions = 0;
  std::thread bgThread;
  std::string lastResultMsg;
};

}   // namespace aff

#endif   // AFF_EXAMPLELLMSIM_H
