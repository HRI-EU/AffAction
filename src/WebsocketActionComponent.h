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

#ifndef AFF_WEBSOCKETACTIONCOMPONENT_H
#define AFF_WEBSOCKETACTIONCOMPONENT_H

#include "ComponentBase.h"

#define ASIO_STANDALONE

#if defined (_MSC_VER)
#define _WEBSOCKETPP_CPP11_TYPE_TRAITS_
#endif
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <thread>

#define AFF_WEBSOCKETACTIONCOMPONENT_DEFAULTPORT (35000)

namespace aff
{

typedef websocketpp::server<websocketpp::config::asio> WebsocketServer;

class WebsocketActionComponent : public ComponentBase
{
public:

  WebsocketActionComponent(EntityBase* parent, unsigned int port=AFF_WEBSOCKETACTIONCOMPONENT_DEFAULTPORT);
  virtual ~WebsocketActionComponent();

private:

  void initWebsocket();
  void runThread(double freq);
  virtual void onStartWebSocket();
  virtual void onStopWebSocket();
  virtual void onSendWebSocket(std::string textToSend);

  WebsocketServer server;
  websocketpp::connection_hdl hdl;
  bool connected = false;
  bool started = false;
  unsigned int port = AFF_WEBSOCKETACTIONCOMPONENT_DEFAULTPORT;
  std::thread bgThread;
  std::string lastResultMsg;
};

}   // namespace aff

#endif   // AFF_WEBSOCKETACTIONCOMPONENT_H
