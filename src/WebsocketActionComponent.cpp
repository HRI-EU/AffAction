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

#include "WebsocketActionComponent.h"

#include <Rcs_macros.h>



namespace aff
{

WebsocketActionComponent::WebsocketActionComponent(EntityBase* parent, unsigned int port_) :
  ComponentBase(parent), port(port_)
{
  getEntity()->subscribe("Stop", &WebsocketActionComponent::onStopWebSocket, this);
  getEntity()->subscribe("Start", &WebsocketActionComponent::onStartWebSocket, this);
  getEntity()->subscribe("SendWebsocket", &WebsocketActionComponent::onSendWebSocket, this);
}

WebsocketActionComponent::~WebsocketActionComponent()
{
  onStopWebSocket();
}

void WebsocketActionComponent::onStartWebSocket()
{
  if (!started)
  {
    started = true;
    initWebsocket();
    bgThread = std::thread(&WebsocketActionComponent::runThread, this, 12.0);
  }

}

void WebsocketActionComponent::onStopWebSocket()
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

void WebsocketActionComponent::onSendWebSocket(std::string textToSend)
{
  if (connected)
  {
    this->server.send(hdl, textToSend, websocketpp::frame::opcode::TEXT);
  }
  else
  {
    RLOG_CPP(0, "Websocket not connected - skipped sending '" << textToSend << "'");
  }
}

void WebsocketActionComponent::runThread(double freq)
{
  while (started)
  {
    this->server.poll();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / freq));
  }
}

void WebsocketActionComponent::initWebsocket()
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
    getEntity()->publish("ActionSequence", msg->get_payload());
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


}   // namespace aff
