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

#ifndef AFF_WEBSOCKETCLIENTCOMPONENT_H
#define AFF_WEBSOCKETCLIENTCOMPONENT_H

#include "ComponentBase.h"

#define ASIO_STANDALONE

#if defined (_MSC_VER)
#define _WEBSOCKETPP_CPP11_TYPE_TRAITS_
#define WEBSOCKETPP_USE_STD_RANDOM_DEVICE
#pragma warning(push)
#pragma warning(disable : 4267)
#endif

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#if defined (_MSC_VER)
#pragma warning(pop)
#endif

#include <Rcs_macros.h>

#include <iostream>
#include <string>
#include <json.hpp>


using json = nlohmann::json;

namespace aff
{

// {"tool_name": "speak_only", "arguments": {"text": "Hello there I am a lively agent that is really fun to interact with"}}
// {"tool_name": "analyse_webcam_image", "arguments": {"question": "What do you see in the image"}}

typedef websocketpp::client<websocketpp::config::asio_client> client;




class WebsocketClientComponent : public ComponentBase
{
public:

  WebsocketClientComponent(EntityBase* parent, std::string uri_="ws://localhost:5557") : ComponentBase(parent), uri(uri_)
  {
    subscribe("fire_tool_and_forget", &WebsocketClientComponent::fire_and_forget);
    subscribe("fire_tool_and_wait", &WebsocketClientComponent::fire_and_wait);
  }

private:

  void fire_and_forget_(std::string content)
  {
    // The payload you want to send
    nlohmann::json payload =
    {
      {"role", "system"},
      {"content", content}
    };
    std::string message = payload.dump();

    client c;

    try
    {
      // Logging off for clarity
      c.clear_access_channels(websocketpp::log::alevel::all);
      c.clear_error_channels(websocketpp::log::elevel::all);

      c.init_asio();

      std::string uri = "ws://localhost:5557";  // must match your Python server

      websocketpp::lib::error_code ec;
      client::connection_ptr con = c.get_connection(uri, ec);
      if (ec)
      {
        std::cout << "could not create connection: " << ec.message() << std::endl;
        return;
      }

      // We only want to send once, then immediately close
      con->set_open_handler([&c, message](websocketpp::connection_hdl hdl)
      {
        c.send(hdl, message, websocketpp::frame::opcode::text);
        c.close(hdl, websocketpp::close::status::going_away, "done");
      });

      c.connect(con);

      // Run the Asio event loop (blocks until closed)
      c.run();
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception: " << e.what() << std::endl;
    }
    catch (websocketpp::lib::error_code e)
    {
      std::cerr << "WebSocket++ Error: " << e.message() << std::endl;
    }
    catch (...)
    {
      std::cerr << "Other exception" << std::endl;
    }
  }

  void fire_and_forget(std::string content)
  {
    std::thread([this,content]()
    {
      this->fire_and_forget_(content);
    }).detach();
  }




  void fire_and_wait_(std::string content)
  {
    nlohmann::json payload =
    {
      {"role", "system"},
      {"content", content}
    };
    std::string message = payload.dump();

    client c;

    try
    {
      c.clear_access_channels(websocketpp::log::alevel::all);
      c.clear_error_channels(websocketpp::log::elevel::all);

      c.init_asio();

      std::string uri = "ws://localhost:5557";  // must match your Python server

      websocketpp::lib::error_code ec;
      client::connection_ptr con = c.get_connection(uri, ec);
      if (ec)
      {
        std::cerr << "Could not create connection: " << ec.message() << std::endl;
        return;
      }

      // open handler: send the message
      con->set_open_handler([&c, message](websocketpp::connection_hdl hdl)
      {
        c.send(hdl, message, websocketpp::frame::opcode::text);
      });

      // message handler: print response and close
      con->set_message_handler([this,&c](websocketpp::connection_hdl hdl, client::message_ptr msg)
      {
        c.close(hdl, websocketpp::close::status::going_away, "done");

        json payload =
        {
          {"tool_name", "speak_only"},
          {"arguments", {{"text", msg->get_payload()}}}
        };

        std::string speak_str = payload.dump();
        this->getEntity()->publish("AAA", speak_str);
      });

      // error handler (optional but useful)
      con->set_fail_handler([&c](websocketpp::connection_hdl hdl)
      {
        RLOG_CPP(0, "Connection failed");
      });

      c.connect(con);

      // This will block until the connection is closed
      c.run();
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception: " << e.what() << std::endl;
    }
    catch (websocketpp::lib::error_code e)
    {
      std::cerr << "WebSocket++ Error: " << e.message() << std::endl;
    }
    catch (...)
    {
      std::cerr << "Other exception" << std::endl;
    }
  }

  void fire_and_wait(std::string content)
  {
    std::thread([this,content]()
    {
      this->fire_and_wait_(content);
    }).detach();
  }

  std::string uri;
};



}   // namespace


#endif
