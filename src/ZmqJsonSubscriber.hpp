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

#include "ComponentBase.h"
#include "json.hpp"

#include <Rcs_macros.h>

#include <zmq.hpp>

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>


namespace aff
{
class ZmqJsonSubscriber : public ComponentBase
{
private:
  bool runLoop = false;
  bool nwThreadRunning = false;
  std::string ip_address;

public:
  ZmqJsonSubscriber(EntityBase* parent, std::string zmq_ip="tcp://*:5556")
    : ComponentBase(parent), ip_address(zmq_ip)
  {
    RLOG_CPP(0, "Creating ZmqJsonSubscriber with ip " << zmq_ip);
    subscribe("Start", &ZmqJsonSubscriber::onStart);
    subscribe("Stop", &ZmqJsonSubscriber::onStop);
    subscribe("ReceiveZMQ", &ZmqJsonSubscriber::onReceiveZMQ);
  }

  virtual ~ZmqJsonSubscriber()
  {
  }

  void onStart()
  {
    if (nwThreadRunning)
    {
      return;
    }

    nwThreadRunning = true;
    runLoop = true;
    std::thread nwt = std::thread(&ZmqJsonSubscriber::networkThreadFunc, this);
    nwt.detach();   // Needed, otherwise crashes
  }

  void onStop()
  {
    if (!nwThreadRunning)
    {
      return;
    }

    RLOG(0, "Stopping ZmqJsonSubscriber");
    runLoop = false;

    while (nwThreadRunning)
    {
      fprintf(stderr, ".");
      fflush(stderr);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }

  void networkThreadFunc()
  {
    zmq::context_t context(1);
    std::unique_ptr<zmq::socket_t> socket;

    try
    {
      socket = std::make_unique<zmq::socket_t>(context, ZMQ_SUB);

#if ZMQ_VERSION <= ZMQ_MAKE_VERSION(4, 3, 2)
      socket->setsockopt(ZMQ_RCVTIMEO, 3000); // Timeout in milliseconds
      socket->setsockopt(ZMQ_SUBSCRIBE, "", 0);
#else
      socket->set(zmq::sockopt::rcvtimeo, 3000);
      socket->set(zmq::sockopt::subscribe, "");
      socket->set(zmq::sockopt::conflate, 1); // Receive only last message
#endif

      socket->bind(ip_address); // Bind to all available interfaces

      // Little timeout to avoid getting queued messages
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    catch (const zmq::error_t& e)
    {
      std::cerr << "Error binding socket: " << e.what() << std::endl;
      nwThreadRunning = false;
      return;
    }

    // Flush the queue
    {
      int count = 0;
      zmq::message_t message;
      while (socket->recv(message, zmq::recv_flags::dontwait))
      {
        // Discard the message, effectively clearing the queue
        count++;
      }
      RLOG_CPP(0, "Flushed " << count << " messages");
    }


    while (runLoop)
    {
      zmq::message_t request;

      if (socket->recv(request, zmq::recv_flags::none))
      {
        std::string json_str(static_cast<char*>(request.data()), request.size());
        RLOG_CPP(1, "Received json:\n\n" << json_str);

        // Parse the JSON data with error handling
        nlohmann::json json_data;
        try
        {
          json_data = nlohmann::json::parse(json_str);

          if (json_data.contains("type"))
          {
            std::string msgType = json_data["type"];
            getEntity()->publish("ReceiveZMQ", msgType, json_str);
          }


        }
        catch (const nlohmann::json::parse_error& e)
        {
          RLOG_CPP(0, "JSON parsing error: " << e.what() << "\nReceived data: " << json_str);
        }

      }
      else
      {
        // Timeout occurred, handle it here
        RLOG_CPP(5, "No message received within timeout. Waiting...");
      }

    }

    nwThreadRunning = false;
    std::cout << "Quitting network thread" << std::endl;
  }

  /*******************************************************************************
   *
   ******************************************************************************/
  void onReceiveZMQ(std::string msgType, std::string json_str)
  {
    bool success = false;

    if (msgType=="wake_word" || msgType=="transcription")
    {
      success = handleASR(msgType, json_str);
    }


  }

  /*******************************************************************************
  *
  ******************************************************************************/
  bool handleASR(std::string msgType, std::string json_str)
  {
    RLOG_CPP(1, "Received: " << json_str);
    nlohmann::json json_data;

    try
    {
      json_data = nlohmann::json::parse(json_str);
    }
    catch (const nlohmann::json::parse_error& e)
    {
      RLOG_CPP(0, "JSON parsing error: " << e.what() << "\nReceived data: " << json_str);
      return false;
    }

    if (msgType == "wake_word")
    {
      getEntity()->publish<std::string, std::string>("RenderCommand", "BackgroundColor", std::string("GREEN"));
    }
    else if (msgType == "transcription")
    {
      if (json_data.contains("is_final") && json_data["is_final"].is_boolean())
      {
        bool isFinal = json_data["is_final"];

        if (isFinal)
        {
          std::string transcription = json_data["transcription"];
          RLOG_CPP(0, "transcription: " << transcription);
          if (!transcription.empty())
          {
            getEntity()->publish("EventReceived", transcription);
          }
        }
        else
        {
          std::cout << ".";
        }
        std::string color = isFinal ? "" : "DARKGREEN";
        getEntity()->publish<std::string, std::string>("RenderCommand", "BackgroundColor", color);
      }
      else
      {
        RLOG_CPP(0, "\"is_final\" field is missing or not a boolean.");
        return false;
      }
    }

    return true;
  }

};

}   // namespace
