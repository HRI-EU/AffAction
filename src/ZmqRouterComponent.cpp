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

 Zmq router – central coordinator

 - ROUTER socket receives messages and heart‑beats from workers
 - Tracks liveness per worker (ID, last‑seen time)
 - Sends a JSON “do_work” command to every alive worker
   every COMMAND_INTERVAL_MS milliseconds
 - Drops (and logs) workers that miss HEARTBEAT_LIVENESS ms

 *******************************************************************************/

#include "ZmqRouterComponent.h"

#include <Rcs_macros.h>

#include <json.hpp>
#include <zmq.hpp>
#include <chrono>
#include <unordered_map>
#include <iostream>
#include <iomanip>


namespace aff
{
using Clock = std::chrono::steady_clock;
using ms    = std::chrono::milliseconds;

constexpr int  POLL_TIMEOUT_MS      = 100;   // main‑loop poll period
constexpr int  HEARTBEAT_LIVENESS   = 6000;  // ms without heartbeat: drop worker
constexpr int  COMMAND_INTERVAL_MS  = 50;    // broadcast command every n ms


ZmqRouterComponent::ZmqRouterComponent(EntityBase* parent, std::string connection):
  ComponentBase(parent), LandmarkBase(),
  connectionStr(connection), threadRunning(false), threadFunctionCompleted(false)
{
  subscribe("Start", &ZmqRouterComponent::startZmqThread);
  subscribe("Stop", &ZmqRouterComponent::stopZmqThread);
  subscribe("SetPerceptionCommand", &ZmqRouterComponent::onSetPerceptionCommand);
  subscribe("TriggerPerception", &ZmqRouterComponent::onTriggerPerception);

  subscribe("UpdateScene", &LandmarkBase::onUpdateScene);
  subscribe("FreezePerception", &LandmarkBase::onFreezePerception);
  subscribe("EstimateCameraPose", &LandmarkBase::estimateCameraPose);
  subscribe("EnableDebugGraphics", &LandmarkBase::enableDebugGraphics);
  
  getEntity()->subscribe("Speak", [this](std::string text) mutable
            {
    nlohmann::json payload =
    {
      {"type", "tts"},
      {"cmd",  "SAY"},
      {"text", text}
    };

    getEntity()->publish("TriggerPerception", std::string("tts"), 0, payload.dump());
  });
}

ZmqRouterComponent::~ZmqRouterComponent()
{
  if (threadRunning)
  {
    stopZmqThread();
  }
}

void ZmqRouterComponent::onSetPerceptionCommand(std::string command, int repetitions)
{
  RLOG_CPP(1, "command: " << command << " repetitions: " << repetitions);
  std::lock_guard<std::mutex> lock(commandMtx);
  commandQueue.push({command, repetitions, ""});
}

void ZmqRouterComponent::onTriggerPerception(std::string target_id, int repetitions, std::string jsonString)
{
  RLOG_CPP(1, "target_id: " << target_id << " repetitions: " << repetitions << " json: " << jsonString);
  std::lock_guard<std::mutex> lock(commandMtx);
  commandQueue.push({ target_id, repetitions, jsonString });
}

std::string ZmqRouterComponent::getName() const
{
  return "ZmqRouterComponent";
}

void ZmqRouterComponent::startZmqThread()
{
  if (threadRunning)
  {
    RLOG(1, "Thread already running");
    return;
  }

  RLOG(1, "startZmqThread()");
  threadRunning = true;
  zmqThread = std::thread(&ZmqRouterComponent::zmqThreadFunc, this, connectionStr);

  // Ideally, we should join it in the onStop() function. For some reasons,
  // this does not work on all platforms, maybe due to some dangling zmq
  // class that inhibits joining the thread somehow. This only is the case
  // once we run into a receive timeout. We therefore detach the thread here.
  zmqThread.detach();
}

void ZmqRouterComponent::stopZmqThread()
{
  if (!threadRunning)
  {
    RLOG(5, "Thread already stopped");
    return;
  }

  threadRunning = false;

  // See startZmqThread() why we don't join the thread here as one would expect.
  // mpThread.join();
  // RLOG(0, "Thread joined");
  int stopCount = 0;
  while (!threadFunctionCompleted)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    stopCount++;

    if (stopCount>50)
    {
      RLOG(0, "Still trying to stop ZmqRouterComponent after %.1f seconds ...",
           0.1*stopCount);
    }
  }

}

static zmq::socket_t create_router_socket(zmq::context_t& ctx, const std::string& connection)
{
  try
  {
    zmq::socket_t router(ctx, zmq::socket_type::router);

    const int SND_HWM = 150;  // max unsent messages per peer

    router.set(zmq::sockopt::sndhwm, SND_HWM);
    router.set(zmq::sockopt::router_mandatory, 1);
    router.set(zmq::sockopt::sndtimeo, 0); // non-blocking send

    router.bind(connection);

    RLOG_CPP(1, "ROUTER bound to " << connection);
    return router;
  }
  catch (const zmq::error_t& e)
  {
    RLOG_CPP(0, "ZeroMQ error during router setup with connection '" << connection << "': " << e.what());
    throw;
  }
  catch (const std::exception& e)
  {
    RLOG_CPP(0, "General exception during router setup with connection '" << connection << "': " << e.what());
    throw;
  }
}


void ZmqRouterComponent::zmqThreadFunc(const std::string& connection)
{
  this->threadFunctionCompleted = false;

  // ZeroMQ context & socket setup
  zmq::context_t ctx{1};
  zmq::socket_t router = create_router_socket(ctx, connection);

  // State: worker‑id, last‑heartbeat‑time
  std::unordered_map<std::string, Clock::time_point> workers;

  Clock::time_point lastCmd = Clock::now();

  // Network loop
  while (this->threadRunning)
  {
    // Poll for inbound messages
    zmq::pollitem_t items[] = { { router, 0, ZMQ_POLLIN, 0 } };
    zmq::poll(items, 1, std::chrono::milliseconds{POLL_TIMEOUT_MS});

    // Inbound messages
    if (items[0].revents & ZMQ_POLLIN)
    {
      // Drain *all* queued messages to avoid backlog
      for (;;)
      {
        zmq::message_t identity;
        zmq::message_t empty;
        zmq::message_t payload;

        // Part 1: identity frame
        auto idRes = router.recv(identity, zmq::recv_flags::dontwait);
        if (!idRes)
        {
          break;  // queue empty: done
        }

        // Part 2: empty delimiter (REQ/ROUTER convention)
        zmq::recv_result_t emptyRes = router.recv(empty, zmq::recv_flags::dontwait);
        if (!emptyRes || empty.size() != 0)      // malformed message
        {
          RLOG_CPP(0, "[WARN] Incomplete multipart message (missing empty frame)");

          if (empty.size() == 0)
          {
            RLOG_CPP(0, "[Empty frame]");
          }
          else
          {
            std::string s(static_cast<char*>(empty.data()), empty.size());
            RLOG_CPP(0, "Frame as string: \"" << s << "\"");
          }

          break;
        }

        // Part 3: payload
        auto msgRes = router.recv(payload, zmq::recv_flags::dontwait);
        if (!msgRes)
        {
          RLOG_CPP(0, "[WARN] Incomplete multipart message (missing payload)");
          break;
        }

        std::string id(static_cast<char*>(identity.data()), identity.size());
        std::string payLoadStr(static_cast<char*>(payload.data()), payload.size());
        if (workers.find(id) == workers.end())
        {
          // Found first occurrence
          RLOG_CPP(0, "Worker found for the first time: " << id);
        }
        workers[id] = Clock::now();              // refresh liveness

        try
        {
          nlohmann::json json = nlohmann::json::parse(payLoadStr);
          setJsonInput(json);
          getEntity()->publish("ZmqDealerMessage", id, payLoadStr);
        }
        catch (const nlohmann::json::parse_error& e)
        {
          RLOG_CPP(0, "[JSON parse error] at byte " << e.byte << ": " << e.what());
        }

      }
    }

    // Broadcast command
    auto now = Clock::now();
    if (std::chrono::duration_cast<ms>(now - lastCmd).count() >= COMMAND_INTERVAL_MS)
    {
      // Process commands
      std::string cmdStr, id_str;
      {
        std::lock_guard<std::mutex> lock(commandMtx);

        if (!commandQueue.empty())
        {
          std::tuple<std::string, int, std::string> cmdPair = commandQueue.front();
          nlohmann::json cmd =
          {
            { "type", std::get<0>(cmdPair)},
            { "repetitions", std::get<1>(cmdPair) },
            { "ts",   std::chrono::duration_cast<ms>(now.time_since_epoch()).count() }
          };

          if (!std::get<2>(cmdPair).empty())
          {
            auto bbJson = nlohmann::json::parse(std::get<2>(cmdPair));
            cmd.update(bbJson);
          }
          cmdStr = cmd.dump();
          id_str = std::get<0>(cmdPair);
          commandQueue.pop();
          RLOG_CPP(1, "cmsjson:\n" << cmdStr);
        }

      }

      RLOG_CPP(5, "Going through " << workers.size() << " workers");
      if (!cmdStr.empty())
      {
        for (auto& worker : workers)
        {
          auto& id = worker.first;
          auto& last = worker.second;

          if (id != id_str)
          {
            continue;
          }

          // multipart: [identity][empty][payload]
          zmq::message_t idMsg(id.data(), id.size());
          zmq::message_t empty;                    // zero‑length delimiter
          zmq::message_t body(cmdStr.data(), cmdStr.size());

          try
          {
            auto ok1 = router.send(idMsg, zmq::send_flags::sndmore | zmq::send_flags::dontwait);
            auto ok2 = router.send(empty, zmq::send_flags::sndmore | zmq::send_flags::dontwait);
            auto ok3 = router.send(body, zmq::send_flags::dontwait);

            if (!ok1 || !ok2 || !ok3)
            {
              RLOG_CPP(0, "[DROP] back‑pressure: queue full for " << id);
            }
            else
            {
              RLOG_CPP(1, "[SEND] to   " << id + " -> " << cmdStr);
            }
          }
          catch (const zmq::error_t& e)
          {
            if (e.num() == EHOSTUNREACH)
            {
              RLOG_CPP(0, "[DROP] no route to worker " << id);
              last = Clock::time_point{};      // mark as timed‑out
            }
            else
            {
              RLOG_CPP(0, "[ERROR] send failed for " << id << ": " << e.what());
            }
          }
        }
      }

      lastCmd = now;
    }

    // Liveness sweep
    for (auto it = workers.begin(); it != workers.end();)
    {
      if (std::chrono::duration_cast<ms>(now - it->second).count() > HEARTBEAT_LIVENESS)
      {
        RLOG_CPP(0, "[DROP] worker " << it->first << " timed out");
        it = workers.erase(it);
      }
      else
      {
        ++it;
      }
    }

  }   // while (threadRunning)

  threadFunctionCompleted = true;
}

}   // namespace
