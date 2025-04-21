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

// ──────────────────────────────────────────────────────────────
// router.cpp – central coordinator
//
// • ROUTER socket ← receives messages / heart‑beats from workers
// • Tracks liveness per worker (ID ➜ last‑seen time)
// • Sends a JSON “do_work” command to every *alive* worker
//   every COMMAND_INTERVAL_MS milliseconds
// • Drops (and logs) workers that miss HEARTBEAT_LIVENESS ms
//
// Build:  g++ router.cpp -I/Users/mgienger/Software/AttentiveSupport/src/Smile/src/AffAction/external -I/opt/homebrew/include -std=c++17 \
//           -L/opt/homebrew/Cellar/zeromq/4.3.5_1/lib -lzmq -o router
// Run  :  ./router
// ──────────────────────────────────────────────────────────────
#include "ZmqRouterComponent.h"

#include <Rcs_macros.h>

#include <zmq.hpp>
#include <chrono>
#include <unordered_map>
#include <iostream>
#include <iomanip>
#include <json.hpp>      // header‑only JSON (https://github.com/nlohmann/json)

namespace aff
{
using Clock = std::chrono::steady_clock;
using ms    = std::chrono::milliseconds;
using json  = nlohmann::json;

// ───────────────────────── configurable constants
constexpr int  POLL_TIMEOUT_MS      = 100;   // main‑loop poll period
constexpr int  HEARTBEAT_LIVENESS   = 6000;  // ms without heartbeat → drop worker
constexpr int  COMMAND_INTERVAL_MS  = 50;    // broadcast command every n ms
constexpr char ROUTER_ENDPOINT[]    = "tcp://*:5555";
// ──────────────────────────────────────────────────────────────

// log with wall‑clock timestamp
inline void log(const std::string& msg)
{
  auto now = std::chrono::system_clock::now();
  auto itt = std::chrono::system_clock::to_time_t(now);
  auto tm  = *std::localtime(&itt);
  std::cout << std::put_time(&tm, "%F %T") << " | " << msg << '\n';
}

ZmqRouterComponent::ZmqRouterComponent(EntityBase* parent, std::string connection):
  ComponentBase(parent), connectionStr(connection), threadRunning(false), threadFunctionCompleted(false)
{
  subscribe("Start", &ZmqRouterComponent::startZmqThread);
  subscribe("Stop", &ZmqRouterComponent::stopZmqThread);
}

ZmqRouterComponent::~ZmqRouterComponent()
{
  if (threadRunning)
  {
    RLOG(0, "Thread still running in destructor - stopping it now.");
    stopZmqThread();
    RLOG(0, "Thread stopped.");
  }
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

  RLOG(0, "startZmqThread()");
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
    RLOG(0, "Thread already stopped");
    return;
  }

  RLOG(0, "Trying to stop thread");
  threadRunning = false;

  // See startZmqThread() why we don't join the thread here as one would expect.
  // mpThread.join();
  // RLOG(0, "Thread joined");
  while (!threadFunctionCompleted)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RLOG(0, "onStop() completed");
}

void ZmqRouterComponent::zmqThreadFunc(const std::string& connection)
{
  // ZeroMQ context & socket setup
  zmq::context_t ctx{1};
  zmq::socket_t  router{ctx, zmq::socket_type::router};

  const int SND_HWM = 150;                       // keep at most 150 unsent cmds / worker
  router.set(zmq::sockopt::sndhwm, SND_HWM);
  router.set(zmq::sockopt::router_mandatory, 1); // detect overflow instead of silent drop
  router.set(zmq::sockopt::sndtimeo, 0);         // non‑blocking sends

  router.bind(ROUTER_ENDPOINT);
  log(std::string("ROUTER bound to ") + ROUTER_ENDPOINT);

  // State: worker‑id  → last‑heartbeat‑time
  std::unordered_map<std::string, Clock::time_point> workers;

  Clock::time_point lastCmd = Clock::now();

  // Network loop
  while (this->threadRunning)
  {
    // ── poll for inbound messages
    zmq::pollitem_t items[] = { { router, 0, ZMQ_POLLIN, 0 } };
    zmq::poll(items, 1, std::chrono::milliseconds{POLL_TIMEOUT_MS});

    // ───────────────────────────────────────── inbound messages
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
        if (!idRes) break;                       // queue empty → done

        // Part 2: empty delimiter (REQ/ROUTER convention)
        auto emptyRes = router.recv(empty, zmq::recv_flags::dontwait);
        if (!emptyRes || empty.size() != 0)      // malformed message
        {
          log("[WARN] Incomplete multipart message (missing empty frame)");
          break;
        }

        // Part 3: payload
        auto msgRes = router.recv(payload, zmq::recv_flags::dontwait);
        if (!msgRes)
        {
          log("[WARN] Incomplete multipart message (missing payload)");
          break;
        }

        std::string id(static_cast<char*>(identity.data()), identity.size());
        std::string data(static_cast<char*>(payload.data()), payload.size());

        workers[id] = Clock::now();              // refresh liveness
        log("[RECV] from " + id + " → " + data);

        // Optionally parse / act on non‑heartbeat replies here
        // json msg = json::parse(data, nullptr, false);
        getEntity()->publish("ZmqDealerMessage", id, data);
      }
    }

    // ───────────────────────────────────────── broadcast command
    auto now = Clock::now();
    if (std::chrono::duration_cast<ms>(now - lastCmd).count() >= COMMAND_INTERVAL_MS)
    {
      json cmd =
      {
        { "type", "do_work" },
        { "ts",   std::chrono::duration_cast<ms>(now.time_since_epoch()).count() }
      };
      std::string cmdStr = cmd.dump();

      for (auto& worker : workers)
      {
        auto& id = worker.first;
        auto& last = worker.second;

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
            log("[DROP] back‑pressure: queue full for " + id);
          }
          else
          {
            log("[SEND] to   " + id + " → " + cmdStr);
          }
        }
        catch (const zmq::error_t& e)
        {
          if (e.num() == EHOSTUNREACH)
          {
            log("[DROP] no route to worker " + id);
            last = Clock::time_point{};      // mark as timed‑out
          }
          else
          {
            log(std::string("[ERROR] send failed for ") + id + ": " + e.what());
          }
        }
      }

      lastCmd = now;
    }

    // ───────────────────────────────────────── liveness sweep
    for (auto it = workers.begin(); it != workers.end();)
    {
      if (std::chrono::duration_cast<ms>(now - it->second).count() > HEARTBEAT_LIVENESS)
      {
        log("[DROP] worker " + it->first + " timed‑out");
        it = workers.erase(it);
      }
      else ++it;
    }
  }
}

}   // nymespace
