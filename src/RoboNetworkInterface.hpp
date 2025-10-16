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

#ifndef AFF_ROBONETWORKINTERFACE_H
#define AFF_ROBONETWORKINTERFACE_H


#include <Rcs_macros.h>

#include <zmq.hpp>

#include <string>
#include <iostream>
#include <thread>
#include <chrono>



namespace aff
{

/*******************************************************************************
 * Remote-side network interface
 ******************************************************************************/
class RoboNetworkInterface
{
public:

  RoboNetworkInterface(std::string otherRecv, std::string otherSend, double dt_commands) :
    context(1),   // 1 = one I/O thread
    otherRecvEndpoint(otherRecv),
    otherSendEndpoint(otherSend),
    senderCommandPeriod(dt_commands)
  {
    RCHECK(senderCommandPeriod > 0.0);
  }

  virtual ~RoboNetworkInterface()
  {
    stop();
  }

  virtual void start()
  {
    if (runLoop.load(std::memory_order_acquire))
    {
      RLOG(0, "RoboNetworkInterface already started - doing nothing");
      return;
    }

    runLoop.store(true, std::memory_order_release);
    watchDogTriggered.store(false, std::memory_order_release);
    recv_thread = std::thread(&RoboNetworkInterface::recvThreadFunc, this);

    RLOG_CPP(0, "RoboNetworkInterface: Waiting for message from " << otherSendEndpoint);
    size_t waitCount = 0;
    while (!isInitialized.load(std::memory_order_acquire))
    {
      fprintf(stderr, ".");
      fflush(stderr);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      waitCount++;

      if (waitCount > 100)
      {
        RLOG_CPP(0, "Didn't hear from robot for 10 seconds - giving up");
        return;
      }
    }

    send_thread = std::thread(&RoboNetworkInterface::sendThreadFunc, this);
    RLOG(0, "RoboNetworkInterface: All threads started");
  }

  virtual void stop()
  {
    if (!runLoop.load(std::memory_order_acquire))
    {
      RLOG(0, "RoboNetworkInterface already stopped - doing nothing");
      return;
    }

    runLoop.store(false, std::memory_order_release);

    if (send_thread.joinable())
    {
      send_thread.join();
      RLOG(0, "Joined socket send thread");
    }

    if (recv_thread.joinable())
    {
      recv_thread.join();
      RLOG(0, "Joined socket receive thread");
    }

    RLOG(0, "RoboNetworkInterface stopped");
  }

protected:

  void recvThreadFunc()
  {
    RLOG_CPP(0, "Listening to robot feedback on " << otherRecvEndpoint);
    try
    {
      const double max_timeout = 2.0;   // seconds
      double t_watchdog = getMonotonicTimeSeconds();
      zmq::socket_t sub(context, zmq::socket_type::sub);

      sub.set(zmq::sockopt::subscribe, "");
      sub.set(zmq::sockopt::rcvtimeo, 100);   // 100 ms timeout to catch runLoop
      sub.set(zmq::sockopt::linger, 0);
      sub.connect(otherRecvEndpoint);

      while (runLoop && !watchDogTriggered)
      {
        zmq::message_t msg;
        if (sub.recv(msg, zmq::recv_flags::none))
        {
          t_watchdog = getMonotonicTimeSeconds();
          std::string msg_str(static_cast<char*>(msg.data()), msg.size());
          const bool dataOk = process_incoming_message(msg_str);

          if (dataOk)
          {
            isInitialized.store(true, std::memory_order_release);
          }
          else
          {
            RLOG_CPP(0, "Incoming data not ok: " << msg_str);
          }
        }

        if ((getMonotonicTimeSeconds() - t_watchdog > max_timeout) &&
            (isInitialized.load(std::memory_order_acquire)))
        {
          RLOG(0, "Watchdog triggered - robot disconnected");
          watchDogTriggered = true;
        }

      }
    }
    catch (const std::exception& e)
    {
      RLOG_CPP(0, "Recv thread terminating: " << e.what());
    }

    RLOG(0, "Exiting recvThreadFunc()");
  }


  void sendThreadFunc()
  {
    zmq::socket_t pub_socket(context, zmq::socket_type::pub);
    pub_socket.set(zmq::sockopt::sndhwm, 10);   // drop if queued more than sndhwm
    pub_socket.set(zmq::sockopt::linger, 0);    // dont block on close, but loose messages
    pub_socket.connect(otherSendEndpoint);

    // Convert the double (seconds) to the clock's native duration
    using Clock = std::chrono::steady_clock;
    const Clock::duration period = std::chrono::duration_cast<Clock::duration>(
                                     std::chrono::duration<double>(senderCommandPeriod));
    Clock::time_point next_tick = Clock::now() + period;
    size_t loopCount = 0;


    while (runLoop && !watchDogTriggered)
    {
      std::string cmdJson = generate_command_message();

      if (!cmdJson.empty())
      {
        try
        {
          pub_socket.send(zmq::buffer(cmdJson), zmq::send_flags::none);
        }
        catch (const zmq::error_t& e)
        {
          RLOG_CPP(0, "ZMQ send error: " << e.what());
        }
      }

      // Wait until next cycle: absolute tick avoids drift
      next_tick += period;

      // Sleep until then if we're early, otherwise handle overrun
      auto now = Clock::now();
      if (now < next_tick)
      {
        std::this_thread::sleep_until(next_tick);
      }
      else
      {
        // Missed the deadline.
        double dt_over = std::chrono::duration<double>(now - next_tick).count();

        if (dt_over > senderCommandPeriod)
        {
          RLOG_CPP(0, "Missed command period to " << otherRecvEndpoint << " - resyncing " << loopCount <<
                   " overflow[msec]: " << std::fixed << std::setprecision(6) << 1000.0 * dt_over);
        }
      }

      next_tick = now + period;
      loopCount++;
    }   // while ...



    // Before we quit the command sender thread, we sened a final quit command
    // to the drivers so that they shut down gracefully
    try
    {
      pub_socket.send(zmq::buffer("{ \"quit\": true}"), zmq::send_flags::none);
    }
    catch (const zmq::error_t& e)
    {
      RLOG_CPP(0, "ZMQ send error on quit command: " << e.what());
    }

    RLOG(0, "Exiting sendThreadFunc()");
  }

  static inline double getMonotonicTimeSeconds() noexcept
  {
    return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
  }



protected:

  RoboNetworkInterface(const RoboNetworkInterface&)            = delete;
  RoboNetworkInterface& operator=(const RoboNetworkInterface&) = delete;
  RoboNetworkInterface(RoboNetworkInterface&&)                 = delete;
  RoboNetworkInterface& operator=(RoboNetworkInterface&&)      = delete;

  virtual bool process_incoming_message(const std::string& recv_msg) = 0;
  virtual std::string generate_command_message() = 0;

  mutable std::atomic<bool> isInitialized{false};
  std::atomic<bool> watchDogTriggered{false};
  std::atomic<bool> runLoop{false};
  std::thread recv_thread;
  std::thread send_thread;
  zmq::context_t context;
  std::string otherRecvEndpoint;
  std::string otherSendEndpoint;
  double senderCommandPeriod;
};

}   // namespace


#endif
