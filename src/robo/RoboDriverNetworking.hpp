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

#include <Rcs_macros.h>

#include <zmq.hpp>

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <exception>
#include <sstream>

/*******************************************************************************
 * Class to send feedback message from driver process to the remote computer.
 * The driver proccess calls updateMessage() through the passed function object.
 * This will not be executed in the driver's process, but instead deferred to
 * the networking thread with a condition variable to avoid hitting timing limits
 * through networking. This is particularly an issue for high-frequency loops
 * (like the 1kHz loop examples in the Kortex library).
 *
 * Features:
 * - Checks if port has already bound and returns success from start() so that
 *   duplicate processes will be recognized.
 * - Due to the pub-sub communication pattern, there may be several processes
 *   receiving robot data, and several ones writing commands, at the same time.
 *   This can be convenient, but is not particularly safe.
 *******************************************************************************/
class FeedbackThread
{
public:
  FeedbackThread() = default;

  bool start(std::string endpoint, const std::atomic_bool& run_flag)
  {
    if (thread_.joinable())
    {
      RLOG(1, "FeedbackThread::start() called while thread is already running.");
      return false;
    }

    this->runLoop.store(true, std::memory_order_release);

    {
      std::lock_guard<std::mutex> lk(thread_init_mtx_);
      threadInitialized = false;             // reset before launching
    }

    thread_ = std::thread(&FeedbackThread::networkThreadFcn, this, endpoint, std::cref(run_flag));

    std::unique_lock<std::mutex> lk(thread_init_mtx_);
    bool ok = thread_init_cv_.wait_for(lk, std::chrono::seconds(1),
                                       [this] { return threadInitialized; });
    lk.unlock();

    if (!ok)
    {
      runLoop.store(false, std::memory_order_release);
      if (thread_.joinable())
      {
        thread_.join();
      }
      RLOG(1, "Didn't hear from FeedbackThread for 1 second - giving up");
      return false;
    }

    return true;
  }

  ~FeedbackThread()
  {
    stop();
  }

  void stop()
  {
    if (!runLoop.exchange(false))  // only stop once
    {
      RLOG(1, "Feedback thread already stopped");
      return;
    }

    cv_.notify_all();
    if (thread_.joinable())
    {
      RLOG(0, "Waiting for feedback thread to join");
      thread_.join();
      RLOG(0, "Feedback thread joined");
    }
    else
    {
      RLOG(0, "Feedback thread already stopped");
    }

  }

  void updateMessage(const std::string& new_msg)
  {
    static int count = -1;

    if (++count % 100 == 0)
    {
      RLOG_CPP(1, new_msg);
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      message_ = new_msg;
    }
    cv_.notify_one();
  }

private:

  void networkThreadFcn(std::string endpoint, const std::atomic_bool& run_flag)
  {
    zmq::context_t ctx(1);
    zmq::socket_t pub_socket(ctx, zmq::socket_type::pub);

    try
    {
      pub_socket.bind(endpoint);
      pub_socket.set(zmq::sockopt::sndhwm, 1000);   // prevent infinite queueing
      pub_socket.set(zmq::sockopt::linger, 0);      // fast socket shutdown
    }
    catch (const zmq::error_t& e)
    {
      std::ostringstream detailed_err;
      detailed_err << e.what() << " (errno=" << e.num() << ", " << zmq_strerror(e.num()) << ")";
      RLOG_CPP(0, "Exiting from FeedbackThread with error " << detailed_err.str());
      RLOG_CPP(0, "It seems that another process is already binding this port: " << endpoint <<
               " .Please make sure that no other RoboDriver is running");
      return;
    }
    catch (const std::exception& e)
    {
      RLOG_CPP(0, "Exception when binding port in FeedbackThread: " << e.what());
      return;
    }

    RLOG_CPP(0, "Feedback thread sending on '" << endpoint << "'");
    {
      std::lock_guard<std::mutex> lk(thread_init_mtx_);
      threadInitialized = true;              // set condition under the mutex
    }
    thread_init_cv_.notify_one();              // wake the starter immediately

    std::unique_lock<std::mutex> lock(mtx_);
    while (run_flag && runLoop)
    {
      cv_.wait(lock, [this,&run_flag] { return !run_flag || !runLoop || !message_.empty(); });

      // The condition variable will also be notified in case of quitting. In this case, we
      // quit the thread function without sending anything.
      if (!run_flag || !runLoop)
      {
        break;
      }

      std::string msg = message_;
      message_.clear();
      lock.unlock();

      try
      {
        pub_socket.send(zmq::buffer(msg), zmq::send_flags::none);
      }
      catch (const zmq::error_t& e)
      {
        RLOG_CPP(0, "ZMQ send error: " << e.what());
      }

      lock.lock();
    }

    runLoop = false;
    RLOG(0, "Quitting feedback thread");
  }

  std::string             message_;
  std::mutex              mtx_;
  std::condition_variable cv_;
  std::thread             thread_;
  std::atomic<bool> runLoop{false};

  std::mutex thread_init_mtx_;
  std::condition_variable thread_init_cv_;
  bool threadInitialized{false};
};


/*******************************************************************************
 * Receives commands from the remote host, and passes them to the robo driver
 * thread. There is no constraint on the frequency of the incoming commands,
 * they can come at any frequency (slower than the driver loop). Filtering is
 * done inside the driver's thread.
 *
 * The passed cmdFcn is responsible to implement the parsing of the commands.
 * If the cmdFcn returns true, the thread quits and stops all driver threads.
 * This allows to implement a quit-logic in the parsed commands.
 *******************************************************************************/
class CommandThread
{
public:
  CommandThread() = default;

  void start(std::string endpoint,
             std::function<bool(const std::string&)> cmdFcn,
             std::atomic_bool& run_flag,
             bool blocking=false)
  {
    if (thread_.joinable())
    {
      RLOG(1, "CommandThread::start() called but thread is already running.");
      return;
    }

    this-> runLoop = true;
    if (blocking)
    {
      networkLoop(endpoint, cmdFcn, run_flag);
    }
    else
    {
      thread_ = std::thread(&CommandThread::networkLoop, this,
                            endpoint, cmdFcn, std::ref(run_flag));
    }

  }

  void stop()
  {
    if (!runLoop.exchange(false))  // only stop once
    {
      RLOG(1, "Command thread already stopped");
      return;
    }

    if (thread_.joinable())
    {
      RLOG(0, "Waiting for command thread to join");
      thread_.join();
      RLOG(0, "Command thread joined");
    }
    else
    {
      RLOG(0, "Command thread already stopped");
    }

  }


private:

  void networkLoop(std::string endpoint,
                   std::function<bool(const std::string&)> cmdFcn,
                   std::atomic_bool& run_flag)
  {
    RLOG(0, "Command receiver thread running");
    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, zmq::socket_type::sub);
    sub.set(zmq::sockopt::subscribe, "");
    sub.set(zmq::sockopt::rcvtimeo, 100); // recv timeout 100ms to enable stopping
    sub.bind(endpoint);

    while (run_flag && runLoop)
    {
      zmq::message_t msg;

      try
      {
        zmq::recv_result_t result = sub.recv(msg, zmq::recv_flags::none);

        if (result)
        {
          std::string msg_str(static_cast<const char*>(msg.data()), msg.size());
          bool quitMe = cmdFcn(msg_str);  // thread-safe

          if (quitMe)
          {
            run_flag.store(false, std::memory_order_release);
            runLoop.store(false, std::memory_order_release);
          }
        }

      }
      catch (const zmq::error_t& e)
      {
        if (e.num() == EINTR)
        {
          RLOG_CPP(1, "ZMQ recv interrupted by signal, shutting down" << e.what());
          break;  // graceful shutdown
        }
        else
        {
          RLOG_CPP(0, "ZMQ error: " << e.what());
        }
      }
      catch (const std::exception& e)
      {
        RLOG_CPP(0, "JSON error: " << e.what());
      }

    }

    runLoop = false;
    RLOG(0, "Quitting command thread");
  }

  std::thread thread_;
  std::atomic<bool> runLoop{false};
};
