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

#include "json.hpp"

#include <Rcs_filters.h>
#include <Rcs_macros.h>
#include <Rcs_basicMath.h>
#include <Rcs_VecNd.h>

#include <thread>
#include <mutex>
#include <chrono>

#if defined (AFFACTION_WITH_KINOVA_GEN3)

#if defined (_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4146)
#pragma warning(disable : 4244)
#endif

#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#if defined (_MSC_VER)
#pragma warning(pop)
#endif

namespace k_api = Kinova::Api;

#else

#include <windows.h>

#endif //AFFACTION_WITH_KINOVA_GEN3


constexpr std::size_t   DOF_ARM        = 7;   // Gen3 R-07
constexpr std::uint16_t TCP_PORT   = 10000;   // high-level services
constexpr std::uint16_t UDP_PORT   = 10001;   // BaseCyclic feedback

#define MINIMAL_GRIPPER_POSITION_ERROR  ((double)1.5)
#define MINIMAL_GRIPPER_VELOCITY  ((double)0.75)


/*******************************************************************************
 * Time in seconds from epoch
 ******************************************************************************/
static double getWallclockTime()
{
  auto currentTime = std::chrono::system_clock::now();
  double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime.time_since_epoch()).count();
  return seconds;
}

/*******************************************************************************
 * Angle wrapping helpers
 ******************************************************************************/
// wrap angle (deg) to [0,360)
inline double wrap360(double d)
{
  double m = std::fmod(d, 360.0);
  return (m < 0.0) ? m + 360.0 : m;
}

// shortest signed arc a-b in (-180,180] deg
inline double signed_diff_deg(double a_deg, double b_deg)
{
  double d = std::fmod(a_deg - b_deg, 360.0);
  if (d <= -180.0)
  {
    d += 360.0;
  }
  if (d  >  180.0)
  {
    d -= 360.0;
  }
  return d;
}


inline std::vector<double>
closest_to_default(const std::vector<double>& init,
                   const std::vector<double>& def)
{
  std::vector<double> out;
  out.reserve(init.size());
  for (std::size_t i = 0; i < init.size(); ++i)
  {
    out.push_back(def[i] + signed_diff_deg(init[i], def[i]));
  }
  return out;
}

/*******************************************************************************
 * Cross-platform microsecond timer
 ******************************************************************************/
static int64_t GetTickUs()
{
#if defined(_MSC_VER)
  LARGE_INTEGER start, frequency;

  QueryPerformanceFrequency(&frequency);
  QueryPerformanceCounter(&start);

  return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
  struct timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);

  return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}


/*******************************************************************************
 * Polling timer class
 ******************************************************************************/
class PollingTimer
{
public:

  enum class TimerMode
  {
    Poll, Sleep, SleepAndPoll
  };

  explicit PollingTimer(double dt_seconds,
                        TimerMode timerMode_=TimerMode::Poll,
                        std::chrono::microseconds guard = std::chrono::microseconds(200))
    : interval_us_(static_cast<int64_t>(dt_seconds * 1e6)),
      last_tick_us_(GetTickUs()),
      initial_tick_us_(last_tick_us_),
      timer_count_(0),
      timerMode(timerMode_),
      steady_interval_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(dt_seconds))),
      next_steady_(std::chrono::steady_clock::now()),
      guard_(guard)                                    // how long we allow for the final spin
  {
  }

  int64_t getWaitCycleCount() const
  {
    return timer_count_;
  }

  int64_t getTickUs() const
  {
    return GetTickUs() - initial_tick_us_;
  }

  double getTimeSinceStart() const
  {
    return 1.0e-6*(GetTickUs() - initial_tick_us_);
  }

  void wait()
  {
    switch (timerMode)
    {
      case TimerMode::Poll:
        waitPoll();
        break;

      case TimerMode::Sleep:
        waitSleep();
        break;

      case TimerMode::SleepAndPoll:
        waitSleepAndPoll();
        break;

      default:
        RFATAL("Timermode unknown");
    }

    timer_count_++;
  }

private:

  // Busy-wait loop with optional throttling
  void waitPoll()
  {
    while (true)
    {
      int64_t now = GetTickUs();
      if (now - last_tick_us_ >= interval_us_)
      {
        last_tick_us_ = now;
        break;
      }

    }
  }

  // Drift-free version using steady_clock and sleep_until
  void waitSleep()
  {
    next_steady_ += steady_interval_;
    std::this_thread::sleep_until(next_steady_);
  }

  void waitSleepAndPoll()
  {

    // 1) book the next absolute wake-up time
    next_steady_ += steady_interval_;

    // 2) sleep until guard time before the target
    auto target_minus_guard = next_steady_ - guard_;

    // If guard_ is larger than the period, clamp it.
    if (target_minus_guard > std::chrono::steady_clock::now())
    {
      std::this_thread::sleep_until(target_minus_guard);
    }

    // 3) spin for the remaining few microseconds for sub-millisecond accuracy
    while (std::chrono::steady_clock::now() < next_steady_)
      ;  // tight loop, but only for guard_ us
  }

  // For busy-wait
  int64_t interval_us_;
  int64_t last_tick_us_;
  int64_t initial_tick_us_;
  int64_t timer_count_;

  TimerMode timerMode;

  // For steady-clock loop
  std::chrono::steady_clock::duration steady_interval_;
  std::chrono::steady_clock::time_point next_steady_;
  std::chrono::microseconds guard_;   // max spin duration at the tail
};


/*******************************************************************************
 *
 ******************************************************************************/

class KortexDriver
{
public:

  KortexDriver() = default;
  KortexDriver(const KortexDriver& other) = delete;

  void start(std::string ip_address,
             std::function<void(const std::string&)> feedbackFcn,
             const std::atomic_bool& run_flag,
             bool readOnly,
             std::string controlMode,
             std::vector<double> q_default)
  {
    if (kortexThread.joinable())
    {
      RLOG(0, "KortexDriver already running - not starting new thread");
      return;
    }

    this->runLoop = true;

    if (controlMode == "TestWithoutRobot")
    {
      this->kortexThread = std::thread(&KortexDriver::roboThreadFuncTest, this,
                                       ip_address, readOnly,
                                       feedbackFcn, std::cref(run_flag), q_default);
    }
#if defined (AFFACTION_WITH_KINOVA_GEN3)
    else if (controlMode == "HighLevel")
    {
      this->kortexThread = std::thread(&KortexDriver::roboThreadFuncHighLevel, this,
                                       ip_address, readOnly,
                                       feedbackFcn, std::cref(run_flag));
    }
    else if (controlMode == "LowLevel")
    {
      this->kortexThread = std::thread(&KortexDriver::roboThreadFuncLowLevel, this,
                                       ip_address, readOnly,
                                       feedbackFcn, std::cref(run_flag), q_default);
    }
#endif
    else
    {
      RFATAL("Unknown control mode: %s", controlMode.c_str());
    }

    RLOG(0, "Waiting for robot ...");
    while (!this->isInitialized)
    {
      fprintf(stderr, ".");
      fflush(stderr);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RLOG(0, "Kortex thread running");
  }

  void stop()
  {
    this->runLoop = false;

    RLOG(0, "Waiting for robo thread to stop");
    if (kortexThread.joinable())
    {
      kortexThread.join();
    }
    RLOG(0, "Robo thread stopped");
  }

  struct RoboCommand
  {
    RoboCommand() : gripper_pos(0.0), gripper_force(100.0),
      received_gripper_pos(false),
      received_gripper_force(false),
      received_q_des(false)
    {
    }

    double gripper_pos;
    double gripper_force;

    std::array<double,DOF_ARM> q_des{};

    bool received_gripper_pos;
    bool received_gripper_force;
    bool received_q_des;
  };

  bool setCommand(const std::string& message)
  {
    auto j = nlohmann::json::parse(message);
    KortexDriver::RoboCommand cmd;

    if (j.contains("q_des"))
    {
      // We receive radians. Degrees are only used internally.
      cmd.q_des = j["q_des"].get<std::array<double, DOF_ARM>>();
      cmd.received_q_des = true;
      VecNd_constMulSelf(cmd.q_des.data(), 180.0/M_PI, DOF_ARM);
    }

    if (j.contains("gripper_command"))
    {
      cmd.gripper_pos = j["gripper_command"].get<double>();
      cmd.received_gripper_pos = true;
    }

    if (j.contains("gripper_force"))
    {
      cmd.gripper_force = j["gripper_force"].get<double>();
      cmd.received_gripper_force = true;
    }

    bool quitMe = j.value("quit", false);

    // thread-safe
    std::lock_guard<std::mutex> lock(cmdMtx);
    this->incomingCommand = cmd;

    return quitMe;
  }

  RoboCommand getCommand() const
  {
    std::lock_guard<std::mutex> lock(cmdMtx);
    return this->incomingCommand;
  }


private:

  void roboThreadFuncTest(const std::string& ip,
                          bool readOnly,
                          std::function<void(const std::string&)> feedbackFcn,
                          const std::atomic_bool& run_flag,
                          std::vector<double> q_default_deg)
  {
    RLOG(0, "Starting roboThreadFuncTest");
    constexpr double dt = 0.01;
    constexpr double tmc = 0.05;
    constexpr int filter_substeps = 20;
    constexpr double dt_filter = dt / (double)filter_substeps;
    std::vector<double> maxVelInDeg = { 75.0, 75.0, 75.0, 75.0, 60.0, 60.0, 60.0 };

    // simple mockup state
    std::vector<double> q_curr_deg(DOF_ARM, 0.0);     // position  [deg]
    std::vector<double> qd_des_deg(DOF_ARM, 0.0);

    if (q_default_deg.empty())
    {
      q_default_deg = q_curr_deg;
    }
    else
    {
      q_curr_deg = q_default_deg;
      VecNd_addRandom(q_curr_deg.data(), -10.0, 10.0, DOF_ARM);
    }

    RCHECK(q_default_deg.size() == DOF_ARM);

    // Initialize continuous angles close to default pose
    std::vector<double> q_cont_deg = closest_to_default(q_curr_deg, q_default_deg);

    // Initialize filter with robot's continuous state
    std::unique_ptr<Rcs::RampFilterND> filteredJointCommands =
      std::make_unique<Rcs::RampFilterND>(tmc, 0.0, dt_filter, DOF_ARM);
    filteredJointCommands->init(q_cont_deg.data());
    for (size_t i = 0; i < q_cont_deg.size(); ++i)
    {
      filteredJointCommands->setMaxVel(maxVelInDeg[i], i);
      this->incomingCommand.q_des[i] = q_cont_deg[i];
      if (q_cont_deg[i] != q_curr_deg[i])
      {
        RLOG(0, "continuous angles adjusted at index %zu: curr: %f   cont: %f   default: %f",
             i, q_curr_deg[i], q_cont_deg[i], q_default_deg[i]);
      }
    }

    // everything is ready - tell main thread
    isInitialized.store(true, std::memory_order_release);

    PollingTimer timer(dt, PollingTimer::TimerMode::SleepAndPoll);

    while (run_flag && runLoop)
    {
      RoboCommand cmd = getCommand();

      if (cmd.received_q_des)
      {
        filteredJointCommands->setTarget(cmd.q_des.data());
      }

      for (int i = 0; i < filter_substeps; ++i)
      {
        filteredJointCommands->iterate();
      }

      qd_des_deg = computeDesiredJointSpeeds(filteredJointCommands.get(), q_curr_deg);

      for (size_t i = 0; i < DOF_ARM; ++i)
      {
        q_curr_deg[i] += qd_des_deg[i] * dt;

        // For testing purposes, we mimic the discontinuities at the 0 / 360 degrees boundaries
        if (q_curr_deg[i] > 360.0)
        {
          q_curr_deg[i] -= 360.0;
        }
        else if (q_curr_deg[i] < 0.0)
        {
          q_curr_deg[i] += 360.0;
        }

        const double delta = signed_diff_deg(q_curr_deg[i], wrap360(q_cont_deg[i]));
        q_cont_deg[i] += delta;
        // if (i==DOF_ARM-1)
        // {
        //   RLOG(0, "[%zu] Cmd: %f   Raw: %f   Cont: %f   delta: %f", i,
        //        cmd.q_des[i], q_curr_deg[i], q_cont_deg[i], delta);
        // }
      }

      nlohmann::json fb;
      std::vector<double> q_cont_rad(DOF_ARM), qd_curr_rad(DOF_ARM), tau(DOF_ARM, 0.0);
      for (size_t i = 0; i < DOF_ARM; ++i)
      {
        q_cont_rad[i] = RCS_DEG2RAD(q_cont_deg[i]);
        qd_curr_rad[i] = RCS_DEG2RAD(qd_des_deg[i]);
      }

      fb["position"] = q_cont_rad;
      fb["velocity"] = qd_curr_rad;
      fb["torque"] = tau;                 // always zero in sim
      fb["imu_acceleration"] = { 0, 0, 9.81 };        // dummy gravity vector
      fb["gripper_position"] = cmd.gripper_pos;

      feedbackFcn(fb.dump());

      // 4. sleep to keep fixed rate
      timer.wait();

      if (timer.getWaitCycleCount() % 100 == 0)
      {
        RLOG_CPP(1, "Time: " << timer.getTickUs() / 1000);
      }

    }

    RLOG(0, "Quitting roboThreadFuncTest");
  }

  /*!
   * \brief  Main robot control thread, ROS 1 style.
   *
   * \param ip         IPv4 address of the Gen3 controller.
   * \param readOnly   If true, the loop only reads feedback (no commands sent).
   * \param feedback   Thread-safe publisher object for outgoing feedback.
   * \param run_flag   External flag; loop runs while (run_flag && internal flag).
   *
   * \throw std::runtime_error on any unrecoverable communication error.
   */
#if defined (AFFACTION_WITH_KINOVA_GEN3)
  void roboThreadFuncHighLevel(const std::string& ip,
                               bool readOnly,
                               std::function<void(const std::string&)> feedbackFcn,
                               const std::atomic_bool& run_flag)
  {
    RFATAL("Needs fixing");
    using steady_clock = std::chrono::steady_clock;

    // error callback
    auto error_cb = [](Kinova::Api::KError err)
    {
      std::cerr << "[Kortex RPC error] " << err.toString() << '\n';
    };

    try
    {
      // transport & session (TCP)
      RLOG_CPP(1, "Connecting to ip " << ip << " at TCP port " << TCP_PORT
               << " and UDP port " << UDP_PORT);
      Kinova::Api::TransportClientTcp tcp;
      if (!tcp.connect(ip, TCP_PORT))
      {
        throw std::runtime_error("TCP connection failed");
      }

      RLOG_CPP(1, "Creating TCP session");
      Kinova::Api::RouterClient router_tcp(&tcp, error_cb);
      Kinova::Api::SessionManager session(&router_tcp);

      auto session_info = Kinova::Api::Session::CreateSessionInfo();
      session_info.set_username("admin");
      session_info.set_password("admin");
      session.CreateSession(session_info);

      // auxiliary UDP router (feedback)
      Kinova::Api::TransportClientUdp udp;
      if (!udp.connect(ip, UDP_PORT))
      {
        throw std::runtime_error("UDP connection failed");
      }

      RLOG_CPP(1, "Creating UDP session");
      Kinova::Api::RouterClient router_udp(&udp, error_cb);
      Kinova::Api::SessionManager session_udp(&router_udp);
      session_udp.CreateSession(session_info);            // UDP session

      // clients
      RLOG_CPP(1, "Creating base-client and base-cyclic client");
      Kinova::Api::Base::BaseClient base(&router_tcp);
      Kinova::Api::BaseCyclic::BaseCyclicClient base_cyclic(&router_udp);

      // clear faults & switch mode
      if (base.GetArmState().active_state() == Kinova::Api::Common::ARMSTATE_IN_FAULT)
      {
        RLOG_CPP(1, "Clearing faults");
        base.ClearFaults();
      }
      else
      {
        RLOG_CPP(1, "No faults to be cleared");
      }

      Kinova::Api::Base::ServoingModeInformation mode;
      mode.set_servoing_mode(
        Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      base.SetServoingMode(mode);

      // wait until the robot reports SERVOING_READY
      const auto deadline = steady_clock::now() + std::chrono::seconds{5};
      while (base.GetArmState().active_state() != Kinova::Api::Common::ARMSTATE_SERVOING_READY)
      {
        if (steady_clock::now() > deadline)
        {
          throw std::runtime_error("Timeout: servoing not ready");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{50});
      }

      // Read sensor data from robot
      Kinova::Api::BaseCyclic::Feedback base_feedback = base_cyclic.RefreshFeedback();

      // Initialize filter with current robot's state
      // Actuator Limit (magnitude) (from User-Guide-Gen3-R07.pdf pp.98)
      // large (joints 1 - 4) 79.64 deg/s (1.39 rad/s)
      // small (joints 5 - 7) 69.91 deg/s (1.22 rad/s)
      std::vector<double> maxVelInDeg = {75.0, 75.0, 75.0, 75.0, 60.0, 60.0, 60.0};
      const double tmc = 0.05;
      const double dt = 0.025;   // 40Hz
      std::vector<double> jointPositionsInDeg = getJointPositionsInDeg(base_feedback);
      std::unique_ptr<Rcs::RampFilterND> filteredJointCommands =
        std::make_unique<Rcs::RampFilterND>(tmc, 0.0, dt, DOF_ARM);
      filteredJointCommands->init(jointPositionsInDeg.data());
      for (size_t i = 0; i < jointPositionsInDeg.size(); ++i)
      {
        filteredJointCommands->setMaxVel(maxVelInDeg[i], i);
        this->incomingCommand.q_des[i] = jointPositionsInDeg[i];
        //RLOG(0, "jnt %zu: %.3f deg", i, jointPositionsInDeg[i]);
      }

      this->isInitialized = true;
      RLOG_CPP(1, "Finished initialization, entering control loop");



      size_t loopCount = 0;

      /* ---------- main loop ---------- */
      while (run_flag.load() && runLoop)
      {
        double t_cycle = getWallclockTime();

        // Get feedback and iterate command filters
        base_feedback = base_cyclic.RefreshFeedback();   // UDP 10001

        RoboCommand cmd = getCommand();

        if (cmd.received_q_des)
        {
          filteredJointCommands->setTarget(cmd.q_des.data());
        }

        filteredJointCommands->iterate();
        jointPositionsInDeg = getJointPositionsInDeg(base_feedback);
        std::vector<double> qd_des = computeDesiredJointSpeeds(filteredJointCommands.get(),
                                                               jointPositionsInDeg);

        if (!readOnly)
        {
          // Set desired joint velocities (deg/s)
          Kinova::Api::Base::JointSpeeds js_cmd;
          for (size_t i = 0; i < DOF_ARM; ++i)
          {
            auto* s = js_cmd.add_joint_speeds();
            s->set_joint_identifier(static_cast<std::uint32_t>(i));
            s->set_duration(0);     // valid until next cmd

            if (i==6)
            {
              s->set_value((float) qd_des[i]);
              RLOG(0, "qd: %f", qd_des[i]);
            }
            else
            {
              s->set_value(0.0);
            }
          }
          base.SendJointSpeedsCommand(js_cmd);    // TCP 10000
        }

        // Trigger feedback message
        feedbackFcn(feedback2JsonString(base_feedback, 0, nullptr, std::vector<double>()));

        // FPS
        t_cycle = getWallclockTime() - t_cycle;

        if (t_cycle > 1.1*dt)
        {
          RLOG_CPP(1, "Overflow: t_cycle = " << t_cycle);
        }

        if (loopCount%25==0)
        {
          RLOG_CPP(1, "Control-loop FPS: " << 1.0/t_cycle);
        }

        loopCount++;
      }

      // Shutdown
      Kinova::Api::Base::JointSpeeds stop_cmd;
      for (std::size_t i = 0; i < DOF_ARM; ++i)
      {
        auto* s = stop_cmd.add_joint_speeds();
        s->set_joint_identifier(static_cast<std::uint32_t>(i));
        s->set_value(0.0);
      }
      base.SendJointSpeedsCommand(stop_cmd);

      session.CloseSession();
      tcp.disconnect();
      udp.disconnect();
      RLOG_CPP(1, "Robot thread exited cleanly.");
    }
    catch (const Kinova::Api::KDetailedException& ex)
    {
      RLOG_CPP(0, "[KDetailedException] " << ex.what());
      throw;
    }
    catch (const std::exception& ex)
    {
      RLOG_CPP(0, "[roboThreadFuncROS1] " << ex.what());
      throw;
    }
  }

  void roboThreadFuncLowLevel(const std::string& ip,
                              bool readOnly,
                              std::function<void(const std::string&)> feedbackFcn,
                              const std::atomic_bool& run_flag,
                              std::vector<double> q_default_deg)
  {
    auto error_callback = [](k_api::KError err)
    {
      cout << "_________ callback error _________" << err.toString();
    };

    RLOG_CPP(1, "Creating TCP client");
    k_api::TransportClientTcp tcp;
    k_api::RouterClient router(&tcp, error_callback);

    if (!tcp.connect(ip, TCP_PORT))
    {
      throw std::runtime_error("TCP connect failed");
    }

    RLOG_CPP(1, "Creating UDP client");
    k_api::TransportClientUdp transport_real_time;
    k_api::RouterClient router_real_time(&transport_real_time, error_callback);

    transport_real_time.connect(ip, UDP_PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    RLOG_CPP(1, "Creating session");
    k_api::SessionManager session_manager(&router);
    session_manager.CreateSession(create_session_info);
    k_api::SessionManager session_manager_real_time(&router_real_time);
    session_manager_real_time.CreateSession(create_session_info);

    // Create services
    RLOG_CPP(1, "Creating base-client and base-cyclic client");
    k_api::Base::BaseClient base(&router);
    k_api::BaseCyclic::BaseCyclicClient base_cyclic(&router_real_time);
    k_api::ActuatorConfig::ActuatorConfigClient actuator_config(&router);

    // Read sensor data from robot
    k_api::BaseCyclic::Feedback base_feedback = base_cyclic.RefreshFeedback();

    // Get actuator count
    unsigned int actuator_count = base.GetActuatorCount().count();
    RCHECK(actuator_count==DOF_ARM);

    // Initialize actuator commands to current position.
    size_t nActuators = 0;
    k_api::BaseCyclic::Command base_command;
    for (auto actuator : base_feedback.actuators())
    {
      k_api::BaseCyclic::ActuatorCommand* actuator_command;
      actuator_command = base_command.mutable_actuators()->Add();
      actuator_command->set_position(actuator.position());
      actuator_command->set_velocity(0.0);
      actuator_command->set_torque_joint(0.0);
      actuator_command->set_command_id(0);
      nActuators++;
    }
    RCHECK(nActuators==DOF_ARM);

    // Initialize filter with current robot's state
    // Actuator Limit (magnitude) (from User-Guide-Gen3-R07.pdf pp.98)
    // large (joints 1 - 4) 79.64 deg/s (1.39 rad/s)
    // small (joints 5 - 7) 69.91 deg/s (1.22 rad/s)
    std::vector<double> maxVelInDeg = {75.0, 75.0, 75.0, 75.0, 60.0, 60.0, 60.0};
    const double tmc = 0.1;   // keep >= 10 times dt
    const double dt = 0.01;
    std::vector<double> q_curr_deg = getJointPositionsInDeg(base_feedback);

    if (q_default_deg.empty())
    {
      q_default_deg = q_curr_deg;
    }
    RCHECK(q_default_deg.size()==DOF_ARM);

    // Initialize continuous angles close to default pose
    std::vector<double> q_cont_deg = closest_to_default(q_curr_deg, q_default_deg);


    // Initialize filters with robot's continuous state
    std::unique_ptr<Rcs::RampFilterND> filteredJointCommands =
      std::make_unique<Rcs::RampFilterND>(tmc, 0.0, dt, DOF_ARM);
    filteredJointCommands->init(q_cont_deg.data());
    for (size_t i = 0; i < q_cont_deg.size(); ++i)
    {
      filteredJointCommands->setMaxVel(maxVelInDeg[i], i);
      this->incomingCommand.q_des[i] = q_cont_deg[i];
      if (q_cont_deg[i] != q_curr_deg[i])
      {
        RLOG(0, "continuous angles adjusted at index %zu: curr: %f   cont: %f   default: %f",
             i, q_curr_deg[i], q_cont_deg[i], q_default_deg[i]);
      }
    }

    // Clear faults
    if (base.GetArmState().active_state() == Kinova::Api::Common::ARMSTATE_IN_FAULT)
    {
      RLOG_CPP(1, "Clearing faults");
      base.ClearFaults();
    }
    else
    {
      RLOG_CPP(1, "No faults to be cleared");
    }

    // Set the base in low-level servoing mode
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base.SetServoingMode(servoing_mode);


    // Get the gripper's actual position from the base feedback
    // For gripper position, 0 is fully opened and 100 is fully closed.
    // For gripper speed, 0 is fully stopped and 100 is opening/closing (depending on
    // position used) at maximum speed.
    // Force parameter is used as a force limit to apply when closing or opening
    // the gripper. If this force limit is exceeded the gripper motion will stop.
    // 0 is the lowest force limit and 100 the maximum.
    k_api::GripperCyclic::MotorCommand* gripper_motor_command;
    {
      float gripper_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();

      // Initialize interconnect command to current gripper position.
      base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
      gripper_motor_command = base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
      gripper_motor_command->set_position(gripper_position);
      gripper_motor_command->set_velocity(0.0);
      gripper_motor_command->set_force(100.0);
    }



    // Initialize commands with current state
    base_feedback = base_cyclic.Refresh(base_command);

    if (!readOnly)
    {
      // Set actuators in velocity mode now that the command is equal to measure
      auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
      control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);
      for (unsigned int i = 0; i < actuator_count; i++)
      {
        actuator_config.SetControlMode(control_mode_message, i+1);
      }
    }

    // Real-time loop
    this->isInitialized = true;

    RLOG(1, "Entering low-level loop");
    PollingTimer timer(dt, PollingTimer::TimerMode::SleepAndPoll);



    while (run_flag.load() && runLoop)
    {

      if (readOnly)
      {
        base_feedback = base_cyclic.RefreshFeedback();

        // Trigger feedback message with 50Hz
        if (timer.getWaitCycleCount()%2==0)
        {
          int64_t time_usec = timer.getTickUs();
          feedbackFcn(feedback2JsonString(base_feedback, time_usec, nullptr, std::vector<double>()));
          RLOG_CPP(1, "Time: " << timer.getTickUs()/1000);
        }
      }
      else
      {
        RoboCommand cmd = getCommand();

        if (cmd.received_q_des)
        {
          filteredJointCommands->setTarget(cmd.q_des.data());
        }

        filteredJointCommands->iterate();
        q_curr_deg = getJointPositionsInDeg(base_feedback);

        for (std::size_t i = 0; i < q_curr_deg.size(); ++i)
        {
          const double delta = signed_diff_deg(q_curr_deg[i], wrap360(q_cont_deg[i]));
          q_cont_deg[i] += delta;
          RLOG(1, "Raw: %f   Cont: %f   delta: %f", q_curr_deg[i], q_cont_deg[i], delta);
        }

        std::vector<double> qd_des = computeDesiredJointSpeeds(filteredJointCommands.get(), q_curr_deg);

        // Incrementing identifier ensures actuators can reject out of time frames
        base_command.set_frame_id((base_command.frame_id()+1) % 65536);

        // Velocity command. We set the position to the current position to
        // avoid following error to trigger.
        // Bonus: When doing this instead of disabling the following error, if
        // communication is lost and first actuator continues to move under
        // torque command, resulting position error with command will
        // trigger a following error and switch back the actuator in position
        // command to hold its position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
          base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
          base_command.mutable_actuators(i)->set_velocity((float) qd_des[i]);
          base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
        }

        // Gripper command
        const double gripper_curr = base_feedback.interconnect().gripper_feedback().motor()[0].position();
        const double gripper_error = cmd.gripper_pos - gripper_curr;
        const double gripper_p_gain = 2.5;
        double gripper_velocity = Math_clip(gripper_p_gain*dt*fabs(gripper_error), MINIMAL_GRIPPER_VELOCITY, 100.0);

        if (fabs(gripper_error) < MINIMAL_GRIPPER_POSITION_ERROR)
        {
          gripper_velocity = 0.0;
        }

        gripper_motor_command->set_position((float) cmd.gripper_pos);
        gripper_motor_command->set_velocity((float) gripper_velocity);
        gripper_motor_command->set_force((float) cmd.gripper_force);

        // Send command and update feedback from robot
        base_feedback = base_cyclic.Refresh(base_command, 0);

        // Trigger feedback message with 50Hz
        if (timer.getWaitCycleCount()%2==0)
        {
          int64_t time_usec = timer.getTickUs();
          feedbackFcn(feedback2JsonString(base_feedback, time_usec, &cmd, q_cont_deg));
        }
      }

      timer.wait();
    }

    // Set actuators back in position
    if (!readOnly)
    {
      auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
      control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
      for (unsigned int i = 0; i < actuator_count; i++)
      {
        actuator_config.SetControlMode(control_mode_message, i+1);
      }
    }
    RLOG_CPP(0, "Robo thread clean exit");

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base.SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Close API session
    session_manager.CloseSession();
    session_manager_real_time.CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router.SetActivationStatus(false);
    tcp.disconnect();
    router_real_time.SetActivationStatus(false);
    transport_real_time.disconnect();

    RLOG(0, "Quitting robot driver thread");
  }

  std::string feedback2JsonString(const k_api::BaseCyclic::Feedback& feedback,
                                  int64_t time_usec, const RoboCommand* cmd,
                                  std::vector<double> q_cont_deg)
  {
    std::vector<double> jointVel(feedback.actuators_size());
    std::vector<double> jointTorque(feedback.actuators_size());
    std::vector<double> jointPos;

    if (q_cont_deg.empty())
    {
      jointPos = getJointPositionsInDeg(feedback);
    }
    else
    {
      jointPos = q_cont_deg;
    }

    for (int j = 0; j < feedback.actuators_size(); ++j)
    {
      jointPos[j] = RCS_DEG2RAD(jointPos[j]);
      jointVel[j] =  RCS_DEG2RAD(feedback.actuators(j).velocity());
      jointTorque[j] = feedback.actuators(j).torque();
    }

    std::vector<double> toolWrench;
    toolWrench.push_back(feedback.base().tool_external_wrench_force_x());
    toolWrench.push_back(feedback.base().tool_external_wrench_force_y());
    toolWrench.push_back(feedback.base().tool_external_wrench_force_z());
    toolWrench.push_back(feedback.base().tool_external_wrench_torque_x());
    toolWrench.push_back(feedback.base().tool_external_wrench_torque_y());
    toolWrench.push_back(feedback.base().tool_external_wrench_torque_z());

    std::vector<double> imu_accel;
    imu_accel.push_back(feedback.base().imu_acceleration_x());
    imu_accel.push_back(feedback.base().imu_acceleration_y());
    imu_accel.push_back(feedback.base().imu_acceleration_z());

    nlohmann::json fbJson;
    fbJson["time"] = getWallclockTime();
    fbJson["cycle_time_usec"] = time_usec;
    fbJson["position"] = jointPos;
    fbJson["velocity"] = jointVel;
    fbJson["torque"] = jointTorque;
    fbJson["imu_acceleration"] = imu_accel;
    fbJson["gripper_position"] = feedback.interconnect().gripper_feedback().motor()[0].position();

    if (cmd)
    {
      std::vector<double> joint_err(DOF_ARM, 0.0);
      std::vector<double> joint_cmd(DOF_ARM, 0.0);

      for (size_t i=0; i<DOF_ARM; ++i)
      {
        joint_err[i] = RCS_RAD2DEG(RCS_DEG2RAD(cmd->q_des[i]) - jointPos[i]);
        joint_cmd[i] = RCS_DEG2RAD(cmd->q_des[i]);
      }
      fbJson["position_error"] = joint_err;
      fbJson["position_command"] = joint_cmd;
    }

    return fbJson.dump();
  }

  std::vector<double> getJointPositionsInDeg(const k_api::BaseCyclic::Feedback& feedback) const
  {
    std::vector<double> jointPos(feedback.actuators_size());

    for (size_t i=0; i<jointPos.size(); ++i)
    {
      jointPos[i] = feedback.actuators(i).position();
    }

    return jointPos;
  }

#endif

  std::vector<double> computeDesiredJointSpeeds(const Rcs::RampFilterND* filteredCommands,
                                                const std::vector<double>& x_curr_in_deg) const
  {
    const double ffwGain = 0.9;// ffwGain is Kinova's velocity overshoot
    const double tau = 0.2;    // seconds to reach 63% of the target error (smaller is more stiff)
    const double fbGain = 1.0 / tau;
    std::vector<double> qd_des(filteredCommands->getDim(), 0.0);

    for (unsigned int i = 0; i < qd_des.size(); i++)
    {
      // const double x_des = filteredCommands->getPosition(i);
      // double fberr = Math_fmodAngle(RCS_DEG2RAD(x_des)) - Math_fmodAngle(RCS_DEG2RAD(x_curr_in_deg[i]));
      // fberr = RCS_RAD2DEG(Math_fmodAngle(fberr));

      // const double maxVel = filteredCommands->getMaxVel(i);
      // const double xd_des = ffwGain*filteredCommands->getVelocity(i) + fbGain*fberr;


      // feedback error in degrees, shortest path
      const double err_deg = signed_diff_deg(filteredCommands->getPosition(i), x_curr_in_deg[i]);

      // feed-forward + proportional feedback
      const double xd_des = ffwGain * filteredCommands->getVelocity(i) + fbGain * err_deg;

      // saturate
      qd_des[i] = Math_clip(xd_des, -filteredCommands->getMaxVel(i), filteredCommands->getMaxVel(i));
    }

    return qd_des;
  }


  mutable std::mutex cmdMtx;
  std::atomic<bool> isInitialized{false};
  RoboCommand incomingCommand;
  std::atomic<bool> runLoop{false};
  std::thread kortexThread;
};
