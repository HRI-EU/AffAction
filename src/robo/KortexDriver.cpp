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

#include "KortexDriver.hpp"
#include "RoboDriverNetworking.hpp"

#include <Rcs_cmdLine.h>

#include <zmq.hpp>
#include <csignal>


static std::atomic<bool> runLoop(true);


/*******************************************************************************
 * Convenience class for maintaining networking information for the
 * Jaco Gen3 robots
 ******************************************************************************/
class JacoNetworkInfo
{
public:
  std::string robo_ip;
  int roboToRemotePort;
  int remoteToRoboPort;
  std::string robo_computer_ip; // Default: "localhost"
  std::string roboSender;       // Default: "tcp://*:5555"
  std::string roboReceiver;     // Default: "tcp://*:5556";
  std::string remoteReceiver;   // Default: "tcp://localhost:5555"
  std::string remoteSender;     // Default: "tcp://localhost:5556"
  std::string roboMode;
  std::vector<double> q_default_deg;

  static JacoNetworkInfo getNetworkInfo(const std::string roboName)
  {
    // 7-dof Jaco Gen3 (right mounted)
    JacoNetworkInfo rummenigge;
    rummenigge.robo_ip = "10.107.149.2";
    rummenigge.roboToRemotePort = 5555;
    rummenigge.remoteToRoboPort = 5556;
    rummenigge.robo_computer_ip = "localhost";
    rummenigge.roboSender = "tcp://*:" + std::to_string(rummenigge.roboToRemotePort);
    rummenigge.roboReceiver = "tcp://*:" + std::to_string(rummenigge.remoteToRoboPort);
    rummenigge.remoteReceiver = "tcp://" + rummenigge.robo_computer_ip + ":" + std::to_string(rummenigge.roboToRemotePort);
    rummenigge.remoteSender = "tcp://" + rummenigge.robo_computer_ip + ":" + std::to_string(rummenigge.remoteToRoboPort);
    rummenigge.roboMode = "LowLevel";
    rummenigge.q_default_deg = { 220.0, 80.0, 80.0, -100.0, 0.0, -40.0, -110.0 };

    // 7-dof Jaco Gen3 (left mounted)
    JacoNetworkInfo littbarski;
    littbarski.robo_ip = "10.107.149.3";
    littbarski.roboToRemotePort = 5557;
    littbarski.remoteToRoboPort = 5558;
    littbarski.robo_computer_ip = "localhost";
    littbarski.roboSender = "tcp://*:" + std::to_string(littbarski.roboToRemotePort);
    littbarski.roboReceiver = "tcp://*:" + std::to_string(littbarski.remoteToRoboPort);
    littbarski.remoteReceiver = "tcp://" + littbarski.robo_computer_ip + ":" + std::to_string(littbarski.roboToRemotePort);
    littbarski.remoteSender = "tcp://" + littbarski.robo_computer_ip + ":" + std::to_string(littbarski.remoteToRoboPort);
    littbarski.roboMode = "LowLevel";
    littbarski.q_default_deg = { -40.0, -80.0, -80.0, 100.0, 0.0, 40.0, 110.0 };

    // Just simulation
    JacoNetworkInfo test_right = rummenigge;
    test_right.roboMode = "TestWithoutRobot";

    // Just simulation
    JacoNetworkInfo test_left = littbarski;
    test_left.roboMode = "TestWithoutRobot";

    static std::map<std::string, JacoNetworkInfo> nwInfo =
    {
      { "rummenigge", rummenigge },
      { "littbarski", littbarski },
      { "test_right", test_right },
      { "test_left",  test_left  }
    };

    auto it = nwInfo.find(roboName);

    if (it == nwInfo.end())
    {
      RFATAL("roboName '%s' not known", roboName.c_str());
    }

    return it->second;
  }

};

/*******************************************************************************
 *
 *******************************************************************************/
void quit(int /*sig*/)
{
  static int kHit = 0;
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit+1);
  kHit++;
  runLoop = false;

  if (kHit > 1)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}

/******************************************************************************
 * Simple test helpers
 *
 *  - commandPublisher:  binds a PUB socket on the command port
 *                       and injects dummy joint commands
 ******************************************************************************/
static void sinusoidalCommandPublisher(const std::string& endpoint,
                                       const std::atomic_bool& run_flag,
                                       const std::vector<double>& q_init)
{
  if (q_init.size() != DOF_ARM)
  {
    RLOG(0, "Wrong size in q_init: %zu", q_init.size());
    //return;
  }

  zmq::context_t ctx(1);
  zmq::socket_t  pub(ctx, zmq::socket_type::pub);
  pub.connect(endpoint);

  size_t cnt = 0;
  std::vector<double> q(q_init.size(), 0.0);
  while (run_flag)
  {
    // trivial sinusoid command in radians, but only to the distal 3 joints
    nlohmann::json j;
    for (size_t i = 0; i < q.size(); ++i)
    {
      q[i] = q_init[i];

      if (i==q.size()-1 || i==q.size()-2 || i==q.size()-3)
      {
        q[i] += RCS_DEG2RAD(0.1*190.0) * std::sin(0.1 * cnt);
      }

    }

    j["q_des"]           = q;
    j["gripper_command"] = 0.0;
    j["gripper_force"]   = 0.0;

    if (cnt>150)
    {
      j["quit"] = true;
    }

    const std::string payload = j.dump(4);
    RLOG_CPP(0, "[sinusoidalCommandPublisher]: " << cnt);
    RLOG_CPP(1, payload);
    pub.send(zmq::buffer(payload), zmq::send_flags::none);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ++cnt;
  }
  RLOG(0, "sinusoidalCommandPublisher says good bye");
}


/******************************************************************************
 * Simple test helpers
 *
 *  - feedbackSubscriber: connects a SUB socket to the feedback port
 *                        and prints every JSON packet it receives
 ******************************************************************************/
static std::vector<double> feedbackSubscriber(const std::string& endpoint,
                                              const std::atomic_bool& run_flag,
                                              bool once=false)
{
  std::vector<double> position;
  zmq::context_t ctx(1);
  zmq::socket_t sub(ctx, zmq::socket_type::sub);
  sub.connect(endpoint);
  sub.set(zmq::sockopt::subscribe, "");


  while (run_flag)
  {
    zmq::message_t msg;
    if (sub.recv(msg, zmq::recv_flags::dontwait))
    {
      try
      {
        auto* begin = static_cast<const char*>(msg.data());
        auto* end   = begin + msg.size();
        auto j = nlohmann::json::parse(begin, end);

        RLOG_CPP(0, "[feedbackSubscriber]");
        RLOG_CPP(1, j.dump(4));

        position = j.at("position").get<std::vector<double>>();

        if (once)
        {
          break;
        }
      }
      catch (const nlohmann::json::parse_error& e)
      {
        RLOG_CPP(0, "JSON parse error: " << e.what());
      }
      catch (const nlohmann::json::out_of_range& e)
      {
        RLOG_CPP(0, "Missing key 'position': " << e.what());
      }
      catch (const nlohmann::json::type_error& e)
      {
        RLOG_CPP(0, "Invalid type for 'position': " << e.what());
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }


  RLOG(0, "[once=%s] feedbackSubscriber says good bye", once ? "true" : "false");

  return position;
}


/*******************************************************************************
 *
 *******************************************************************************/
static void runRobo(int argc, char** argv)
{
  std::string robo_name = "test_right";
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-robo_name", &robo_name, "Name of Jaco Gen3 (default is '%s')", robo_name.c_str());
  bool readOnly = argP.hasArgument("-ro", "Read-only, no motor commands");
  bool testMe = argP.hasArgument("-testMe", "Start test network endpoints");

  if (argP.hasArgument("-h"))
  {
    argP.print();
    return;
  }

  JacoNetworkInfo nwInfo = JacoNetworkInfo::getNetworkInfo(robo_name);

  if (testMe && nwInfo.roboMode!="TestWithoutRobot")
  {
    RMSG("Test function should not be used with real robot (only TestWithoutRobot mode)");
    return;
  }

  FeedbackThread feedback;
  feedback.start(nwInfo.roboSender, runLoop);

  KortexDriver robo;

  // auto fbFcn = std::bind(&FeedbackThread::updateMessage, &feedback, std::placeholders::_1);
  auto fbFcn = [&feedback](const std::string& message)
  {
    feedback.updateMessage(message);
  };

  // auto cmdFcn = std::bind(&KortexDriver::setCommand, &robo, std::placeholders::_1);
  auto cmdFcn = [&robo](const std::string& message) -> bool
  {
    RLOG_CPP(1, "Received: " << message);
    return robo.setCommand(message);
  };

  robo.start(nwInfo.robo_ip, fbFcn, runLoop, readOnly, nwInfo.roboMode, nwInfo.q_default_deg);

  if (testMe)
  {

    std::thread([&nwInfo]()
    {
      std::vector<double> q = feedbackSubscriber(nwInfo.remoteReceiver, runLoop, true);
      sinusoidalCommandPublisher(nwInfo.remoteSender, runLoop, q);

    }).detach();

    std::thread([&nwInfo]()
    {
      feedbackSubscriber(nwInfo.remoteReceiver, runLoop);
    }).detach();
  }

  // Start non-threaded
  bool blocking = true;
  CommandThread commands;
  commands.start(nwInfo.roboReceiver, cmdFcn, runLoop, blocking);

  commands.stop();
  robo.stop();
  feedback.stop();
}


/*******************************************************************************
 * Ctrl-C stops threads
 *******************************************************************************/
int main(int argc, char** argv)
{
  signal(SIGINT, quit);   // Ctrl-C stops threads

  int mode = 0;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Mode (default is %d)", mode);

  switch (mode)
  {
    case 0:
      printf("\nHere's what you can do:\n\n");
      printf("\t-m 0   Prints this message (default)\n");
      printf("\t-m 1   Run Jaco arm\n");
      printf("\t-m 2   Run sender\n");
      printf("\t-m 3   Run receiver\n");
      printf("\n");
      argP.print();
      break;

    case 1:
      runRobo(argc, argv);
      break;

    case 2:   // Command publisher, initialized with q from robot
    {
      // PTUDriver: bin/KortexDriver -m 2 -dl 1 -subscriber_port 5559 -publisher_port 5560
      int subscriber_port = 5555;
      int publisher_port = 5556;
      std::string ip = "tcp://localhost";
      argP.getArgument("-ip", &ip, "Network connection, default: %s", ip.c_str());
      argP.getArgument("-subscriber_port", &subscriber_port, "Network subscriber port, default: %d", subscriber_port);
      argP.getArgument("-publisher_port", &publisher_port, "Network publisher port, default: %d", publisher_port);


      std::string subscriber_connection = ip + ":" + std::to_string(subscriber_port);
      RLOG_CPP(0, "Waiting for initial joint values on " << subscriber_connection);
      std::vector<double> q = feedbackSubscriber(subscriber_connection, runLoop, true);

      RLOG(0, "Initializing command publisher with:");
      for (size_t i=0; i<q.size(); ++i)
      {
        RLOG(0, "q[%zu] = %f", i, q[i]);
      }

      std::string publisher_connection = ip + ":" + std::to_string(publisher_port);
      RLOG_CPP(0, "Starting command publisher on " << publisher_connection);
      sinusoidalCommandPublisher(publisher_connection, runLoop, q);
    }
    break;

    case 3:   // Feedback subscriber
    {
      bool once = false;
      int port = 5555;
      std::string ip = "tcp://localhost";
      argP.getArgument("-once", &once, "Just one read, then stop");
      argP.getArgument("-port", &port, "Network port, default: %d", port);
      argP.getArgument("-ip", &ip, "Network connection, default: %s", ip.c_str());

      if (argP.hasArgument("-h"))
      {
        argP.print();
        break;
      }

      std::string connection = ip + ":" + std::to_string(port);
      std::vector<double> q = feedbackSubscriber(connection, runLoop, once);
      for (size_t i=0; i<q.size(); ++i)
      {
        RLOG(0, "q[%zu] = %f", i, q[i]);
      }
    }
    break;

    default:
      RLOG_CPP(0, "No mode " << mode);
  };

  RLOG_CPP(0, "Thanks, that's it for mode " << mode);

  return 0;
}
