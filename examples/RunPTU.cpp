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

#include <EntityBase.h>
#include <PW70Component.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>

#include "json.hpp"
#include <zmq.hpp>

#include <csignal>
#include <thread>
#include <string>



static bool runLoop = true;

static void quit(int)
{
  runLoop = false;
}

static void networkRecvCommandsThreadFunc(zmq::context_t& context, int recvPort)
{
  // Set up the socket for receiving motor commands
  zmq::socket_t recv_socket(context, zmq::socket_type::sub);

  try
  {
    // Set up the socket for sending joint angles
    std::string recvPortStr = "tcp://*:" + std::to_string(recvPort);
    RLOG_CPP(0, "Binding recv_socket to " << recvPortStr);
    recv_socket.bind(recvPortStr);

    if (!recv_socket)
    {
      throw std::runtime_error("Failed to bind recv_socket to " + recvPortStr);
    }

    // Set ZMQ_SUBSCRIBE and ZMQ_RCVTIMEO option to receive all messages
    RLOG_CPP(0, "Setting ZMQ_SUBSCRIBE and ZMQ_RCVTIMEO options on recv_socket");
#if ZMQ_VERSION < ZMQ_MAKE_VERSION(4, 3, 2)
    recv_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    recv_socket.setsockopt(ZMQ_RCVTIMEO, 1000); // Timeout in milliseconds
#else
    recv_socket.set(zmq::sockopt::subscribe, "");
    recv_socket.set(zmq::sockopt::rcvtimeo, 1000);
#endif
    RLOG_CPP(0, "Sockets successfully initialized");
  }
  catch (const zmq::error_t& e)
  {
    RLOG_CPP(0, "ZeroMQ error during initialization: " << e.what());
    throw;  // Rethrow after logging the error
  }
  catch (const std::exception& e)
  {
    RLOG_CPP(0, "Error during initialization: " << e.what());
    throw;  // Rethrow after logging the error
  }

  size_t loopCount = 0;

  while (runLoop)
  {
    try
    {
      // Receive the message with a non-blocking flag
      zmq::message_t message;
      zmq::recv_result_t result = recv_socket.recv(message);

      // Message received
      if (result.has_value())
      {
        // Parse the received JSON command
        std::string command_str(static_cast<char*>(message.data()), message.size());
        nlohmann::json motor_commands = nlohmann::json::parse(command_str);
        RLOG_CPP(3, "Received motor commands: " << motor_commands.dump());
        // if (motor_commands.contains("q_des"))
        // {
        //   std::vector<double> jointCommand = motor_commands["q_des"].get<std::vector<double>>();
        //   if (jointCommand.size()==filteredJointCommands->getDim())
        //   {
        //     if (!readOnly)
        //     {
        //       VecNd_constMulSelf(jointCommand.data(), 180.0 / M_PI, jointCommand.size());
        //       std::lock_guard<std::mutex> lock(cmdMtx);
        //       filteredJointCommands->setTarget(jointCommand.data());
        //     }
        //   }
        //   else
        //   {
        //     RLOG_CPP(0, "Mismatch in received joint command dimension: received "
        //              << jointCommand.size() << " but expected "
        //              << filteredJointCommands->getDim() << " values");
        //   }
        // }
        // if (motor_commands.contains("gripper_command"))
        // {
        //   std::lock_guard<std::mutex> lock(cmdMtx);
        //   gripper_target_position = motor_commands["gripper_command"].get<double>();
        // }
        // if (motor_commands.contains("gripper_force"))
        // {

        //   std::lock_guard<std::mutex> lock(cmdMtx);
        //   gripper_target_force = motor_commands["gripper_force"].get<double>();
        // }
      }   // if (result.has_value())
    }
    catch (const zmq::error_t& e)
    {
      if (e.num() == EAGAIN)
      {
        // This means no message was received (e.g., in non-blocking mode)
        // You can log or ignore this depending on your needs
        RLOG_CPP(0, "No message received, will retry.");
      }
      else
      {
        // Handle other ZMQ exceptions
        RLOG_CPP(0, "ZMQ error occurred: " << e.what());
      }
    }

    Timer_waitDT(0.005);
    loopCount++;
  }

  RLOG(0, "Quitting network thread");
}

static void networkSendSensorsThreadFunc(const aff::PW70VelocityComponent& pw70,
                                         zmq::context_t& context, int sendPort)
{

  // Set up the socket for sending joint angles
  zmq::socket_t send_socket(context, zmq::socket_type::pub);

  try
  {
    // Set up the socket for sending joint angles
    std::string sendPortStr = "tcp://*:" + std::to_string(sendPort);
    RLOG_CPP(0, "Binding send_socket to " << sendPortStr);
    send_socket.bind(sendPortStr);

    if (!send_socket)
    {
      throw std::runtime_error("Failed to bind send_socket to " + sendPortStr);
    }

    RLOG_CPP(0, "Sockets successfully initialized");
  }
  catch (const zmq::error_t& e)
  {
    RLOG_CPP(0, "ZeroMQ error during initialization: " << e.what());
    throw;  // Rethrow after logging the error
  }
  catch (const std::exception& e)
  {
    RLOG_CPP(0, "Error during initialization: " << e.what());
    throw;  // Rethrow after logging the error
  }

  double time_stamp_prev = 0.0;

  while (runLoop)
  {
    // We make this a 100Hz poll so that we don't interfere with the 50Hz CAN communication.
    Timer_waitDT(0.01);

    double pan_position, tilt_position, pan_velocity, tilt_velocity, time_stamp;
    pw70.getSensorData(pan_position, tilt_position, pan_velocity, tilt_velocity, time_stamp);

    if (time_stamp == time_stamp_prev)
    {
      continue;
    }

    time_stamp_prev = time_stamp;

    nlohmann::json sensorJson;
    sensorJson["pan_position"] = pan_position;
    sensorJson["tilt_position"] = tilt_position;
    sensorJson["pan_velocity"] = pan_velocity;
    sensorJson["tilt_velocity"] = tilt_velocity;
    sensorJson["time_stamp"] = time_stamp;

    std::string serialized_data = sensorJson.dump();
    zmq::message_t message(serialized_data.size());
    memcpy(message.data(), serialized_data.c_str(), serialized_data.size());
    auto result = send_socket.send(message, zmq::send_flags::none);
    if (!result)
    {
      RLOG_CPP(0, "Error: Failed to send message.");
    }
    else
    {
      RLOG_CPP(2, "Message sent successfully: " << serialized_data);
    }

  }

  RLOG(0, "Quitting network thread");
}


int main(int argc, char** argv)
{
  Timer_setZero();

  int mode = 0;
  int sensorSenderPort = 5555;
  int commandReceiverPort = 5556;
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  argP.getArgument("-m", &mode, "Test mode (default is %d)", mode);

  aff::EntityBase entity;

  // Start the driver before sending out the first sensor data
  aff::PW70VelocityComponent pw70(&entity, 0, 1);
  pw70.onStart();   // Returns only after initialization is finished

  // Configure zmq networking
  RLOG_CPP(0, "ZMQ_VERSION: " << ZMQ_VERSION << " ZMQ_MAKE_VERSION(4, 3, 2) " << ZMQ_MAKE_VERSION(4, 3, 2));
  RLOG_CPP(0, "ZMQ_VERSION 4.7.0: " << ZMQ_MAKE_VERSION(4, 7, 0));
  zmq::context_t context(1);

  auto sendThread = std::thread(networkSendSensorsThreadFunc, std::cref(pw70), std::ref(context), sensorSenderPort);
  //auto recvThread = std::thread(networkRecvCommandsThreadFunc, std::ref(context), commandReceiverPort);

  // Ctrl-C sets runLoop to false. We do this after initialization so that
  // we can use Ctrl-C during startup
  signal(SIGINT, quit);





  RPAUSE();


  pw70.onStop();


  runLoop = false;

  RLOG_CPP(0, "Joining network threads");

  //recvThread.join();
  //RLOG_CPP(0, "Receiver thread joined");

  sendThread.join();
  RLOG_CPP(0, "Sender thread joined");


  RLOG_CPP(0, "Thanks for running the PTU with this fine velocity controller");

  return 0;
}
