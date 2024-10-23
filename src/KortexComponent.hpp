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

#include "ComponentBase.h"
#include "json.hpp"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>

#include <zmq.hpp>

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>



namespace aff
{


class JointNameIndexPair
{
public:

  JointNameIndexPair() : jointId(-1)
  {
  }

  JointNameIndexPair(const std::string& name, int id=-1) : jointName(name), jointId(id)
  {
  }

  RcsJoint* getJoint(RcsGraph* graph)
  {
    RcsJoint* jnt = nullptr;

    if ((jointId==-1) || (!STREQ(graph->joints[jointId].name, jointName.c_str())))
    {
      jnt = RcsGraph_getJointByName(graph, jointName.c_str());
      if (!jnt)
      {
        return nullptr;
      }
      else
      {
        jointId = jnt->jointIndex;
      }

    }
    else
    {
      jnt = &graph->joints[jointId];
    }

    return jnt;
  }

  //private:
  std::string jointName;
  int jointId;
};

class KortexComponent : public ComponentBase
{
public:
  KortexComponent(EntityBase* parent, std::string suffix="_right")
    : ComponentBase(parent)
  {
    jntNameIdPairs.push_back(JointNameIndexPair("joint_1"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_2"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_3"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_4"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_5"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_6"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_7"+suffix));

    RLOG(0, "Subscribing to events");
    subscribe("Start", &KortexComponent::onStart);
    subscribe("Stop", &KortexComponent::onStop);
    subscribe("UpdateGraph", &KortexComponent::onUpdateGraph);
    subscribe("SetJointCommand", &KortexComponent::onSetJointPosition);
    subscribe("InitFromState", &KortexComponent::onInitFromState);
    subscribe("EmergencyStop", &KortexComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &KortexComponent::onEmergencyRecover);
    subscribe("EnableCommands", &KortexComponent::onEnableCommands);

    RLOG(0, "Done constructor");
  }

  ~KortexComponent()
  {
    onStop();
  }

  void onStart()
  {
    if (runLoop)
    {
      RLOG(0, "KortexComponent already started - doing nothing");
      return;
    }

    runLoop = true;
    recv_thread = std::thread(&KortexComponent::recvThreadFunc, this);
    recv_thread.detach();   // Needed, otherwise crashes

    while (!initialized())
    {
      fprintf(stderr, ".");
      fflush(stderr);
      Timer_waitDT(0.1);
    }

  }

  void onUpdateGraph(RcsGraph* graph)
  {
    std::vector<double> jntPosTmp, jntVelTmp;

    {
      std::lock_guard<std::mutex> lock(recvMtx);
      jntPosTmp = jointPosition;
      jntVelTmp = jointVelocity;
    }

    if (jntPosTmp.size()!=jntNameIdPairs.size())
    {
      RLOG(0, "No data yet received");
      return;
    }

    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      RcsJoint* jnt = jntNameIdPairs[i].getJoint(graph);
      RCHECK_MSG(jnt, "Joint '%s' not fund in graph", jntNameIdPairs[i].jointName.c_str());
      MatNd_set(graph->q, jnt->jointIndex, 0, jntPosTmp[i]);
      MatNd_set(graph->q_dot, jnt->jointIndex, 0, jntVelTmp[i]);
    }

  }

  void onStop()
  {
    runLoop = false;

    if (recv_thread.joinable())
    {
      recv_thread.join();
    }

  }

  bool initialized() const
  {
    return jointPosition.size()==jntNameIdPairs.size() ? true : false;
  }

private:

  void onSetJointPosition(const MatNd* q_des)
  {
    if ((enableCommands) && (!eStop))
    {
      std::vector<double> q7;
      for (size_t i=0; i<jntNameIdPairs.size(); ++i)
      {
        RCHECK_MSG(jntNameIdPairs[i].jointId!=-1, "Joint: '%s'",
                   jntNameIdPairs[i].jointName.c_str());
        double qi = MatNd_get(q_des, jntNameIdPairs[i].jointId, 0);
        q7.push_back(qi);
      }

      std::lock_guard<std::mutex> lock(cmdMtx);
      jointCommands = q7;
    }

  }

  void onInitFromState(const RcsGraph* target)
  {
    RLOG(0, "RoboJacoComponent::onInitFromState()");
    onSetJointPosition(target->q);
  }

  void onEmergencyStop()
  {
    if (this->eStop == false)
    {
      RLOG(0, "KortexComponent::EmergencyStop");
    }

    this->eStop = true;
    enableCommands = false;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "KortexComponent::EmergencyRecover");
    this->eStop = false;
    enableCommands = true;
  }

  void onEnableCommands()
  {
    enableCommands = true;
  }

  void recvThreadFunc()
  {
    Timer_waitDT(2.0);

    RLOG_CPP(0, "Creating zmq context");
    zmq::context_t context(1);

    RLOG_CPP(0, "Creating recv_socket");
    zmq::socket_t recv_socket(context, zmq::socket_type::sub);

    // Set up the socket for receiving joint angles
    try
    {
      // Attempt to connect the socket
      RLOG_CPP(0, "Connecting to tcp://localhost:5555");
      recv_socket.connect("tcp://localhost:5555");
      if (!recv_socket)
      {
        RLOG(0, "Failed to connect recv_socket to tcp://localhost:5555");
        throw std::runtime_error("Failed to connect recv_socket to tcp://localhost:5555");
      }
      else
      {
        RLOG(0, "Success to connect recv_socket to tcp://localhost:5555");
      }

      // Set socket option to subscribe to all messages (ZMQ_SUBSCRIBE with an empty filter)
      RLOG_CPP(0, "Setting ZMQ_SUBSCRIBE option");
      recv_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

      RLOG_CPP(0, "recv_socket successfully set up");
    }
    catch (const zmq::error_t& e)
    {
      RLOG_CPP(0, "ZeroMQ error: " << e.what());
      throw;
    }
    catch (const std::exception& e)
    {
      RLOG_CPP(0, "Error: " << e.what());
      throw;
    }



    // Set up the socket for sending motor commands
    RLOG_CPP(0, "Creating send_socket");
    zmq::socket_t send_socket(context, zmq::socket_type::pub);

    // Set up the socket for receiving joint angles
    try
    {
      // Attempt to connect the socket
      RLOG_CPP(0, "Connecting to tcp://localhost:5556");
      send_socket.connect("tcp://localhost:5556");
      if (!send_socket)
      {
        RLOG(0, "Failed to connect send_socket to tcp://localhost:5556");
        throw std::runtime_error("Failed to connect send_socket to tcp://localhost:5556");
      }
      else
      {
        RLOG(0, "Success to connect send_socket to tcp://localhost:5556");
      }

      RLOG_CPP(0, "send_socket successfully set up");
    }
    catch (const zmq::error_t& e)
    {
      RLOG_CPP(0, "ZeroMQ error: " << e.what());
      throw;
    }
    catch (const std::exception& e)
    {
      RLOG_CPP(0, "Error: " << e.what());
      throw;
    }


    RLOG_CPP(0, "Entering while loop");

    while (runLoop)
    {
      // Check for motor commands (non-blocking)
      zmq::pollitem_t items[] = {{recv_socket, 0, ZMQ_POLLIN, 0}};
      zmq::poll(items, 1, 0); // Poll with a 0 timeout (non-blocking)
      if (items[0].revents & ZMQ_POLLIN)
      {
        receive_joint_angles(recv_socket);
      }

      // Send motor commands
      if (enableCommands)
      {
        send_motor_commands(send_socket);
      }
    }

    RLOG(0, "Quitting run thread");
  }

  void receive_joint_angles(zmq::socket_t& recv_socket)
  {
    zmq::message_t message;
    recv_socket.recv(message, zmq::recv_flags::none);
    std::string recv_msg(static_cast<char*>(message.data()), message.size());
    nlohmann::json recv_json;

    try
    {
      // Try to parse the JSON string safely
      recv_json = nlohmann::json::parse(recv_msg);

      // If successful, process the parsed JSON data
      RLOG_CPP(1, "Parsed joint angles: " << recv_json.dump(4));

      std::lock_guard<std::mutex> lock(recvMtx);
      jointPosition = recv_json["position"].get<std::vector<double>>();
      jointVelocity = recv_json["velocity"].get<std::vector<double>>();
      jointTorque = recv_json["torque"].get<std::vector<double>>();

      // Further processing of joint angles here...

    }
    catch (const nlohmann::json::parse_error& e)
    {
      // Handle JSON parsing errors gracefully
      std::cerr << "JSON parsing error: " << e.what() << std::endl;
      std::cerr << "Invalid JSON string: " << recv_json << std::endl;

      // You can take additional actions here, such as:
      // - Logging the error to a file
      // - Attempting a retry
      // - Returning a default value
      // - Skipping the current message and continuing
    }



    //json joint_angles = json::parse(angles_str);
    //RLOG_CPP(0, "Received joint angles: " << joint_angles.dump());
  }

  void send_motor_commands(zmq::socket_t& send_socket)
  {

    if (jointCommands.empty() || (jointCommands==jointCommandsPrev))
    {
      return;
    }

    RLOG_CPP(0, "Sending motor commands");

    // Simulate motor commands (replace with actual control values)
    nlohmann::json cmdJson;

    {
      std::lock_guard<std::mutex> lock(cmdMtx);
      cmdJson["q_des"] = jointCommands;
    }

    std::string message_str = cmdJson.dump();
    zmq::message_t message(message_str.size());
    memcpy(message.data(), message_str.c_str(), message_str.size());

    send_socket.send(message, zmq::send_flags::none);
    RLOG_CPP(0, "Sent motor commands: " << message_str);

    jointCommandsPrev = jointCommands;
  }



  std::thread recv_thread;
  bool enableCommands = false;
  bool runLoop = false;
  bool eStop = false;
  std::vector<JointNameIndexPair> jntNameIdPairs;
  std::vector<double> jointPosition, jointVelocity, jointTorque;
  std::vector<double> jointCommands, jointCommandsPrev;
  mutable std::mutex recvMtx;
  mutable std::mutex cmdMtx;
};

}   // namespace
