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
#include <Rcs_math.h>
#include <Rcs_dynamics.h>

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
  KortexComponent(EntityBase* parent, std::string suffix="")
    : ComponentBase(parent)
  {
    jntNameIdPairs.push_back(JointNameIndexPair("joint_1"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_2"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_3"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_4"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_5"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_6"+suffix));
    jntNameIdPairs.push_back(JointNameIndexPair("joint_7"+suffix));

    gripperNameIdPairs.push_back(JointNameIndexPair("finger_joint"+suffix));

    RLOG(0, "Subscribing to events");
    subscribe("Start", &KortexComponent::onStart);
    subscribe("Stop", &KortexComponent::onStop);
    subscribe("UpdateGraph", &KortexComponent::onUpdateGraph);
    subscribe("SetJointCommand", &KortexComponent::onSetJointPosition);
    subscribe("InitFromState", &KortexComponent::onInitFromState);
    subscribe("EmergencyStop", &KortexComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &KortexComponent::onEmergencyRecover);
    subscribe("EnableCommands", &KortexComponent::onEnableCommands);
    subscribe("SetGripperForce", &KortexComponent::onSetGripperForce);

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

  void estimateTouch(const RcsGraph* graph)
  {
    // Torques read back from the robot
    MatNd* T_robo = MatNd_create(jointTorque.size(), 1);
    VecNd_copy(T_robo->ele, jointTorque.data(), T_robo->size);

    // Gravity compensation model. We ignore the gripper dof
    MatNd* T_gravity = MatNd_create(graph->nJ, 1);
    double gravityVec[3] = {0.0, 0.0, -9.81};
    RcsGraph_computeGravityTorque(graph, gravityVec, T_gravity);
    T_gravity->m--;

    if (T_robo->m!=T_gravity->m)
    {
      RLOG(0, "Mismatch in torque vector dimensions: robo: %d   gravity: %d", T_robo->m, T_gravity->m);
      return;
    }

    REXEC(2)
    {
      RLOG(0, "gravity   sensor");
      MatNd_printTwoArraysDiff(T_gravity, T_robo, 3);
    }


    if (torqueTic==-1)
    {
      MatNd_subSelf(T_robo, T_gravity);
      const double trq = VecNd_sqrLength(T_robo->ele, T_robo->size);
      RLOG(1, "Torque: %f", trq);
      if (trq>40.0)
      {
        torqueTic++;
        getEntity()->publish("Speak", std::string("Hey, get your fingers away from me!"));
      }
    }
    else if (torqueTic>100)
    {
      torqueTic = -1;
    }
    else
    {
      torqueTic++;
    }


    MatNd_destroy(T_gravity);
    MatNd_destroy(T_robo);
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
      RCHECK_MSG(jnt, "Robot arm joint '%s' not found in graph", jntNameIdPairs[i].jointName.c_str());

      if (i<jntPosTmp.size())
      {
        MatNd_set(graph->q, jnt->jointIndex, 0, jntPosTmp[i]);
      }

      if (i<jntVelTmp.size())
      {
        MatNd_set(graph->q_dot, jnt->jointIndex, 0, jntVelTmp[i]);
      }
    }

    for (size_t i=0; i<gripperNameIdPairs.size(); ++i)
    {
      RcsJoint* jnt = gripperNameIdPairs[i].getJoint(graph);
      RCHECK_MSG(jnt, "Gripper joint '%s' not found in graph", gripperNameIdPairs[i].jointName.c_str());
      const double gripper_angle = RCS_DEG2RAD(0.4*gripper_position);
      MatNd_set(graph->q, jnt->jointIndex, 0, gripper_angle);
    }

    estimateTouch(graph);
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

      double gripper_des = 0.0;
      for (size_t i=0; i<gripperNameIdPairs.size(); ++i)
      {
        RCHECK_MSG(gripperNameIdPairs[i].jointId!=-1, "Joint: '%s'",
                   gripperNameIdPairs[i].jointName.c_str());
        gripper_des = MatNd_get(q_des, gripperNameIdPairs[i].jointId, 0);
      }



      std::lock_guard<std::mutex> lock(cmdMtx);
      jointCommands = q7;
      gripper_command = gripper_des;
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

  void onSetGripperForce(double force)   // between 0 and 100
  {
    std::lock_guard<std::mutex> lock(cmdMtx);
    gripper_force = force;
  }

  void recvThreadFunc()
  {
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



      Timer_waitDT(0.01);
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

      if (recv_json.contains("position"))
      {
        jointPosition = recv_json["position"].get<std::vector<double>>();
      }

      if (recv_json.contains("velocity"))
      {
        jointVelocity = recv_json["velocity"].get<std::vector<double>>();
      }

      if (recv_json.contains("torque"))
      {
        jointTorque = recv_json["torque"].get<std::vector<double>>();
      }

      if (recv_json.contains("gripper_position"))
      {
        gripper_position = recv_json["gripper_position"].get<double>();
      }


      // Further processing of joint angles here...
      std::vector<double> tool_wrench;
      if (recv_json.contains("tool_wrench"))
      {
        tool_wrench = recv_json["tool_wrench"].get<std::vector<double>>();
      }

    }
    catch (const nlohmann::json::parse_error& e)
    {
      RLOG_CPP(0, "JSON parsing error: " << e.what());
      RLOG_CPP(0, "Invalid JSON string: " << recv_json);
    }


  }

  void send_motor_commands(zmq::socket_t& send_socket)
  {

    if (jointCommands.empty() || (jointCommands==jointCommandsPrev))
    {
      return;
    }

    RLOG_CPP(0, "Sending motor commands");

    bool isNewCommand = false;
    nlohmann::json cmdJson;

    {
      std::lock_guard<std::mutex> lock(cmdMtx);
      if (!jointCommands.empty() && (jointCommands!=jointCommandsPrev))
      {
        cmdJson["q_des"] = jointCommands;
        isNewCommand = true;
      }

      if (gripper_command!=gripper_command_prev)
      {
        cmdJson["gripper_command"] = RCS_RAD2DEG(gripper_command)/0.4;
        cmdJson["gripper_force"] = gripper_force;
        isNewCommand = true;
      }
    }

    if (isNewCommand)
    {
      std::string message_str = cmdJson.dump();
      zmq::message_t message(message_str.size());
      memcpy(message.data(), message_str.c_str(), message_str.size());

      send_socket.send(message, zmq::send_flags::none);
      RLOG_CPP(0, "Sent motor commands: " << message_str);
    }

    jointCommandsPrev = jointCommands;
    gripper_command_prev = gripper_command;
  }



  std::thread recv_thread;
  bool enableCommands = false;
  bool runLoop = false;
  bool eStop = false;
  int torqueTic = -1;
  std::vector<JointNameIndexPair> jntNameIdPairs;
  std::vector<JointNameIndexPair> gripperNameIdPairs;
  std::vector<double> jointPosition, jointVelocity, jointTorque, gravityTorque;
  double gripper_position = 0.0;   // 0: open, 100: closed
  double gripper_command = 0.0;
  double gripper_command_prev = 0.0;
  double gripper_force = 10.0;
  std::vector<double> jointCommands, jointCommandsPrev;
  mutable std::mutex recvMtx;
  mutable std::mutex cmdMtx;
};

}   // namespace
