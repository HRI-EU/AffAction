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

/*******************************************************************************
 *
 ******************************************************************************/
class NetworkComponent
{
public:

  virtual ~NetworkComponent()
  {
    stop();
  }

  virtual void start()
  {
    if (runLoop)
    {
      RLOG(0, "NetworkComponent already started - doing nothing");
      return;
    }

    runLoop = true;
    std::string recvEndpoint = "tcp://localhost:5555";
    recv_thread = std::thread(&NetworkComponent::recvThreadFunc, this, recvEndpoint);

    while (!isInitialized.load(std::memory_order_acquire))
    {
      fprintf(stderr, ".");
      fflush(stderr);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::string sendEndpoint = "tcp://*:5556";
    send_thread = std::thread(&NetworkComponent::sendThreadFunc, this, sendEndpoint);
    RLOG(0, "NetworkComponent: All threads started");
  }

  virtual void stop()
  {
    if (!runLoop)
    {
      RLOG(0, "NetworkComponent already stopped - doing nothing");
      return;
    }

    runLoop = false;

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

    RLOG(0, "NetworkComponent stopped");
  }

  void recvThreadFunc(std::string recvEndpoint)
  {

    try
    {
      const double max_timeout = 2.0;   // seconds
      double t_watchdog = Timer_getSystemTime();
      zmq::context_t context(1);
      zmq::socket_t sub(context, zmq::socket_type::sub);

      sub.connect(recvEndpoint);
      sub.set(zmq::sockopt::subscribe, "");
      sub.set(zmq::sockopt::rcvtimeo, 100);   // 100 ms timeout to catch runLoop

      while (runLoop && !watchDogTriggered)
      {
        zmq::message_t msg;
        if (sub.recv(msg, zmq::recv_flags::none))
        {
          t_watchdog = Timer_getSystemTime();
          std::string msg_str(static_cast<char*>(msg.data()), msg.size());
          const bool dataOk = process_incoming_message(msg_str);

          if (dataOk)
          {
            isInitialized.store(true, std::memory_order_release);
          }
        }

        RLOG(1, "Checking watchdog: %f %f %f",
             Timer_getSystemTime(),
             t_watchdog,
             Timer_getSystemTime()-t_watchdog);

        if ((Timer_getSystemTime() - t_watchdog > max_timeout) &&
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


  void sendThreadFunc(std::string sendEndpoint)
  {
    zmq::context_t context(1);
    zmq::socket_t send_socket(context, zmq::socket_type::pub);
    send_socket.bind(sendEndpoint);

    while (runLoop && !watchDogTriggered)
    {
      nlohmann::json cmdJson = compile_outgoing_message();

      if (!cmdJson.empty())
      {
        std::string message_str = cmdJson.dump();
        zmq::message_t message(message_str.size());
        memcpy(message.data(), message_str.c_str(), message_str.size());

        send_socket.send(message, zmq::send_flags::none);
        RLOG_CPP(0, "Sent motor commands: " << message_str);
      }
    }

    RLOG(0, "Exiting sendThreadFunc()");
  }



protected:

  virtual bool process_incoming_message(const std::string& recv_msg) = 0;
  virtual nlohmann::json compile_outgoing_message() = 0;

  mutable std::atomic<bool> isInitialized{false};
  bool watchDogTriggered = false;
  bool runLoop = false;
  std::thread recv_thread;
  std::thread send_thread;
};










/*******************************************************************************
 *
 ******************************************************************************/
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

  std::string jointName;
  int jointId;
};

class KortexComponent : public ComponentBase, public NetworkComponent
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

    subscribe("Start", &NetworkComponent::start);
    subscribe("Stop", &NetworkComponent::stop);
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
    double gripper_angle = 0.0;

    {
      std::lock_guard<std::mutex> lock(this->recvMtx);
      jntPosTmp = this->jointPosition;
      jntVelTmp = this->jointVelocity;
      gripper_angle = RCS_DEG2RAD(0.4*this->gripper_position);
    }

    if (jntPosTmp.size()!=jntNameIdPairs.size() || jntVelTmp.size()!=jntNameIdPairs.size())
    {
      RLOG(0, "No data yet received");
      return;
    }

    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      RcsJoint* jnt = jntNameIdPairs[i].getJoint(graph);
      RCHECK_MSG(jnt, "Robot arm joint '%s' not found in graph",
                 jntNameIdPairs[i].jointName.c_str());
      MatNd_set(graph->q, jnt->jointIndex, 0, jntPosTmp[i]);
      MatNd_set(graph->q_dot, jnt->jointIndex, 0, jntVelTmp[i]);
    }

    for (size_t i=0; i<gripperNameIdPairs.size(); ++i)
    {
      RcsJoint* jnt = gripperNameIdPairs[i].getJoint(graph);
      RCHECK_MSG(jnt, "Gripper joint '%s' not found in graph",
                 gripperNameIdPairs[i].jointName.c_str());
      MatNd_set(graph->q, jnt->jointIndex, 0, gripper_angle);
    }

    //estimateTouch(graph);
  }

private:

  void onSetJointPosition(const MatNd* q_des)
  {
    if ((!enableCommands) || eStop)
    {
      return;
    }

    std::vector<double> q7(jntNameIdPairs.size());
    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      RCHECK_MSG(jntNameIdPairs[i].jointId!=-1, "Joint: '%s'",
                 jntNameIdPairs[i].jointName.c_str());
      q7[i] = MatNd_get(q_des, jntNameIdPairs[i].jointId, 0);
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

  bool process_incoming_message(const std::string& recv_msg)
  {
    nlohmann::json recv_json;
    bool membersInitialized = false;

    try
    {
      // Try to parse the JSON string safely
      recv_json = nlohmann::json::parse(recv_msg);

      // If successful, process the parsed JSON data
      RLOG_CPP(1, "Parsed joint angles: " << recv_json.dump(4));

      std::vector<double> q, qd, tor;
      double q_grip = -1.0;

      if (recv_json.contains("position"))
      {
        q = recv_json["position"].get<std::vector<double>>();
      }

      if (recv_json.contains("gripper_position"))
      {
        q_grip = recv_json["gripper_position"].get<double>();
      }

      if (recv_json.contains("velocity"))
      {
        qd = recv_json["velocity"].get<std::vector<double>>();
      }

      if (recv_json.contains("torque"))
      {
        tor = recv_json["torque"].get<std::vector<double>>();
      }

      // Further processing of joint angles here...
      // std::vector<double> tool_wrench;
      // if (recv_json.contains("tool_wrench"))
      // {
      //   tool_wrench = recv_json["tool_wrench"].get<std::vector<double>>();
      // }

      if ((q.size()==7) && (qd.size()==7) && (tor.size()==7) && (q_grip!=-1))
      {
        std::lock_guard<std::mutex> lock(this->recvMtx);
        this->jointPosition = q;
        this->jointVelocity = qd;
        this->jointTorque = tor;
        this->gripper_position = q_grip;
        membersInitialized = true;
      }

    }
    catch (const nlohmann::json::parse_error& e)
    {
      RLOG_CPP(0, "JSON parsing error: " << e.what());
      RLOG_CPP(0, "Invalid JSON string: " << recv_json);
    }

    return membersInitialized;
  }

  nlohmann::json compile_outgoing_message()
  {
    Timer_waitDT(0.01);               // 100 Hz command rate
    nlohmann::json cmdJson;

    if (!enableCommands || jointCommands.empty() || (jointCommands==jointCommandsPrev))
    {
      return cmdJson;
    }

    RLOG_CPP(0, "Sending motor commands");

    {
      std::lock_guard<std::mutex> lock(cmdMtx);
      if (!jointCommands.empty() && (jointCommands!=jointCommandsPrev))
      {
        cmdJson["q_des"] = jointCommands;
      }

      if (gripper_command!=gripper_command_prev)
      {
        cmdJson["gripper_command"] = RCS_RAD2DEG(gripper_command)/0.4;
        cmdJson["gripper_force"] = gripper_force;
      }
    }

    jointCommandsPrev = jointCommands;
    gripper_command_prev = gripper_command;

    return cmdJson;
  }



  bool enableCommands = false;
  bool eStop = false;
  int torqueTic = -1;
  std::vector<JointNameIndexPair> jntNameIdPairs;
  std::vector<JointNameIndexPair> gripperNameIdPairs;
  std::vector<double> jointPosition, jointVelocity, jointTorque, gravityTorque;
  double gripper_position = 0.0;   // 0: open, 100: closed
  double gripper_command = 0.0;
  double gripper_command_prev = 0.0;
  double gripper_force = 100.0;
  std::vector<double> jointCommands, jointCommandsPrev;
  mutable std::mutex recvMtx;
  mutable std::mutex cmdMtx;
};

}   // namespace
