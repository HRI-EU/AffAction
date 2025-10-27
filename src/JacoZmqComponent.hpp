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

#include "RoboNetworkInterface.hpp"
#include "ComponentBase.h"
#include "json.hpp"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_math.h>
#include <Rcs_dynamics.h>
#include <Rcs_utilsCPP.h>

#include <mutex>



namespace aff
{


class JacoZmqComponent : public ComponentBase, public RoboNetworkInterface
{
public:
  JacoZmqComponent(EntityBase* parent,
                   double dt_commands,
                   std::string roboType,// "Jaco6", "Jaco7"
                   std::string suffix,//="",
                   std::string otherRecv,
                   std::string otherSend)
    : ComponentBase(parent), RoboNetworkInterface(otherRecv, otherSend, dt_commands)
  {
    RLOG_CPP(1, "suffix: " << suffix << " otherRecv: " << otherRecv << " otherSend: " << otherSend);

    if (roboType=="Jaco6")
    {
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_joint_1"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_joint_2"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_joint_3"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_joint_4"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_joint_5"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_joint_6"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_nonexisting_4"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_joint_finger_1"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_joint_finger_2"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2n6s300_joint_finger_3"+suffix));
    }
    else if (roboType=="Jaco7")
    {
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_1_right"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_2_right"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_3_right"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_4_right"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_5_right"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_6_right"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_7_right"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_finger_1_right"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_finger_2_right"+suffix));
      jntNameIdPairs.push_back(Rcs::JointNameIndexPair("j2s7s300_joint_finger_3_right"+suffix));
    }
    else
    {
      RFATAL("Unknown roboType: '%s' - must be Jaco6 or Jcao7", roboType.c_str());
    }


    subscribe("Start", &RoboNetworkInterface::start);
    subscribe("Stop", &RoboNetworkInterface::stop);
    subscribe("UpdateGraph", &JacoZmqComponent::onUpdateGraph);
    subscribe("SetJointCommand", &JacoZmqComponent::onSetJointPosition);
    subscribe("InitFromState", &JacoZmqComponent::onInitFromState);
    subscribe("EmergencyStop", &JacoZmqComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &JacoZmqComponent::onEmergencyRecover);
    subscribe("EnableCommands", &JacoZmqComponent::onEnableCommands);

    RLOG(0, "Done constructor");
  }

  ~JacoZmqComponent()
  {
  }

  void onUpdateGraph(RcsGraph* graph)
  {
    std::vector<double> jntPosTmp, jntVelTmp;

    {
      std::lock_guard<std::mutex> lock(this->recvMtx);
      jntPosTmp = this->jointPosition;
      jntVelTmp = this->jointVelocity;
    }

    if (jntPosTmp.size()!=jntNameIdPairs.size() || jntVelTmp.size()!=jntNameIdPairs.size())
    {
      RLOG(1, "No data yet received");
      return;
    }

    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      RcsJoint* jnt = jntNameIdPairs[i].getJoint(graph);
      RCHECK_MSG(jnt, "Joint '%s' not found in graph",
                 jntNameIdPairs[i].jointName.c_str());

      if (!jnt->constrained)
      {
        MatNd_set(graph->q, jnt->jointIndex, 0, jntPosTmp[i]);
        MatNd_set(graph->q_dot, jnt->jointIndex, 0, jntVelTmp[i]);
      }
    }

  }

private:

  void onSetJointPosition(const MatNd* q_des)
  {
    if ((!enableCommands) || eStop)
    {
      return;
    }

    std::vector<double> q(jntNameIdPairs.size());
    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      RCHECK_MSG(jntNameIdPairs[i].jointId!=-1, "Joint: '%s'",
                 jntNameIdPairs[i].jointName.c_str());
      q[i] = MatNd_get(q_des, jntNameIdPairs[i].jointId, 0);
    }

    std::lock_guard<std::mutex> lock(cmdMtx);
    this->jointCommands = q;
  }

  void onInitFromState(const RcsGraph* target)
  {
    RLOG(1, "JacoZmqComponent::onInitFromState()");
    onSetJointPosition(target->q);
    this->jointCommandsPrev = this->jointCommands;

    for (size_t i = 0; i < jntNameIdPairs.size(); ++i)
    {
      const RcsJoint* jnt = jntNameIdPairs[i].getJoint(target);
      RCHECK_MSG(jnt, "Joint '%s' not found in graph",
                 jntNameIdPairs[i].jointName.c_str());
      RLOG(1, "Setting joint %zu to %f", i, target->q->ele[jnt->jointIndex]);
    }
  }

  void onEmergencyStop()
  {
    if (this->eStop == false)
    {
      RLOG(0, "JacoZmqComponent::EmergencyStop");
    }

    this->eStop = true;
    enableCommands = false;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "JacoZmqComponent::EmergencyRecover");
    this->eStop = false;
    enableCommands = true;
  }

  void onEnableCommands()
  {
    enableCommands = true;
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
      RLOG_CPP(5, "Parsed joint angles: " << recv_json.dump(4));

      std::vector<double> q, qd;

      if (recv_json.contains("position"))
      {
        q = recv_json["position"].get<std::vector<double>>();
      }

      if (recv_json.contains("velocity"))
      {
        qd = recv_json["velocity"].get<std::vector<double>>();
      }

      if ((q.size()==jntNameIdPairs.size()) &&
          (qd.size()==jntNameIdPairs.size()))
      {
        std::lock_guard<std::mutex> lock(this->recvMtx);
        this->jointPosition = q;
        this->jointVelocity = qd;
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

  std::string generate_command_message()
  {
    nlohmann::json cmdJson;

    if (!enableCommands || jointCommands.empty() || (jointCommands==jointCommandsPrev))
    {
      return std::string();
    }

    nlohmann::json payload =
    {
      {"robot_name", "Jaco Gen2"},
      {"timestamp",  Timer_getSystemTime()},
      {"actuators", nlohmann::json::array()},
      {"quit", false}
    };

    auto& acts = payload["actuators"];
    for (int i = 0; i < jointCommands.size(); ++i)
    {
      acts.push_back(
      {
        {"id",    "joint_" + std::to_string(i + 1)},
        {"type",  "joint"},
        {"index", i},
        {"position", jointCommands[i]},
        {"no_vmax", 0.2},
        {"no_tmc",  0.1}
      });
    }

    {
      std::lock_guard<std::mutex> lock(cmdMtx);
      if (!jointCommands.empty() && (jointCommands!=jointCommandsPrev))
      {
        cmdJson = payload;
      }
    }

    // Memorize previous state
    jointCommandsPrev = jointCommands;

    return cmdJson.dump();
  }

  bool enableCommands = false;
  bool eStop = false;
  std::vector<Rcs::JointNameIndexPair> jntNameIdPairs;
  std::vector<double> jointPosition, jointVelocity;
  std::vector<double> jointCommands, jointCommandsPrev;
  mutable std::mutex recvMtx;
  mutable std::mutex cmdMtx;
};

}   // namespace
