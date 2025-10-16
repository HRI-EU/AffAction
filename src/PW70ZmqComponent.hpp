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
#include <Rcs_utilsCPP.h>

#include <mutex>



namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
class PW70ZmqComponent : public ComponentBase, public RoboNetworkInterface
{
public:
  PW70ZmqComponent(EntityBase* parent,
                   double dt_commands,
                   std::string suffix="",
                   std::string otherRecv="tcp://localhost:40006",
                   std::string otherSend="tcp://localhost:40007")
    : ComponentBase(parent), RoboNetworkInterface(otherRecv, otherSend, dt_commands)
  {
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("ptu_pan_joint"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("ptu_tilt_joint"+suffix));

    subscribe("Start", &RoboNetworkInterface::start);
    subscribe("Stop", &RoboNetworkInterface::stop);
    subscribe("UpdateGraph", &PW70ZmqComponent::onUpdateGraph);
    subscribe("SetJointCommand", &PW70ZmqComponent::onSetJointPosition);
    subscribe("InitFromState", &PW70ZmqComponent::onInitFromState);
    subscribe("EmergencyStop", &PW70ZmqComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &PW70ZmqComponent::onEmergencyRecover);
    subscribe("EnableCommands", &PW70ZmqComponent::onEnableCommands);

    RLOG(0, "Done constructor");
  }

  ~PW70ZmqComponent()
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
      RLOG(0, "No data yet received");
      return;
    }

    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      RcsJoint* jnt = jntNameIdPairs[i].getJoint(graph);
      RCHECK_MSG(jnt, "Robot joint '%s' not found in graph",
                 jntNameIdPairs[i].jointName.c_str());
      MatNd_set(graph->q, jnt->jointIndex, 0, jntPosTmp[i]);
      MatNd_set(graph->q_dot, jnt->jointIndex, 0, jntVelTmp[i]);
    }

  }

private:

  void onSetJointPosition(const MatNd* q_des)
  {
    if ((!enableCommands) || eStop)
    {
      return;
    }

    std::vector<double> q_des_vec(jntNameIdPairs.size());
    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      RCHECK_MSG(jntNameIdPairs[i].jointId!=-1, "Joint: '%s'",
                 jntNameIdPairs[i].jointName.c_str());
      q_des_vec[i] = MatNd_get(q_des, jntNameIdPairs[i].jointId, 0);
    }

    std::lock_guard<std::mutex> lock(cmdMtx);
    jointCommands = q_des_vec;
  }

  void onInitFromState(const RcsGraph* target)
  {
    RLOG(0, "PW70ZmqComponent::onInitFromState()");
    onSetJointPosition(target->q);
    jointCommandsPrev = jointCommands;

    for (size_t i = 0; i < jntNameIdPairs.size(); ++i)
    {
      const RcsJoint* jnt = jntNameIdPairs[i].getJoint(target);
      RCHECK_MSG(jnt, "Robot joint '%s' not found in graph",
                 jntNameIdPairs[i].jointName.c_str());
      RLOG(0, "Setting joint %zu to %f", i, target->q->ele[jnt->jointIndex]);
    }
  }

  void onEmergencyStop()
  {
    if (this->eStop == false)
    {
      RLOG(0, "PW70ZmqComponent::EmergencyStop");
    }

    this->eStop = true;
    enableCommands = false;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "PW70ZmqComponent::EmergencyRecover");
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
      RLOG_CPP(1, "Parsed joint angles: " << recv_json.dump(4));

      std::vector<double> q, qd;

      if (recv_json.contains("position"))
      {
        q = recv_json["position"].get<std::vector<double>>();
      }

      if (recv_json.contains("velocity"))
      {
        qd = recv_json["velocity"].get<std::vector<double>>();
      }

      if ((q.size()==2) && (qd.size()==2))
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
      {
        "joints", {
          {
            "pan", {
              {"index", 0},
              {"position_command", jointCommands[0]},
              {"novmax", 0.2},
              {"notmc", 0.1}
            }
          },
          {
            "tilt", {
              {"index", 1},
              {"position_command", jointCommands[1]},
              {"novmax", 0.2},
              {"notmc", 0.1}
            }
          }
        }
      },
      {"quit", false}
    };





    {
      std::lock_guard<std::mutex> lock(cmdMtx);
      if (!jointCommands.empty() && (jointCommands!=jointCommandsPrev))
      {
        cmdJson = payload;
      }

    }

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
