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


class FrankaComponent : public ComponentBase, public RoboNetworkInterface
{
public:
  FrankaComponent(EntityBase* parent,
                  double dt_commands,
                  std::string suffix,
                  std::string otherRecv,
                  std::string otherSend,
                  bool quitDriverOnExit_=true)
    : ComponentBase(parent), RoboNetworkInterface(otherRecv, otherSend, dt_commands, quitDriverOnExit_)
  {
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("fr3_joint1"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("fr3_joint2"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("fr3_joint3"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("fr3_joint4"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("fr3_joint5"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("fr3_joint6"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("fr3_joint7"+suffix));

    baseFtsName = "fts_base_" + suffix;
    eeFtsName = "fts_ee_" + suffix;

    subscribe("Start", &RoboNetworkInterface::start);
    subscribe("Stop", &RoboNetworkInterface::stop);
    subscribe("UpdateGraph", &FrankaComponent::onUpdateGraph);
    subscribe("SetJointCommand", &FrankaComponent::onSetJointPosition);
    subscribe("InitFromState", &FrankaComponent::onInitFromState);
    subscribe("EmergencyStop", &FrankaComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &FrankaComponent::onEmergencyRecover);
    subscribe("EnableCommands", &FrankaComponent::onEnableCommands);

    RLOG(0, "Done constructor");
  }

  ~FrankaComponent()
  {
    stop();
  }

  void onUpdateGraph(RcsGraph* graph)
  {
    std::vector<double> jntPosTmp, jntVelTmp, wrench_base, wrench_ee;

    {
      std::lock_guard<std::mutex> lock(this->recvMtx);
      jntPosTmp = this->jointPosition;
      jntVelTmp = this->jointVelocity;
      wrench_base = this->wrench_in_base;
      wrench_ee = this->wrench_in_ee;
    }

    if (jntPosTmp.size()!=jntNameIdPairs.size() || jntVelTmp.size()!=jntNameIdPairs.size())
    {
      RLOG(1, "No data yet received");
      return;
    }

    // Update joint angles and velocities
    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      RcsJoint* jnt = jntNameIdPairs[i].getJoint(graph);
      RCHECK_MSG(jnt, "Robot arm joint '%s' not found in graph",
                 jntNameIdPairs[i].jointName.c_str());
      MatNd_set(graph->q, jnt->jointIndex, 0, jntPosTmp[i]);
      MatNd_set(graph->q_dot, jnt->jointIndex, 0, jntVelTmp[i]);
    }

    // Update virtual force sensors
    if (!wrench_base.empty())
    {
      RcsSensor* s = RcsGraph_getSensorByName(graph, baseFtsName.c_str());
      if (s && s->type==RCSSENSOR_LOAD_CELL)
      {
        VecNd_copy(s->rawData->ele, wrench_base.data(), 6);
      }
    }

    if (!wrench_ee.empty())
    {
      RcsSensor* s = RcsGraph_getSensorByName(graph, eeFtsName.c_str());
      if (s && s->type==RCSSENSOR_LOAD_CELL)
      {
        VecNd_copy(s->rawData->ele, wrench_ee.data(), 6);
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

    std::vector<double> q7(jntNameIdPairs.size());
    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      RCHECK_MSG(jntNameIdPairs[i].jointId!=-1, "Joint: '%s'",
                 jntNameIdPairs[i].jointName.c_str());
      q7[i] = MatNd_get(q_des, jntNameIdPairs[i].jointId, 0);
    }

    std::lock_guard<std::mutex> lock(cmdMtx);
    this->jointCommands = q7;
  }

  void onInitFromState(const RcsGraph* target)
  {
    RLOG(0, "FrankaComponent::onInitFromState()");
    onSetJointPosition(target->q);
    this->jointCommandsPrev = this->jointCommands;

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
      RLOG(0, "FrankaComponent::EmergencyStop");
    }

    this->eStop = true;
    enableCommands = false;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "FrankaComponent::EmergencyRecover");
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

      std::vector<double> q, qd, tor, wrench_base, wrench_ee;

      if (recv_json.contains("position"))
      {
        q = recv_json["position"].get<std::vector<double>>();
      }

      if (recv_json.contains("velocity"))
      {
        qd = recv_json["velocity"].get<std::vector<double>>();
      }

      if (recv_json.contains("torque"))
      {
        tor = recv_json["torque"].get<std::vector<double>>();
      }

      if (recv_json.contains("wrench_in_base"))
      {
        wrench_base = recv_json["wrench_in_base"].get<std::vector<double>>();
      }

      if (recv_json.contains("wrench_in_ee"))
      {
        wrench_ee = recv_json["wrench_in_ee"].get<std::vector<double>>();
      }

      if ((q.size()==7) && (qd.size()==7) && (tor.size()==7))
      {
        std::lock_guard<std::mutex> lock(this->recvMtx);
        this->jointPosition = q;
        this->jointVelocity = qd;
        this->jointTorque = tor;
        this->wrench_in_base = wrench_base;
        this->wrench_in_ee = wrench_ee;
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
      {"robot_name", "my little robot"},
      {"timestamp",  Timer_getSystemTime()},
      {"actuators", nlohmann::json::array()},
      {"quit", false}
    };

    auto& acts = payload["actuators"];
    for (int i = 0; i < 7; ++i)
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
  std::string baseFtsName, eeFtsName;
  std::vector<double> jointPosition, jointVelocity, jointTorque;
  std::vector<double> wrench_in_base, wrench_in_ee;
  std::vector<double> jointCommands, jointCommandsPrev;
  mutable std::mutex recvMtx;
  mutable std::mutex cmdMtx;
};

}   // namespace
