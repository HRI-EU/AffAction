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

#ifndef AFF_REACHYCOMPONENT_HPP
#define AFF_REACHYCOMPONENT_HPP

#include "ComponentBase.h"
#include "RoboNetworkInterface.hpp"
#include "json.hpp"

#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>

#include <cmath>
#include <mutex>
#include <string>
#include <vector>



namespace aff
{

/*******************************************************************************
 * Reachy Mini head client for ReachyDriver.py
 ******************************************************************************/
class ReachyComponent : public ComponentBase, public RoboNetworkInterface
{
public:
  ReachyComponent(EntityBase* parent,
                  double dt_commands,
                  std::string suffix="",
                  std::string otherRecv="tcp://localhost:40028",
                  std::string otherSend="tcp://localhost:40029",
                  bool quitDriverOnExit=true)
    : ComponentBase(parent), RoboNetworkInterface(otherRecv, otherSend, dt_commands, quitDriverOnExit)
  {
    // Driver indices 0, 1 and 2 are yaw, pitch and roll, respectively.
    jntNameIdPairs.emplace_back("ptu_pan_joint"+suffix);
    jntNameIdPairs.emplace_back("ptu_tilt_joint"+suffix);
    jntNameIdPairs.emplace_back("ptu_roll_joint"+suffix);

    subscribe("Start", &RoboNetworkInterface::start);
    subscribe("Stop", &RoboNetworkInterface::stop);
    subscribe("UpdateGraph", &ReachyComponent::onUpdateGraph);
    subscribe("SetJointCommand", &ReachyComponent::onSetJointPosition);
    subscribe("InitFromState", &ReachyComponent::onInitFromState);
    subscribe("EmergencyStop", &ReachyComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &ReachyComponent::onEmergencyRecover);
    subscribe("EnableCommands", &ReachyComponent::onEnableCommands);
  }

  ~ReachyComponent() override
  {
    stop();
  }

  void stop() override
  {
    RoboNetworkInterface::stop();
    std::lock_guard<std::mutex> lock(cmdMtx);
    jointCommands.clear();
  }

private:
  void onUpdateGraph(RcsGraph* graph)
  {
    std::vector<double> q, qd;
    {
      std::lock_guard<std::mutex> lock(recvMtx);
      q = jointPosition;
      qd = jointVelocity;
    }

    if (q.empty())
    {
      return;
    }

    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      const RcsJoint* jnt = jntNameIdPairs[i].getJoint(graph);
      RCHECK_MSG(jnt, "Robot joint '%s' not found in graph",
                 jntNameIdPairs[i].getJointName().c_str());
      MatNd_set(graph->q, jnt->jointIndex, 0, q[i]);
      MatNd_set(graph->q_dot, jnt->jointIndex, 0, qd[i]);
    }
  }

  void onSetJointPosition(const MatNd* q_des)
  {
    std::lock_guard<std::mutex> lock(cmdMtx);
    if (!enableCommands || eStop)
    {
      return;
    }

    std::vector<double> q(jntNameIdPairs.size());
    for (size_t i=0; i<jntNameIdPairs.size(); ++i)
    {
      const int jointIndex = jntNameIdPairs[i].getJointIndex();
      RCHECK_MSG(jointIndex!=-1, "Joint: '%s'",
                 jntNameIdPairs[i].getJointName().c_str());
      q[i] = MatNd_get(q_des, jointIndex, 0);
      if (!std::isfinite(q[i]))
      {
        return;
      }
    }
    jointCommands.swap(q);
  }

  void onInitFromState(const RcsGraph* graph)
  {
    for (auto& pair : jntNameIdPairs)
    {
      const RcsJoint* jnt = pair.getJoint(graph);
      RCHECK_MSG(jnt, "Robot joint '%s' not found in graph", pair.getJointName().c_str());
    }

    std::lock_guard<std::mutex> lock(cmdMtx);
    jointCommands.clear();
  }

  void onEmergencyStop()
  {
    std::lock_guard<std::mutex> lock(cmdMtx);
    eStop = true;
    enableCommands = false;
    jointCommands.clear();
  }

  void onEmergencyRecover()
  {
    std::lock_guard<std::mutex> lock(cmdMtx);
    eStop = false;
    enableCommands = true;
  }

  void onEnableCommands()
  {
    std::lock_guard<std::mutex> lock(cmdMtx);
    enableCommands = true;
  }

  bool process_incoming_message(const std::string& recv_msg) override
  {
    try
    {
      const auto feedback = nlohmann::json::parse(recv_msg);
      auto q = feedback.at("position").get<std::vector<double>>();
      auto qd = feedback.at("velocity").get<std::vector<double>>();

      if (q.size()!=jntNameIdPairs.size() || qd.size()!=jntNameIdPairs.size())
      {
        return false;
      }

      for (size_t i=0; i<q.size(); ++i)
      {
        if (!std::isfinite(q[i]) || !std::isfinite(qd[i]))
        {
          return false;
        }
      }

      std::lock_guard<std::mutex> lock(recvMtx);
      jointPosition.swap(q);
      jointVelocity.swap(qd);
      return true;
    }
    catch (const nlohmann::json::exception& e)
    {
      RLOG_CPP(1, "Invalid Reachy feedback: " << e.what());
      return false;
    }
  }

  std::string generate_command_message() override
  {
    std::lock_guard<std::mutex> lock(cmdMtx);
    if (!enableCommands || eStop || jointCommands.empty())
    {
      return std::string();
    }

    nlohmann::json payload =
    {
      {"robot_name", "Reachy"},
      {"timestamp", Timer_getSystemTime()},
      {"actuators", nlohmann::json::array()},
      {"quit", false}
    };

    // Resend the latest target because PUB may drop messages during connection.
    auto& actuators = payload["actuators"];
    for (size_t i=0; i<jointCommands.size(); ++i)
    {
      actuators.push_back(
      {
        {"id", "joint_" + std::to_string(i + 1)},
        {"type", "joint"},
        {"index", i},
        {"position", jointCommands[i]}
      });
    }

    return payload.dump();
  }

  bool enableCommands = false;
  bool eStop = false;
  std::vector<Rcs::JointNameIndexPair> jntNameIdPairs;
  std::vector<double> jointPosition, jointVelocity;
  std::vector<double> jointCommands;
  std::mutex recvMtx, cmdMtx;
};

}   // namespace aff

#endif   // AFF_REACHYCOMPONENT_HPP
