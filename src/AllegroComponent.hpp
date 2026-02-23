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

/*
Here's the command data for the hands. The tmc and vmax values can be skipped if not needed.
Positions are in radians.
{
    "actuators": [
        {
            "id": "joint_1",
            "index": 0,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.06981317007977311,
            "type": "joint"
        },
        {
            "id": "joint_2",
            "index": 1,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_3",
            "index": 2,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_4",
            "index": 3,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_5",
            "index": 4,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_6",
            "index": 5,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_7",
            "index": 6,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_8",
            "index": 7,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_9",
            "index": 8,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_10",
            "index": 9,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_11",
            "index": 10,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_12",
            "index": 11,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_13",
            "index": 12,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_14",
            "index": 13,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_15",
            "index": 14,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        },
        {
            "id": "joint_16",
            "index": 15,
            "tmc": 0.1,
            "vmax": 0.2,
            "position": 0.03490658503988659,
            "type": "joint"
        }
    ],
    "quit": false,
    "robot_name": "my little robot",
    "timestamp": 1770883111.328081
}

*/




namespace aff
{


class AllegroComponent : public ComponentBase, public RoboNetworkInterface
{
public:
  AllegroComponent(EntityBase* parent,
                   double dt_commands,
                   std::string suffix,
                   std::string otherRecv,
                   std::string otherSend)
    : ComponentBase(parent), RoboNetworkInterface(otherRecv, otherSend, dt_commands)
  {
    RLOG_CPP(1, "suffix: " << suffix << " otherRecv: " << otherRecv << " otherSend: " << otherSend);
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_0_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_1_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_2_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_3_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_4_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_5_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_6_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_7_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_8_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_9_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_10_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_11_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_12_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_13_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_14_0"+suffix));
    jntNameIdPairs.push_back(Rcs::JointNameIndexPair("joint_15_0"+suffix));

    fingerTips.push_back(Rcs::BodyNameIndexPair("link_3_0_tip"+suffix));
    fingerTips.push_back(Rcs::BodyNameIndexPair("link_7_0_tip"+suffix));
    fingerTips.push_back(Rcs::BodyNameIndexPair("link_11_0_tip"+suffix));
    fingerTips.push_back(Rcs::BodyNameIndexPair("link_15_0_tip"+suffix));
    this->fingerTipPressure = std::vector<double>(4, 0.0);

    subscribe("Start", &RoboNetworkInterface::start);
    subscribe("Stop", &RoboNetworkInterface::stop);
    subscribe("UpdateGraph", &AllegroComponent::onUpdateGraph);
    subscribe("SetJointCommand", &AllegroComponent::onSetJointPosition);
    subscribe("InitFromState", &AllegroComponent::onInitFromState);
    subscribe("EmergencyStop", &AllegroComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &AllegroComponent::onEmergencyRecover);
    subscribe("EnableCommands", &AllegroComponent::onEnableCommands);

    RLOG(0, "Done constructor");
  }

  ~AllegroComponent()
  {
  }

  void onUpdateGraph(RcsGraph* graph)
  {
    std::vector<double> jntPosTmp, jntVelTmp, ftfTmp;

    {
      std::lock_guard<std::mutex> lock(this->recvMtx);
      jntPosTmp = this->jointPosition;
      jntVelTmp = this->jointVelocity;
      ftfTmp = this->fingerTipPressure;
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
      MatNd_set(graph->q, jnt->jointIndex, 0, jntPosTmp[i]);
      MatNd_set(graph->q_dot, jnt->jointIndex, 0, jntVelTmp[i]);
    }

    for (size_t i=0; i<ftfTmp.size(); ++i)
    {
      std::string col = std::string("#") + valueToColorRGB(ftfTmp[i]) + std::string("ff");
      const RcsBody* b = fingerTips[i].getBody(graph);
      if (b && b->nShapes>0)
      {
        strcpy(b->shapes[0].color, col.c_str());
      }
    }

  }

  void setWrongThumbMode(bool enable)
  {
    this->wrongThumb = enable;
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
    RLOG(1, "AllegroComponent::onInitFromState()");
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
      RLOG(0, "AllegroComponent::EmergencyStop");
    }

    this->eStop = true;
    enableCommands = false;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "AllegroComponent::EmergencyRecover");
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

      std::vector<double> q, qd, tor, ftf;

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

      if (recv_json.contains("finger_tip_force"))
      {
        ftf = recv_json["finger_tip_force"].get<std::vector<double>>();
      }

      if ((q.size()==jntNameIdPairs.size()) &&
          (qd.size()==jntNameIdPairs.size()) &&
          (tor.size()==jntNameIdPairs.size()))
      {
        if (wrongThumb)
        {
          q[12] *= -1.0;
          q[13] *= -1.0;
          q[13] += RCS_DEG2RAD(190.0);
          q[14] *= -1.0;
          q[15] *= -1.0;
        }

        std::lock_guard<std::mutex> lock(this->recvMtx);
        this->jointPosition = q;
        this->jointVelocity = qd;
        this->jointTorque = tor;
        if (!ftf.empty())
        {
          this->fingerTipPressure = ftf;
        }
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
    for (int i = 0; i < jointCommands.size(); ++i)
    {
      double q_des_i = this->jointCommands[i];

      if (wrongThumb)
      {
        if (i==12 || i==14 || i==15)
        {
          q_des_i *= -1.0;
        }
        else if (i==13)
        {
          q_des_i -= RCS_DEG2RAD(190.0);
          q_des_i *= -1.0;
        }
      }

      acts.push_back(
      {
        {"id",    "joint_" + std::to_string(i + 1)},
        {"type",  "joint"},
        {"index", i},
        {"position", q_des_i},
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

  std::string valueToColorRGB(double value)
  {
    // Clamp value between 0 and 500+
    if (value < 0)
    {
      value = 0;
    }

    int r = 0, g = 0, b = 0;

    if (value <= 124)   // Blue (0,0,255) to Cyan (0,255,255)
    {
      double t = value / 124.0;
      r = 0;
      g = static_cast<int>(255 * t);
      b = 255;
    }
    else if (value <= 249)   // Cyan (0,255,255) to Green (0,255,0)
    {
      double t = (value - 124.0) / (249.0 - 124.0);
      r = 0;
      g = 255;
      b = static_cast<int>(255 * (1 - t));
    }
    else if (value <= 375)   // Green to Yellow (255,255,0)
    {
      double t = (value - 249.0) / (375.0 - 249.0);
      r = static_cast<int>(255 * t);
      g = 255;
      b = 0;
    }
    else if (value <= 500)   // Yellow to Red (255,0,0)
    {
      double t = (value - 375.0) / (500.0 - 375.0);
      r = 255;
      g = static_cast<int>(255 * (1 - t));
      b = 0;
    }
    else   // Beyond 500: Red
    {
      r = 255;
      g = 0;
      b = 0;
    }

    // Clamp values just in case
    r = Math_iClip(r, 0, 255);
    g = Math_iClip(g, 0, 255);
    b = Math_iClip(b, 0, 255);

    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0')
        << std::setw(2) << r
        << std::setw(2) << g
        << std::setw(2) << b;

    return oss.str();
  }

  bool enableCommands = false;
  bool eStop = false;
  bool wrongThumb = false;
  std::vector<Rcs::JointNameIndexPair> jntNameIdPairs;
  std::vector<Rcs::BodyNameIndexPair> fingerTips;
  std::vector<double> jointPosition, jointVelocity, jointTorque;
  std::vector<double> jointCommands, jointCommandsPrev;
  std::vector<double> fingerTipPressure;
  mutable std::mutex recvMtx;
  mutable std::mutex cmdMtx;
};

}   // namespace
