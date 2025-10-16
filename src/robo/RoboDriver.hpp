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

#ifndef AFF_ROBODRIVER_H
#define AFF_ROBODRIVER_H

#include "json.hpp"

#include <Rcs_filters.h>

#include <vector>
#include <map>
#include <mutex>



class RoboDriver
{
public:
  virtual bool setCommand(const std::string& json_msg);

protected:



  /*

  {
  "robot_name": "my little robot" ,
  "timestamp": 0.0,
  "actuators": [
    { "id": "joint_1", "type": "joint", "index": 0,
      "position": 0.4, "velocity": 0.4, "force_torque": 0.0,
      "tmc": 0.1, "vmax": 0.1
    },
    { "id": "joint_2", "type": "joint", "index": 1, "position": 0.4},
    { "id": "gripper", "type": "gripper", "index": 0, "position": 0.4, "effort": 100}
  ],
  "quit": true
  }

  */

  struct ActuatorCommand
  {
    std::string id;         // "joint_1", "gripper", ...
    std::string type;       // "joint", "gripper", ... (string as requested)
    int index = -1;         // array index for vector access

    double position = 0.0;
    double effort   = 0.0;
    double tmc      = 0.0;
    double vmax     = 0.0;

    bool has_position = false;
    bool has_effort   = false;
    bool has_tmc      = false;
    bool has_vmax     = false;
  };

  struct RobotCommand
  {
    std::string robot_name;
    double timestamp = 0.0;
    uint64_t seq = 0;
    std::vector<ActuatorCommand> actuators;
    bool quit = false;

    std::vector<const ActuatorCommand*> getActuatorsOfType(std::string actuator_type) const
    {
      std::vector<const ActuatorCommand*> out;
      for (const auto& a : actuators)
        if (a.type == actuator_type)
        {
          out.push_back(&a);
        }
      return out;
    }
  };


  static bool parse_actuator(const nlohmann::json& j, ActuatorCommand& out);
  static bool parse_robot_command(const nlohmann::json& msg, RobotCommand& cmd);
  static double getWallclockTime();
  static bool setRealTimePrio();

  virtual bool check_robot_command(RobotCommand& cmd) const = 0;
  virtual void applyCommandToFilters(const RobotCommand& robo_cmd,
                                     Rcs::RampFilterND& filt,
                                     double scale_joint_commands=1.0) const;

  mutable std::mutex cmdMtx;
  RobotCommand incomingCommand;
  bool newIncomingCommand = false;
};

#endif
