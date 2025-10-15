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

#include "RoboDriver.hpp"




double RoboDriver::getMaxVel(size_t index) const
{
  return getMaxVel()[index];
}

// ---- tiny helpers: safe getters (never throw) ----
static inline bool j_get_string(const nlohmann::json& j, const char* key, std::string& out)
{
  auto it = j.find(key);
  if (it == j.end() || it->is_null() || !it->is_string())
  {
    return false;
  }
  out = it->get<std::string>();
  return true;
}

static inline bool j_get_int(const nlohmann::json& j, const char* key, int& out)
{
  auto it = j.find(key);
  if (it == j.end() || it->is_null() || !it->is_number_integer())
  {
    return false;
  }
  out = it->get<int>();
  return true;
}

static inline bool j_get_number(const nlohmann::json& j, const char* key, double& out)
{
  auto it = j.find(key);
  if (it == j.end() || it->is_null() || !it->is_number())
  {
    return false;
  }
  double v = it->get<double>();
  if (!std::isfinite(v))
  {
    return false;  // reject NaN/inf
  }
  out = v;
  return true;
}

inline void j_get_seq_u64(const nlohmann::json& j, const char* key, uint64_t& out)
{
  auto it = j.find(key);
  if (it == j.end() || it->is_null())
  {
    return;
  }
  if (it->is_number_unsigned())
  {
    out = it->get<uint64_t>();
  }
  else if (it->is_number_integer())
  {
    auto s = it->get<int64_t>();
    if (s >= 0)
    {
      out = static_cast<uint64_t>(s);
    }
  }
}

/* static */ bool RoboDriver::parse_actuator(const nlohmann::json& j, RoboDriver::ActuatorCommand& out)
{
  out = RoboDriver::ActuatorCommand{};

  if (!j.is_object())
  {
    return false;
  }

  int idx = -1;
  if (!j_get_int(j, "index", idx))
  {
    return false;
  }
  out.index = idx;

  j_get_string(j, "id",   out.id);
  j_get_string(j, "type", out.type);

  double tmp;
  if (j_get_number(j, "position", tmp))
  {
    out.position = tmp;
    out.has_position = true;
  }

  if (j_get_number(j, "effort", tmp))
  {
    out.effort = tmp;
    out.has_effort = true;
  }

  if (j_get_number(j, "vmax", tmp))
  {
    out.vmax = tmp;
    out.has_vmax = true;
  }

  if (j_get_number(j, "tmc", tmp))
  {
    out.tmc = tmp;
    out.has_tmc = true;
  }

  return true;
}



/* static */ bool RoboDriver::parse_robot_command(const nlohmann::json& msg, RoboDriver::RobotCommand& cmd)
{
  if (!msg.is_object())
  {
    return false;
  }

  // If "actuators" exists, it must be an array
  if (msg.contains("actuators") && !msg["actuators"].is_array())
  {
    return false;
  }

  RoboDriver::RobotCommand tmp{};  // build into tmp, then move-assign

  j_get_string(msg, "robot_name", tmp.robot_name);
  j_get_number(msg, "timestamp", tmp.timestamp);
  j_get_seq_u64(msg, "seq", tmp.seq);

  auto it_quit = msg.find("quit");
  if (it_quit != msg.end() && it_quit->is_boolean())
  {
    tmp.quit = it_quit->get<bool>();
  }

  if (!msg.contains("actuators"))
  {
    cmd = std::move(tmp);
    return true;
  }

  const auto& actuators_arr = msg["actuators"];  // already validated as array
  tmp.actuators.reserve(actuators_arr.size());

  for (const auto& aj : actuators_arr)
  {
    RoboDriver::ActuatorCommand a{};
    if (RoboDriver::parse_actuator(aj, a))
    {
      tmp.actuators.push_back(std::move(a));
    }
  }

  cmd = std::move(tmp);

  return true;
}
