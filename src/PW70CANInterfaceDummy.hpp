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

#ifndef PW70CANINTERFACEDUMMY_H
#define PW70CANINTERFACEDUMMY_H

#include <cmath>
#include <thread>
#include <vector>
#include <functional>
#include <thread>
#include <mutex>
#include <iostream>
#include <chrono>

namespace aff
{

class PW70CANInterface
{
public:
  // Constructor and Destructor
  PW70CANInterface(std::function<void(double, double, void*)> limit_check_callback_,
                   std::function<void(double, double, double, void*)> position_callback_,
                   void* param,
                   int freq) :
    limit_check_callback(limit_check_callback_),
    position_callback(position_callback_),
    callbackParam(param)
  {
    recv_thread = std::thread(&PW70CANInterface::receive_messages, this, freq);
  }

  ~PW70CANInterface()
  {
  }

  // Public Methods
  void cleanup()
  {
    if (recv_thread.joinable())
    {
      recv_thread.join();
    }
  }

  bool enable_frequent_position_update(int frequency)
  {
    return true;
  }
  bool disable_frequent_position_update()
  {
    return true;
  }
  bool stop()
  {
    return true;
  }
  bool fast_stop()
  {
    return true;
  }
  bool reference_pan()
  {
    return true;
  }
  bool reference_tilt()
  {
    return true;
  }
  bool reset_stop()
  {
    return true;
  }
  bool set_target_velocity(double pan_velocity_radians, double tilt_velocity_radians)
  {
    return true;
  }
  bool set_target_position(double pan_radians, double tilt_radians)
  {
    return true;
  }
  bool move_position(double pan_radians, double tilt_radians, double pan_velocity_radians, double tilt_velocity_radians)
  {
    return true;
  }
  bool move_velocity(double pan_velocity_radians, double tilt_velocity_radians)
  {
    return true;
  }

  static void limit_check(double pan, double tilt, void* param);
  static void position_update(double pan, double tilt, double timestamp, void* param);
  static int test();

  // Receive messages method
  void PW70CANInterface::receive_messages(int update_frequency)
  {
    double pan_value_radians = 0.0;
    double tilt_value_radians = 0.0;

    while (true)
    {
      // Get current time
      auto current_time = std::chrono::system_clock::now();
      double current_time_sec = std::chrono::duration<double>(current_time.time_since_epoch()).count();

      // Call the callbacks after both pan and tilt have been updated
      limit_check_callback(pan_value_radians, tilt_value_radians, callbackParam);

      if (position_callback)
      {
        position_callback(pan_value_radians, tilt_value_radians, current_time_sec, callbackParam);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds{1000 / update_frequency});

    }
  }
private:

  std::thread recv_thread;
  std::function<void(double, double, void*)> limit_check_callback;
  std::function<void(double, double, double, void*)> position_callback;
  void* callbackParam;
};

}   // namespace

#endif
