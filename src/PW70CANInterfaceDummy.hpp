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
#include <mutex>
#include <iostream>

namespace aff
{

class PW70CANInterface
{
public:
  // Constructor and Destructor
  PW70CANInterface(std::function<void(double, double, void*)> limit_check_callback,
                   std::function<void(double, double, double, void*)> position_callback,
                   void* param, int freq) {}
  ~PW70CANInterface() {}

  // Public Methods
  void cleanup() {}
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
};

}   // namespace

#endif
