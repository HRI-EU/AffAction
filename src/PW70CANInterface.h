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

#ifndef PW70CANINTERFACE_H
#define PW70CANINTERFACE_H

#include <functional>
#include <memory>


namespace aff
{

class PW70CANInterface
{
public:

  virtual ~PW70CANInterface() = default;

  static std::unique_ptr<PW70CANInterface> create(std::function<void(double, double, void*)> limit_callback,
                                                  std::function<void(double, double, double, void*)> pos_callback,
                                                  void* param,
                                                  int frequency,
                                                  bool dummy_mode=false);

  // Public Methods
  virtual void cleanup() = 0;
  virtual bool enable_frequent_position_update(int frequency) = 0;
  virtual bool disable_frequent_position_update() = 0;
  virtual bool stop() = 0;
  virtual bool fast_stop() = 0;
  virtual bool reference_pan() = 0;
  virtual bool reference_tilt() = 0;
  virtual bool reset_stop() = 0;
  virtual bool set_target_velocity(double pan_velocity_radians, double tilt_velocity_radians) = 0;
  virtual bool set_target_position(double pan_radians, double tilt_radians) = 0;
  virtual bool move_position(double pan_radians, double tilt_radians,
                             double pan_velocity_radians, double tilt_velocity_radians) = 0;
  virtual bool move_velocity(double pan_velocity_radians, double tilt_velocity_radians) = 0;

protected:

  PW70CANInterface(std::function<void(double, double, void*)> limit_callback,
                   std::function<void(double, double, double, void*)> pos_callback,
                   void* param, int frequency);

  std::function<void(double, double, void*)> limit_check_callback;
  std::function<void(double, double, double, void*)> position_callback;
  void* callbackParam;
};

}   // namespace

#endif
