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

#include "PW70CANInterface.h"

#if defined (_MSC_VER) && defined (AFFACTION_WITH_PCAN_BASIC)
#include "PW70CANInterfaceWin.hpp"
#elif defined(__linux__) && !defined(__APPLE__)
#include "PW70CANInterfaceLinux.hpp"
#endif

#include "PW70CANInterfaceDummy.hpp"
#include <Rcs_macros.h>


namespace aff
{

PW70CANInterface::PW70CANInterface(std::function<void(double, double, void*)> limit_check_callback_,
                                   std::function<void(double, double, double, void*)> position_callback_,
                                   void* param,
                                   int frequency) :
  limit_check_callback(limit_check_callback_),
  position_callback(position_callback_),
  callbackParam(param)
{
}

std::unique_ptr<PW70CANInterface> PW70CANInterface::create(std::function<void(double, double, void*)> limit_check_callback,
                                                           std::function<void(double, double, double, void*)> position_callback,
                                                           void* param,
                                                           int frequency,
                                                           const std::string& can_id)
{
  std::unique_ptr<PW70CANInterface> pw70;

  if (can_id.empty())
  {
    RLOG(0, "Creating PW70CANInterfaceDummy");
    return std::make_unique<PW70CANInterfaceDummy>(limit_check_callback, position_callback, param, frequency);
  }

#if defined (_MSC_VER) && defined (AFFACTION_WITH_PCAN_BASIC)
  RLOG(0, "Creating PW70CANInterfaceWin");
  pw70 = std::make_unique<PW70CANInterfaceWin>(limit_check_callback, position_callback, param, frequency);
#elif defined(__linux__) && !defined(__APPLE__)
  RLOG(0, "Creating PW70CANInterfaceLinuxPW70CANInterfaceLinux");
  pw70 = std::make_unique<PW70CANInterfaceLinux>(limit_check_callback, position_callback, param, frequency, can_id);
#else
  RLOG(0, "Creating PW70CANInterfaceDummy");
  pw70 = std::make_unique<PW70CANInterfaceDummy>(limit_check_callback, position_callback, param, frequency);
#endif

  return pw70;
}

std::unique_ptr<PW70CANInterface> PW70CANInterface::create(const std::string& can_id)
{
  std::unique_ptr<PW70CANInterface> pw70;

#if defined (_MSC_VER) && defined (AFFACTION_WITH_PCAN_BASIC)
  pw70 = std::make_unique<PW70CANInterfaceWin>();
#elif defined(__linux__) && !defined(__APPLE__)
  pw70 = std::make_unique<PW70CANInterfaceLinux>(can_id);
#else
  pw70 = std::make_unique<PW70CANInterfaceDummy>(nullptr, nullptr, nullptr, 0);
#endif

  return pw70;
}


}   // namespace
