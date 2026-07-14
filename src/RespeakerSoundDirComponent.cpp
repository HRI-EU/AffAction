/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

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

#include "RespeakerSoundDirComponent.h"

#if defined (AFFACTION_WITH_RESPEAKER)
#include "RespeakerSoundDirComponent.hpp"
#endif

#include <Rcs_macros.h>


namespace aff
{

#if defined (AFFACTION_WITH_RESPEAKER)
void RespeakerUSBComponent::usbThreadFunc()
{
  auto rinterface = std::make_unique<RespeakerInterface>();

  // Give device some time to reinitialize
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::cout << "Respeaker version: " << (int)rinterface->version() << "\n";

  while (threadRunning)
  {
    int isVoice, isSpeech, angle;
    rinterface->angle_in_degrees(&isVoice, &isSpeech, &angle);
    RLOG(0, "%d %d %d angle = %d", isVoice, isSpeech, isVoice+isSpeech, angle);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}
#else
void RespeakerUSBComponent::usbThreadFunc()
{
}
#endif

RespeakerUSBComponent::RespeakerUSBComponent(EntityBase* parent) : ComponentBase(parent), threadRunning(false)
{
  subscribe("Start", &RespeakerUSBComponent::startUSBThread);
  subscribe("Stop", &RespeakerUSBComponent::stopUSBThread);
}

RespeakerUSBComponent::~RespeakerUSBComponent()
{
  stopUSBThread();
}

void RespeakerUSBComponent::startUSBThread()
{
  if (threadRunning)
  {
    RLOG(1, "Thread already running");
    return;
  }

  RLOG(0, "startUSBThread()");

  threadRunning = true;
  usbThread = std::thread(&RespeakerUSBComponent::usbThreadFunc, this);
}

void RespeakerUSBComponent::stopUSBThread()
{
  if (!threadRunning)
  {
    RLOG(0, "Thread already stopped");
    return;
  }

  RLOG(0, "Trying to stop thread");
  threadRunning = false;

  if (usbThread.joinable())
  {
    usbThread.join();
  }
  RLOG(0, "Thread joined, stopUSBThread completed");
}

} // namespace aff
