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

#ifndef RCS_TTSCOMPONENT_H
#define RCS_TTSCOMPONENT_H

#include "ComponentBase.h"

#include <thread>


namespace aff
{

class TTSComponent : public ComponentBase
{
public:

  TTSComponent(EntityBase* parent, std::string whichTTS="native");
  virtual ~TTSComponent();
  void setPiperVoice(const std::string& voice); // kathleen (default), alan, joe

private:

  void onStart();
  void onStop();
  void onSpeak(std::string text);
  std::string getAndClearText();
  void onEmergencyStop();
  void localThread();

  bool threadRunning;
  std::string textToSpeak;
  std::string whichTTS;
  std::string onnxStr, jsonStr;
  std::mutex mtx;
  std::thread ttsThread;

  // Avoid copying this class
  TTSComponent(const TTSComponent&);
  TTSComponent& operator=(const TTSComponent&);
};

}

#endif   // RCS_TTSCOMPONENT_H
