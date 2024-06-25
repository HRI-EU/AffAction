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

#include "TTSComponent.h"

#include <Rcs_macros.h>
#include <Rcs_timer.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>

#if defined (_MSC_VER)
#include <sapi.h>
#endif

static const std::string piperPath = std::string(AFFACTION_PIPER_PATH);


namespace aff
{

TTSComponent::TTSComponent(EntityBase* parent, std::string whichTTS_) :
  ComponentBase(parent), threadRunning(false), whichTTS(whichTTS_)
{
  subscribe("Start", &TTSComponent::onStart);
  subscribe("Stop", &TTSComponent::onStop);
  subscribe<std::string>("Speak", &TTSComponent::onSpeak);
  subscribe<>("EmergencyStop", &TTSComponent::onEmergencyStop);
  setPiperVoice("kathleen");
}

TTSComponent::~TTSComponent()
{
}

void TTSComponent::onStart()
{
  if (this->threadRunning == true)
  {
    RLOG(1, "TTS thread already running - doing nothing");
    return;
  }

  this->threadRunning = true;
  ttsThread = std::thread(&TTSComponent::localThread, this);
}

void TTSComponent::onStop()
{
  if (this->threadRunning == false)
  {
    RLOG(1, "TTS thread not running - doing nothing");
    return;
  }

  this->threadRunning = false;
  ttsThread.join();
}

void TTSComponent::onSpeak(std::string text)
{
  RLOG(0, "Saying: %s", text.c_str());

  std::lock_guard<std::mutex> lock(mtx);
  this->textToSpeak = text;
}

std::string TTSComponent::getAndClearText()
{
  std::lock_guard<std::mutex> lock(mtx);
  std::string text = this->textToSpeak;
  this->textToSpeak.clear();
  return text;
}

void TTSComponent::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  onSpeak("Oh no, emergency stop detected");
}

void TTSComponent::setPiperVoice(const std::string& voice)
{
  if (voice == "joe")
  {
    onnxStr = std::string("\"") + piperPath + "/en_US-joe-medium.onnx\"";
    jsonStr = std::string("\"") + piperPath + "/en_en_US_joe_medium_en_US-joe-medium.onnx.json\"";
  }
  else if (voice == "alan")
  {
    onnxStr = std::string("\"") + piperPath + "/en_GB-alan-medium.onnx\"";
    jsonStr = std::string("\"") + piperPath + "/en_en_GB_alan_medium_en_GB-alan-medium.onnx.json\"";
  }
  else if (voice == "kathleen")
  {
    onnxStr = std::string("\"") + piperPath + "/en_US-kathleen-low.onnx\"";
    jsonStr = std::string("\"") + piperPath + "/en_en_US_kathleen_low_en_US-kathleen-low.onnx.json\"";
  }
  else
  {
    RMSG_CPP("Unsupported voice: '" << voice << "'");
  }
}

// If port is set to -1, this thread runs espeak on the local machine
#if defined (_MSC_VER)

void TTSComponent::localThread()
{
  // Initialize COM
  if (FAILED(::CoInitialize(NULL)))
  {
    RLOG_CPP(0, "Failed to initialize COM object");
    return;
  }

  // Create a SAPI voice object
  ISpVoice* pVoice = NULL;
  HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void**)&pVoice);

  if (!SUCCEEDED(hr))
  {
    RLOG_CPP(0, "Failed to create SAPI voice object : 0x % x" << hr);
    return;
  }

  while (this->threadRunning)
  {
    Timer_usleep(100000);   // 10Hz

    std::string text = getAndClearText();

    if (text.empty())
    {
      continue;
    }

    if (whichTTS == "piper")
    {
      std::string piperExe = std::string("\"") + piperPath + "/piper.exe" + std::string("\"");
      std::string consCmd = "echo " + std::string("\"") + text + std::string("\" | ");
      consCmd += piperExe + " -m " + onnxStr + " -c " + jsonStr + " -f \"piper.wav\" >NUL 2>&1 && ";
      consCmd += "powershell -c (New-Object Media.SoundPlayer 'piper.wav').PlaySync() >NUL 2>&1";

      int err = system(consCmd.c_str());

      if (err == -1)
      {
        RMSG_CPP("Couldn't call " << whichTTS << ": " << consCmd);
      }
      else
      {
        RLOG(1, "TTS success: Console command \"%s\"", consCmd.c_str());
      }

    }
    else
    {
      // The text to be converted to speech
      std::wstring widestr = std::wstring(text.begin(), text.end());
      const wchar_t* textToSpeak = widestr.c_str();

      // Speak the text
      pVoice->Speak(textToSpeak, 0, NULL);
      pVoice->WaitUntilDone(INFINITE);
    }

  }

  // Release the voice object
  pVoice->Release();

  // Uninitialize COM
  CoUninitialize();
}
#else

void TTSComponent::localThread()
{

  while (this->threadRunning)
  {
    Timer_usleep(100000);   // 10Hz

    std::string text = getAndClearText();
    std::string consCmd;

    if (text.empty())
    {
      continue;
    }

    if (whichTTS=="native")
    {
      consCmd = "espeak " + std::string("\"") + text + std::string("\"");
    }
    if (whichTTS=="spd-say")
    {
      consCmd = "spd-say " + std::string("\"") + text + std::string("\"");
    }
    else if (whichTTS=="piper")
    {
      // Piper command line:
      // echo "Hello, this is a test" | ./piper  -m en_US-joe-medium.onnx
      // -c en_en_US_john_medium_en_US-john-medium.onnx.json -f test1.wav; aplay test1.wav
      std::string piperExe = piperPath + "/piper";
      consCmd = "echo " + std::string("\"") + text + std::string("\" | ");
      consCmd += piperExe + " -m " + onnxStr + " -c " + jsonStr +
                 " -f piper.wav > piper.txt 2>&1; aplay piper.wav > aplay.txt 2>&1";
    }

    int err = system(consCmd.c_str());

    if (err == -1)
    {
      RMSG_CPP("Couldn't call " << whichTTS << ": " << consCmd);
    }
    else
    {
      RLOG(1, "TTS success: Console command \"%s\"", consCmd.c_str());
    }

  }

}
#endif

}   // namespace aff
