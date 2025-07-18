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
#include <Rcs_utils.h>
#include <Rcs_utilsCPP.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>

#if defined (_MSC_VER)
#include <sapi.h>
#include <sphelper.h> // for SpEnumTokens and SpGetDescription
#endif

static std::string piperPath = std::string(AFFACTION_PIPER_PATH);
#if defined (_MSC_VER)
static std::string piperExe = std::string("\"") + piperPath + "/piper.exe" + std::string("\"");
//static std::string piperExe = piperPath + "\\piper.exe";
#else
static std::string piperExe = piperPath + "/piper";
#endif


namespace aff
{

TTSComponent::TTSComponent(EntityBase* parent, std::string whichTTS_) :
  ComponentBase(parent), threadRunning(false), audioRate(16000), whichTTS(whichTTS_)
{
  subscribe("Start", &TTSComponent::onStart);
  subscribe("Stop", &TTSComponent::onStop);
  subscribe<std::string>("Speak", &TTSComponent::onSpeak);
  subscribe<>("EmergencyStop", &TTSComponent::onEmergencyStop);
  setPiperVoice("ryan");

  // Check for existing of the piper executable. If it does not exits,
  // we fall back to the basic default.
  std::string checkExePath = piperExe;
  Rcs::String_trim(checkExePath, "\"");
  if ((whichTTS=="piper") && (!File_exists(checkExePath.c_str())))
  {
    RLOG_CPP(0, "Piper TTS not found (" + checkExePath + ") - falling back to native TTS");
    whichTTS = "native";
  }

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

void TTSComponent::setPiperPath(const std::string& path)
{
  piperPath = path;
#if defined (_MSC_VER)
  piperExe = std::string("\"") + piperPath + "/piper.exe" + std::string("\"");
#else
  piperExe = piperPath + "/piper";
#endif
  RLOG_CPP(0, "Setting piper path to '" << piperPath << "'");
}

void TTSComponent::setPiperVoice(const std::string& voice)
{
  if (voice == "joe")
  {
    onnxStr = "en_US-joe-medium.onnx\"";
    jsonStr = "en_en_US_joe_medium_en_US-joe-medium.onnx.json\"";
    audioRate = 22050;
  }
  else if (voice == "alan")
  {
    onnxStr = "en_GB-alan-medium.onnx\"";
    jsonStr = "en_en_GB_alan_medium_en_GB-alan-medium.onnx.json\"";
    audioRate = 22050;
  }
  else if (voice == "kathleen")
  {
    onnxStr = "en_US-kathleen-low.onnx\"";
    jsonStr = "en_en_US_kathleen_low_en_US-kathleen-low.onnx.json\"";
    audioRate = 16000;
  }
  else if (voice == "ryan")
  {
    onnxStr = "en_US-ryan-low.onnx\"";
    jsonStr = "en_en_US_ryan_low_en_US-ryan-low.onnx.json\"";
    audioRate = 16000;
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

#if 0
  if (SUCCEEDED(hr))
  {
    // Enumerate the available voices
    IEnumSpObjectTokens* cpEnum = NULL;
    hr = SpEnumTokens(SPCAT_VOICES, NULL, NULL, &cpEnum);
    if (SUCCEEDED(hr))
    {
      ULONG ulCount = 0;
      hr = cpEnum->GetCount(&ulCount);
      if (SUCCEEDED(hr) && ulCount > 0)
      {
        // Loop through all voices and print their names
        for (ULONG i = 0; i < ulCount; i++)
        {
          ISpObjectToken* cpVoiceToken = NULL;
          hr = cpEnum->Item(i, &cpVoiceToken);
          if (SUCCEEDED(hr) && cpVoiceToken)
          {
            // Get voice description (name)
            WCHAR* pszDesc = NULL;
            SpGetDescription(cpVoiceToken, &pszDesc);
            if (pszDesc)
            {
              wprintf(L"Voice %lu: %s\n", i, pszDesc);
              ::CoTaskMemFree(pszDesc);
            }

            // If this is the voice you want, call SetVoice.
            // For example, if we pick the first voice:
            // if (i == 1) { // choose the second listed voice
            //     pVoice->SetVoice(cpVoiceToken);
            // }

            cpVoiceToken->Release();
          }
        }

        // As an example, let's just pick the first voice we enumerated
        cpEnum->Reset();
        ISpObjectToken* cpChosenToken = NULL;
        hr = cpEnum->Item(0, &cpChosenToken); // get the first voice token
        if (SUCCEEDED(hr) && cpChosenToken)
        {
          // Now set the voice to the chosen token
          hr = pVoice->SetVoice(cpChosenToken);
          cpChosenToken->Release();
        }
      }
      cpEnum->Release();
    }

    // Speak something using the chosen voice
    if (SUCCEEDED(hr))
    {
      pVoice->Speak(L"Hello, this is the selected Microsoft voice.", SPF_ASYNC, NULL);
      pVoice->WaitUntilDone(INFINITE);
    }

    // Release the voice
    pVoice->Release();
    pVoice = NULL;
  }
#endif






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
      double t_synthesize = Timer_getSystemTime();
      std::string onnxPath = std::string("\"") + piperPath + "/" + onnxStr;
      std::string jsonPath = std::string("\"") + piperPath + "/" + jsonStr;
      std::string consCmd = "echo " + std::string("\"") + text + std::string("\" | ");
      //consCmd += piperExe + " -m " + onnxPath + " -c " + jsonPath + " -f \"piper.wav\" >NUL 2>&1 && ";
      //consCmd += "powershell -c (New-Object Media.SoundPlayer 'piper.wav').PlaySync() >NUL 2>&1";

      //consCmd += piperExe + " -m " + onnxPath + " -c " + jsonPath +
      //           //" --output-raw  > NUL 2>&1"
      //           " --output-raw 2> NUL "
      //           "| \"C:/Users/RE900373/Documents/Software/Programs/ffmpeg-7.1-essentials_build/bin/ffplay\""
      //           " -f s16le -ar " + std::to_string(audioRate) + " -nodisp -autoexit -i pipe:0 > NUL 2>&1"
      //           ;

      // The command ffplay -f s16le -ar 22050 -nodisp -autoexit -i pipe:0
      // tells ffplay to read 16-bit little-endian raw audio samples at
      // a 22,050 Hz sample rate from standard input, play them back without
      // showing any windows, and close automatically when done.
      consCmd += piperExe + " -m " + onnxPath + " -c " + jsonPath + " --output-raw 2> NUL | ffplay.exe"
                 " -f s16le -ar " + std::to_string(audioRate) + " -nodisp -autoexit -i pipe:0 > NUL 2>&1";

      int err = system(consCmd.c_str());


      if (err == -1)
      {
        RMSG_CPP("Couldn't call " << whichTTS << ": " << consCmd);
      }
      else
      {
        t_synthesize = Timer_getSystemTime() - t_synthesize;
        RLOG(1, "TTS success: Console command \"%s\" took %.3f sec", consCmd.c_str(), t_synthesize);
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
#if defined (__APPLE__)
      consCmd = "say -v Reed " + std::string("\"") + text + std::string("\"");
#else
      consCmd = "espeak " + std::string("\"") + text + std::string("\"");
#endif
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
      std::string onnxPath = std::string("\"") + piperPath + "/" + onnxStr;
      std::string jsonPath = std::string("\"") + piperPath + "/" + jsonStr;
      consCmd = "echo " + std::string("\"") + text + std::string("\" | ");
      consCmd += piperExe + " -m " + onnxPath + " -c " + jsonPath +
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
