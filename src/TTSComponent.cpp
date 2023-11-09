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
#include "EntityBase.h"

#include <Rcs_macros.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#if !defined (_MSC_VER)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#else
#pragma comment(lib, "Ws2_32.lib")
#include <WinSock2.h>
#include "ttspeak.h"
#endif


namespace aff
{

TTSComponent::TTSComponent(EntityBase* parent, int port_) :
  ComponentBase(parent),
  port(port_), threadRunning(false)
{
#if defined (_MSC_VER)
  port = -1;
#endif
  subscribe("Start", &TTSComponent::onStart);
  subscribe("Stop", &TTSComponent::onStop);
  subscribe<std::string>("Speak", &TTSComponent::onSpeak);
  subscribe<>("EmergencyStop", &TTSComponent::onEmergencyStop);
}

TTSComponent::~TTSComponent()
{
}

void TTSComponent::setPort(int port_)
{
  this->port = port_;
}

void TTSComponent::onStart()
{
  if (this->threadRunning == true)
  {
    RLOG(1, "TTS thread already running - doing nothing");
    return;
  }

  this->threadRunning = true;

  if (port==-1)
  {
    ttsThread = std::thread(&TTSComponent::localThread, this);
  }
  else
  {
    ttsThread = std::thread(&TTSComponent::listenerThread, this);
  }
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

  mtx.lock();
  this->textToSpeak = text;
  mtx.unlock();
}

void TTSComponent::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  onSpeak("Oh no, emergency stop detected");
}

void TTSComponent::listenerThread()
{
#if defined (_MSC_VER)
  SOCKET sockfd;
#else
  int sockfd;
#endif

  struct sockaddr_in serveraddr;

  // Creating socket file descriptor
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("socket creation failed");
    RFATAL("socket creation failed");
  }


  /* gethostbyname: get the server's DNS entry */
  const char* hostname = "dexcoop-03";
  struct hostent* server = gethostbyname(hostname);
  if (server == NULL)
  {
    fprintf(stderr,"ERROR, no such host as %s\n", hostname);
    RFATAL("gethostbyname failed");
  }

  memset(&serveraddr, 0, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  memcpy((char*)&serveraddr.sin_addr.s_addr, (char*)server->h_addr,
         server->h_length);
  //bcopy((char*)server->h_addr,
  //  (char*)&serveraddr.sin_addr.s_addr, server->h_length);
  serveraddr.sin_port = htons(port);


  while (this->threadRunning)
  {
    mtx.lock();
    std::string text = this->textToSpeak;
    this->textToSpeak.clear();
    mtx.unlock();

    if (text.length() > 0)
    {
      sendto(sockfd, (const char*) text.c_str(), text.length(),
             0, (const struct sockaddr*) &serveraddr, sizeof(serveraddr));
      RLOG(0, "Speaking: \"%s\"", text.c_str());
    }
  }

#if defined (_MSC_VER)
  closesocket(sockfd);
#else
  close(sockfd);
#endif
}

// If port is set to -1, this thread runs espeak on the local machine
void TTSComponent::localThread()
{

  while (this->threadRunning)
  {
    mtx.lock();
    std::string text = this->textToSpeak;
    this->textToSpeak.clear();
    mtx.unlock();

    if (text.length() > 0)
    {
#if !defined (_MSC_VER)

      std::string consCmd = "spd-say " + std::string("\"") + text + std::string("\"");
      int err = system(consCmd.c_str());

      if (err == -1)
      {
        RMSG("Couldn't call spd-say");
      }
      else
      {
        RLOG(0, "--- Saying \"%s\"", text.c_str());
        RLOG(0, "Console command \"%s\"", consCmd.c_str());
      }

#else
      // Class constructor called with object speechOut
      ttspeak speechOut;

      // Loads the ComSpeak function with max array size of 256 (was 100)
      // and gender enum of type FEMALE
      speechOut.loadComSpeak(256, MALE);

      // String to speech synthesis. It returns after the text has been spoken.
      speechOut.comSpeak(text);
#endif
    }
  }

}

}   // namespace aff
