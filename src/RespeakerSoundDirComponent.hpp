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

#ifndef AFF_RESPEAKERSOUNDDIRCOMPONENT_H
#define AFF_RESPEAKERSOUNDDIRCOMPONENT_H

#include "ComponentBase.h"

#include <Rcs_macros.h>

#include <libusb-1.0/libusb.h>

#include <iostream>
#include <thread>
#include <queue>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>



namespace aff
{


// ------------------- Respeaker Interface (USB) -------------------
class RespeakerInterface
{
public:
  RespeakerInterface()
  {
    if (libusb_init(&ctx) < 0)
    {
      throw std::runtime_error("Failed to init libusb");
    }
    dev_handle = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
    if (!dev_handle)
    {
      libusb_exit(ctx);
      throw std::runtime_error("Failed to find Respeaker device");
    }
    // Reset device
    int r = libusb_reset_device(dev_handle);
    if (r != 0)
    {
      // Not all devices support reset
      std::cerr << "Warning: Could not reset device: " << r << "\n";
    }

    // Give device some time to reinitialize
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }

  ~RespeakerInterface()
  {
    if (dev_handle)
    {
      libusb_close(dev_handle);
    }
    libusb_exit(ctx);
  }

  int angle_in_degrees()
  {
    return readParam(21, 0, true); // DOAANGLE is int
  }

  unsigned char version()
  {
    unsigned char data;
    // This mimics the python code: device.ctrl_transfer(...) = libusb_control_transfer
    // The python code: ctrl_transfer(CTRL_IN, 0, 0x80, 0, length=1)
    // That corresponds to a standard vendor request.
    int transferred = libusb_control_transfer(
                        dev_handle,
                        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                        0,
                        0x80,
                        0,
                        &data,
                        1,
                        TIMEOUT
                      );
    if (transferred < 1)
    {
      std::cerr << "Failed to read version\n";
      return 0;
    }
    return data;
  }

private:

  int readParam(uint16_t index, uint16_t value, bool isInt)
  {
    // The Python code sets cmd = 0x80 | offset (value) and if int 0x40 more.
    // DOAANGLE: offset=0, param block=21 (index)
    // For int parameters:
    // response = struct.unpack(b"ii", response)
    // The device returns 8 bytes: two int32
    // For int: result = response[0]

    uint16_t cmd = 0x80 | value;
    if (isInt)
    {
      cmd |= 0x40;
    }

    unsigned char buf[8] = {0};
    int transferred = libusb_control_transfer(
                        dev_handle,
                        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                        0,
                        cmd,
                        index,
                        buf,
                        sizeof(buf),
                        TIMEOUT
                      );

    if (transferred < (int)sizeof(buf))
    {
      std::cerr << "Failed to read parameter\n";
      return -1;
    }

    int* ints = (int*)buf;
    // ints[0], ints[1]
    // For DOAANGLE it's an int parameter
    return ints[0];
  }

  libusb_context* ctx = nullptr;
  libusb_device_handle* dev_handle = nullptr;
  static constexpr uint16_t VENDOR_ID = 0x2886;
  static constexpr uint16_t PRODUCT_ID = 0x0018;
  static constexpr unsigned int TIMEOUT = 100000;
};


class RespeakerUSBComponent : public RespeakerInterface, public ComponentBase
{
public:
  RespeakerUSBComponent(EntityBase* parent) : RespeakerInterface(), ComponentBase(parent),
    threadRunning(false)
  {
    subscribe("Start", &RespeakerUSBComponent::startUSBThread);
    subscribe("Stop", &RespeakerUSBComponent::stopUSBThread);
  }

private:

  void startUSBThread()
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

  void stopUSBThread()
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

  void usbThreadFunc()
  {
    // Initialize Respeaker device
    RespeakerInterface respeaker;
    std::cout << "Respeaker version: " << (int)respeaker.version() << "\n";

    while (threadRunning)
    {
      int angle = respeaker.angle_in_degrees();
      RLOG(0, "angle = %d", angle);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  bool threadRunning;
  std::thread usbThread;
};

}   // namespace

#endif   // AFF_RESPEAKERSOUNDDIRCOMPONENT_H
