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

// https://wiki.seeedstudio.com/ReSpeaker-USB-Mic-Array/#install-dfu-and-led-control-driver
// https://github.com/mcuee/libusb-win32/releases

#ifndef AFF_RESPEAKERSOUNDDIRCOMPONENT_HPP
#define AFF_RESPEAKERSOUNDDIRCOMPONENT_HPP

#include "ComponentBase.h"

#include <Rcs_macros.h>

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

// ------------------- Linux Implementation -------------------

//#if !defined(_WIN32)

#include <libusb-1.0/libusb.h>

class RespeakerInterfaceLinux : public RespeakerInterfaceBase
{
public:
  RespeakerInterfaceLinux()
  {
    RLOG(5, "RespeakerInterfaceLinux(): libusb_init");
    if (libusb_init(&ctx) < 0)
    {
      throw std::runtime_error("Failed to init libusb");
    }

    // libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_DEBUG);
    // libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);
    // enumerate_usb_devices();

    RLOG(5, "libusb_open_device_with_vid_pid");
    dev_handle = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
    if (!dev_handle)
    {
      RLOG(0, "libusb_exit");
      libusb_exit(ctx);
      throw std::runtime_error("Failed to find Respeaker device");
    }
    // Reset device
    RLOG(5, "libusb_reset_device");
    int r = libusb_reset_device(dev_handle);
    if (r != 0)
    {
      // Not all devices support reset
      std::cerr << "Warning: Could not reset device: " << r << "\n";
    }

    // Give device some time to reinitialize
    //std::this_thread::sleep_for(std::chrono::seconds(10));
    RLOG(5, "done");
  }

  ~RespeakerInterfaceLinux() override
  {
    if (dev_handle)
    {
      libusb_close(dev_handle);
    }
    if (ctx)
    {
      libusb_exit(ctx);
    }
  }

  void enumerate_usb_devices()
  {
    libusb_device** list;
    ssize_t cnt = libusb_get_device_list(ctx, &list);
    for (ssize_t i = 0; i < cnt; ++i)
    {
      libusb_device* device = list[i];
      libusb_device_descriptor desc;
      if (libusb_get_device_descriptor(device, &desc) == 0)
      {
        std::cout << "Vendor ID: " << std::hex << desc.idVendor
                  << ", Product ID: " << desc.idProduct << std::endl;

        //if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID)
        //{
        //  std::cout << "ReSpeaker device found. Attempting to open..." << std::endl;
        //  int r = libusb_open(device, &dev_handle);
        //  if (r == 0)
        //  {
        //    std::cout << "Successfully opened ReSpeaker device!" << std::endl;
        //    break;
        //  }
        //  else
        //  {
        //    std::cerr << "Failed to open ReSpeaker device. Error: " << r << std::endl;
        //  }
        //}
      }
    }

    libusb_free_device_list(list, 1);
  }

  int angle_in_degrees() override
  {
    // VOICEACTIVITY parameter: id = 19, offset = 32, type = int
    int isVoice = readParam(19, 32, true);

    // SPEECHACTIVITY parameter: id = 19, offset = 32, type = int
    int isSpeech = readParam(19, 22, true);
    RLOG(0, "isVoice = %d   isSpeech = %d", isVoice, isSpeech);

    return readParam(21, 0, true); // DOAANGLE is int
  }

  unsigned char version() override
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

//#endif // !_WIN32


// ------------------- Windows Implementation -------------------
#if defined(_WIN32)
#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>
// #pragma comment(lib, "hid.lib")
// #pragma comment(lib, "setupapi.lib")

class RespeakerInterfaceWin : public RespeakerInterfaceBase
{
public:
  RespeakerInterfaceWin()
  {
    RLOG(0, "RespeakerInterfaceWin(): Starting initialization using Windows HID API");

    GUID HidGuid;
    HidD_GetHidGuid(&HidGuid);
    RLOG(0, "RespeakerInterfaceWin(): Retrieved HID GUID");

    HDEVINFO deviceInfoSet = SetupDiGetClassDevs(&HidGuid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (deviceInfoSet == INVALID_HANDLE_VALUE)
    {
      RLOG(0, "RespeakerInterfaceWin(): Failed to get device info set");
      throw std::runtime_error("Failed to get HID device info set");
    }
    RLOG(0, "RespeakerInterfaceWin(): Device info set retrieved");

    SP_DEVICE_INTERFACE_DATA deviceInterfaceData;
    deviceInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    bool deviceFound = false;

    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(deviceInfoSet, nullptr, &HidGuid, i, &deviceInterfaceData); ++i)
    {
      RLOG(0, "RespeakerInterfaceWin(): Enumerating device interface #%d", i);

      DWORD requiredSize = 0;
      SetupDiGetDeviceInterfaceDetail(deviceInfoSet, &deviceInterfaceData, nullptr, 0, &requiredSize, nullptr);

      auto detailData = static_cast<SP_DEVICE_INTERFACE_DETAIL_DATA*>(malloc(requiredSize));
      if (!detailData)
      {
        RLOG(0, "RespeakerInterfaceWin(): Failed to allocate memory for device interface detail");
        continue;
      }
      detailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

      if (SetupDiGetDeviceInterfaceDetail(deviceInfoSet, &deviceInterfaceData, detailData, requiredSize, nullptr, nullptr))
      {
        RLOG(0, "RespeakerInterfaceWin(): Device path: %s", detailData->DevicePath);

        HANDLE handle = CreateFile(
                          detailData->DevicePath,
                          GENERIC_READ,// | GENERIC_WRITE,       // Requested access
                          FILE_SHARE_READ,// | FILE_SHARE_WRITE, // Allow other processes to access the device
                          nullptr,
                          OPEN_EXISTING,
                          FILE_ATTRIBUTE_NORMAL,
                          nullptr);


        if (handle == INVALID_HANDLE_VALUE)
        {
          DWORD errorCode = GetLastError();
          RLOG(0, "RespeakerInterfaceWin(): Failed to open device: %lu", errorCode);
          free(detailData);
          continue;
        }

        // Get device attributes
        HIDD_ATTRIBUTES attrib;
        attrib.Size = sizeof(HIDD_ATTRIBUTES);
        if (HidD_GetAttributes(handle, &attrib))
        {
          RLOG(0, "RespeakerInterfaceWin(): Device attributes - VID: 0x%04X, PID: 0x%04X",
               attrib.VendorID, attrib.ProductID);
        }
        else
        {
          RLOG(0, "RespeakerInterfaceWin(): Failed to get device attributes");
          CloseHandle(handle);
          free(detailData);
          continue;
        }

        // Retrieve manufacturer, product, and serial strings
        wchar_t manufacturer[128];
        wchar_t product[128];
        wchar_t serial[128];

        if (HidD_GetManufacturerString(handle, manufacturer, sizeof(manufacturer)))
        {
          RLOG(0, "RespeakerInterfaceWin(): Manufacturer: %ls", manufacturer);
        }
        else
        {
          RLOG(0, "RespeakerInterfaceWin(): Failed to get manufacturer string");
        }

        if (HidD_GetProductString(handle, product, sizeof(product)))
        {
          RLOG(0, "RespeakerInterfaceWin(): Product: %ls", product);
        }
        else
        {
          RLOG(0, "RespeakerInterfaceWin(): Failed to get product string");
        }

        if (HidD_GetSerialNumberString(handle, serial, sizeof(serial)))
        {
          RLOG(0, "RespeakerInterfaceWin(): Serial Number: %ls", serial);
        }
        else
        {
          RLOG(0, "RespeakerInterfaceWin(): Failed to get serial number string");
        }

        // Check if this is the desired device based on additional information
        if (attrib.VendorID == VENDOR_ID && attrib.ProductID == PRODUCT_ID)
        {
          deviceHandle = handle;
          deviceFound = true;
          RLOG(0, "RespeakerInterfaceWin(): Found and opened ReSpeaker HID device");
          free(detailData);
          break;
        }

        CloseHandle(handle);
        free(detailData);
      }
      else
      {
        RLOG(0, "RespeakerInterfaceWin(): Failed to get device interface detail for device #%d", i);
        free(detailData);
      }
    }

    SetupDiDestroyDeviceInfoList(deviceInfoSet);

    if (!deviceFound || deviceHandle == INVALID_HANDLE_VALUE)
    {
      RLOG(0, "RespeakerInterfaceWin(): Failed to find or open ReSpeaker HID device");
      throw std::runtime_error("Failed to find and open Respeaker HID device");
    }

    RLOG(0, "RespeakerInterfaceWin(): Initialization completed successfully");
  }

  ~RespeakerInterfaceWin() override
  {
    if (deviceHandle != INVALID_HANDLE_VALUE)
    {
      RLOG(0, "RespeakerInterfaceWin(): Closing device handle");
      CloseHandle(deviceHandle);
    }
  }

  int angle_in_degrees() override
  {
    // Implementation remains the same
    return readParam(21, 0, true);
  }

  unsigned char version() override
  {
    // Implementation remains the same
    return 0; // Placeholder for your actual implementation
  }

private:
  HANDLE deviceHandle = INVALID_HANDLE_VALUE;
  static constexpr uint16_t VENDOR_ID = 0x2886;
  static constexpr uint16_t PRODUCT_ID = 0x0018;
  static constexpr unsigned int TIMEOUT = 100000;
  static constexpr unsigned char FEATURE_REPORT_ID = 0x01;

  int readParam(uint16_t index, uint16_t value, bool isInt)
  {
    // Implementation remains the same
    return 0; // Placeholder for your actual implementation
  }
};


#endif // _WIN32


} // namespace aff

#endif   // AFF_RESPEAKERSOUNDDIRCOMPONENT_HPP
