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

#ifndef PW70CANINTERFACEWIN_H
#define PW70CANINTERFACEWIN_H

// Include standard headers
#include <functional>
#include <thread>
#include <mutex>
#include <vector>

// Include Windows headers
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

// Undefine conflicting macros
#ifdef ID
#undef ID
#endif

#ifdef DATA
#undef DATA
#endif

// Include PCAN-Basic API header
#include "PCANBasic.h"

namespace aff
{

class PW70CANInterface
{
public:
  // Constructor and Destructor
  PW70CANInterface(std::function<void(double, double, void*)> limit_check_callback,
                   std::function<void(double, double, double, void*)> position_callback,
                   void* param, int freq);
  ~PW70CANInterface();

  // Public Methods
  void cleanup();
  bool enable_frequent_position_update(int frequency);
  bool disable_frequent_position_update();
  bool stop();
  bool fast_stop();
  bool reference_pan();
  bool reference_tilt();
  bool reset_stop();
  bool set_target_velocity(double pan_velocity_radians, double tilt_velocity_radians);
  bool set_target_position(double pan_radians, double tilt_radians);
  bool move_position(double pan_radians, double tilt_radians, double pan_velocity_radians, double tilt_velocity_radians);
  bool move_velocity(double pan_velocity_radians, double tilt_velocity_radians);

  static void limit_check(double pan, double tilt, void* param);
  static void position_update(double pan, double tilt, double timestamp, void* param);
  static int test();

private:
  // Private Methods
  bool send(std::vector<TPCANMsg> frames);
  void receive_messages();

  // Helper Functions
  uint32_t float_to_le(float value);
  float le_to_float(uint32_t value);

  // Member Variables
  TPCANHandle m_PcanHandle;  // Handle to the PCAN device
  std::function<void(double, double, void*)> limit_check_callback;
  std::function<void(double, double, double, void*)> position_callback;
  bool running;
  std::thread recv_thread;
  std::mutex socket_mutex;
  void* callbackParam;
};

}   // namespace

#endif // PW70CANINTERFACEWIN_H



#include <iostream>
#include <map>
#include <cstdint>


namespace aff
{

// Helper function to convert float to little-endian uint32_t
uint32_t PW70CANInterface::float_to_le(float value)
{
  uint32_t int_value = *reinterpret_cast<uint32_t*>(&value);
  return _byteswap_ulong(int_value); // Swap byte order
}

// Helper function to convert little-endian uint32_t to float
float PW70CANInterface::le_to_float(uint32_t value)
{
  value = _byteswap_ulong(value); // Swap byte order
  return *reinterpret_cast<float*>(&value);
}

// Constructor
PW70CANInterface::PW70CANInterface(std::function<void(double, double, void*)> limit_check_callback,
                                   std::function<void(double, double, double, void*)> position_callback,
                                   void* param, int freq)
  : limit_check_callback(limit_check_callback), position_callback(position_callback),
    running(true), callbackParam(param)
{
  // Initialize PCAN handle (adjust this according to your hardware)
  m_PcanHandle = PCAN_USBBUS1; // Use the appropriate handle for your hardware

  // Initialize the CAN channel at 500 Kbps (adjust baud rate if needed)
  TPCANStatus status = CAN_Initialize(m_PcanHandle, PCAN_BAUD_500K);
  if (status != PCAN_ERROR_OK)
  {
    char errorMsg[256];
    CAN_GetErrorText(status, 0, errorMsg);
    std::cerr << "Error initializing PCAN: " << errorMsg << std::endl;
    exit(EXIT_FAILURE);
  }

  // Start the receive thread
  recv_thread = std::thread(&PW70CANInterface::receive_messages, this);

  // Enable regular status updates.
  enable_frequent_position_update(freq);
}

// Destructor
PW70CANInterface::~PW70CANInterface()
{
  if (running)
  {
    cleanup();
  }
}

// Cleanup method
void PW70CANInterface::cleanup()
{
  disable_frequent_position_update();

  // Stop and fast stop to disable motor current and engage brakes.
  stop();
  // fast_stop();

  // Stop the receive thread
  running = false;
  if (recv_thread.joinable())
  {
    recv_thread.join();
  }

  // Uninitialize the CAN channel
  CAN_Uninitialize(m_PcanHandle);
}

// Send method
bool PW70CANInterface::send(std::vector<TPCANMsg> frames)
{
  std::lock_guard<std::mutex> lock(socket_mutex);
  for (auto& frame : frames)
  {
    TPCANStatus status = CAN_Write(m_PcanHandle, &frame);
    if (status != PCAN_ERROR_OK)
    {
      char errorMsg[256];
      CAN_GetErrorText(status, 0, errorMsg);
      std::cerr << "Error sending CAN frame: " << errorMsg << std::endl;
      return false;
    }
  }
  return true;
}

// Receive messages method
void PW70CANInterface::receive_messages()
{
  bool pan_updated = false;
  bool tilt_updated = false;

  double pan_value_radians = 0.0f;
  double tilt_value_radians = 0.0f;

  while (running)
  {
    TPCANMsg frame;
    TPCANTimestamp timestamp;

    TPCANStatus status;
    {
      // Lock the mutex for thread-safe access
      std::lock_guard<std::mutex> lock(socket_mutex);
      status = CAN_Read(m_PcanHandle, &frame, &timestamp);
    }

    if (status == PCAN_ERROR_QRCVEMPTY)
    {
      // No message available, sleep for a short time
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    else if (status != PCAN_ERROR_OK)
    {
      char errorMsg[256];
      CAN_GetErrorText(status, 0, errorMsg);
      std::cerr << "Error reading CAN frame: " << errorMsg << std::endl;
      running = false;
      break;
    }

    // Process the message
    uint32_t id = frame.ID;

    if (id == 0x70D || id == 0x70E)
    {
      if (frame.DATA[0] == 0x07 && frame.DATA[1] == 0x95)
      {
        uint32_t angle_bytes;
        std::memcpy(&angle_bytes, &frame.DATA[2], sizeof(uint32_t));
        float angle_degrees = le_to_float(angle_bytes);

        if (id == 0x70D)
        {
          tilt_value_radians = -(M_PI/180.0)*angle_degrees;  // Tilt is flipped and in radians
          tilt_updated = true;
        }
        else if (id == 0x70E)
        {
          pan_value_radians = (M_PI/180.0)*angle_degrees;    // Pan value in radians
          pan_updated = true;
        }

        // Check if both pan and tilt have been updated
        if (pan_updated && tilt_updated)
        {
          // Get current time
          auto current_time = std::chrono::system_clock::now();
          double current_time_sec = std::chrono::duration<double>(current_time.time_since_epoch()).count();

          // Call the callbacks after both pan and tilt have been updated
          limit_check_callback(pan_value_radians, tilt_value_radians, callbackParam);

          if (position_callback)
          {
            position_callback(pan_value_radians, tilt_value_radians, current_time_sec, callbackParam);
          }

          // Reset the flags for the next update
          pan_updated = false;
          tilt_updated = false;
        }
      }
    }
  }
}

// Enable frequent position update method
bool PW70CANInterface::enable_frequent_position_update(int frequency)
{
  // Schunk Motion Protocol commands
  std::map<int, std::vector<uint8_t>> can_state_cmds =
  {
    {1, {0x06, 0x95, 0x00, 0x00, 0x80, 0x3F, 0x01}},    // 1 Hz
    {10, {0x06, 0x95, 0xCD, 0xCC, 0xCC, 0x3D, 0x01}},   // 10 Hz
    {25, {0x06, 0x95, 0x0A, 0xD7, 0x23, 0x3D, 0x01}},   // 25 Hz
    {50, {0x06, 0x95, 0x0A, 0xD7, 0xA3, 0x3C, 0x01}},   // 50 Hz
    {100, {0x06, 0x95, 0x0A, 0xD7, 0x23, 0x3C, 0x01}},  // 100 Hz
  };

  auto it = can_state_cmds.find(frequency);
  if (it == can_state_cmds.end())
  {
    std::cerr << "Cannot handle frequency " << frequency << ". Known frequencies are:";
    for (const auto& kv : can_state_cmds)
    {
      std::cerr << " " << kv.first;
    }
    std::cerr << "." << std::endl;
    return false;
  }

  const auto& can_state_cmd = it->second;

  TPCANMsg msg1;
  msg1.ID = 0x50D;
  msg1.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg1.LEN = static_cast<UINT8>(can_state_cmd.size());
  std::copy(can_state_cmd.begin(), can_state_cmd.end(), msg1.DATA);

  TPCANMsg msg2 = msg1;
  msg2.ID = 0x50E;

  return send({ msg1, msg2 });
}

// Disable frequent position update method
bool PW70CANInterface::disable_frequent_position_update()
{
  TPCANMsg msg1;
  msg1.ID = 0x50D;
  msg1.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg1.LEN = 2;
  msg1.DATA[0] = 0x01;
  msg1.DATA[1] = 0x95;

  TPCANMsg msg2 = msg1;
  msg2.ID = 0x50E;

  return send({ msg1, msg2 });
}

// Stop method
bool PW70CANInterface::stop()
{
  TPCANMsg msg1;
  msg1.ID = 0x50D;
  msg1.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg1.LEN = 2;
  msg1.DATA[0] = 0x01;
  msg1.DATA[1] = 0x91;

  TPCANMsg msg2 = msg1;
  msg2.ID = 0x50E;

  return send({ msg1, msg2 });
}

// Fast stop method
bool PW70CANInterface::fast_stop()
{
  TPCANMsg msg1;
  msg1.ID = 0x50D;
  msg1.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg1.LEN = 2;
  msg1.DATA[0] = 0x01;
  msg1.DATA[1] = 0x90;

  TPCANMsg msg2 = msg1;
  msg2.ID = 0x50E;

  return send({ msg1, msg2 });
}

// Reference pan method
bool PW70CANInterface::reference_pan()
{
  TPCANMsg msg;
  msg.ID = 0x50E;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = 2;
  msg.DATA[0] = 0x01;
  msg.DATA[1] = 0x92;

  return send({ msg });
}

// Reference tilt method
bool PW70CANInterface::reference_tilt()
{
  TPCANMsg msg;
  msg.ID = 0x50D;
  msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg.LEN = 2;
  msg.DATA[0] = 0x01;
  msg.DATA[1] = 0x92;

  return send({ msg });
}

// Reset stop method
bool PW70CANInterface::reset_stop()
{
  TPCANMsg msg1;
  msg1.ID = 0x50D;
  msg1.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg1.LEN = 2;
  msg1.DATA[0] = 0x01;
  msg1.DATA[1] = 0x8B;

  TPCANMsg msg2 = msg1;
  msg2.ID = 0x50E;

  return send({ msg1, msg2 });
}

// Set target velocity method
bool PW70CANInterface::set_target_velocity(double pan_velocity_radians, double tilt_velocity_radians)
{
  TPCANMsg msg1, msg2;

  msg1.ID = 0x50D;
  msg1.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg1.LEN = 6;
  msg1.DATA[0] = 0x05;
  msg1.DATA[1] = 0xA0;
  float tilt_velocity = static_cast<float>(-(180.0 / M_PI) * tilt_velocity_radians); // Tilt is flipped
  uint32_t tilt_velocity_le = float_to_le(tilt_velocity);
  std::memcpy(&msg1.DATA[2], &tilt_velocity_le, sizeof(uint32_t));

  msg2 = msg1;
  msg2.ID = 0x50E;
  float pan_velocity = static_cast<float>((180.0 / M_PI) * pan_velocity_radians);
  uint32_t pan_velocity_le = float_to_le(pan_velocity);
  std::memcpy(&msg2.DATA[2], &pan_velocity_le, sizeof(uint32_t));

  return send({ msg1, msg2 });
}

// Set target position method
bool PW70CANInterface::set_target_position(double pan_radians, double tilt_radians)
{
  TPCANMsg msg1, msg2;

  msg1.ID = 0x50D;
  msg1.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg1.LEN = 6;
  msg1.DATA[0] = 0x05;
  msg1.DATA[1] = 0xB0;
  float tilt_pos_degrees = static_cast<float>(-(180.0 / M_PI) * tilt_radians); // Tilt is flipped
  uint32_t tilt_pos_le = float_to_le(tilt_pos_degrees);
  std::memcpy(&msg1.DATA[2], &tilt_pos_le, sizeof(uint32_t));

  msg2 = msg1;
  msg2.ID = 0x50E;
  float pan_pos_degrees = static_cast<float>((180.0 / M_PI) * pan_radians);
  uint32_t pan_pos_le = float_to_le(pan_pos_degrees);
  std::memcpy(&msg2.DATA[2], &pan_pos_le, sizeof(uint32_t));

  return send({ msg1, msg2 });
}

// Move position method
bool PW70CANInterface::move_position(double pan_radians, double tilt_radians, double pan_velocity_radians, double tilt_velocity_radians)
{
  if (!set_target_velocity(pan_velocity_radians, tilt_velocity_radians))
  {
    return false;
  }
  return set_target_position(pan_radians, tilt_radians);
}

// Move velocity method
bool PW70CANInterface::move_velocity(double pan_velocity_radians, double tilt_velocity_radians)
{
  TPCANMsg msg1, msg2;

  msg1.ID = 0x50D;
  msg1.MSGTYPE = PCAN_MESSAGE_STANDARD;
  msg1.LEN = 6;
  msg1.DATA[0] = 0x05;
  msg1.DATA[1] = 0xB5;
  float tilt_velocity_degrees = static_cast<float>(-(180.0 / M_PI) * tilt_velocity_radians); // Tilt is flipped
  uint32_t tilt_velocity_le = float_to_le(tilt_velocity_degrees);
  std::memcpy(&msg1.DATA[2], &tilt_velocity_le, sizeof(uint32_t));

  msg2 = msg1;
  msg2.ID = 0x50E;
  float pan_velocity_degrees = static_cast<float>((180.0 / M_PI) * pan_velocity_radians);
  uint32_t pan_velocity_le = float_to_le(pan_velocity_degrees);
  std::memcpy(&msg2.DATA[2], &pan_velocity_le, sizeof(uint32_t));

  return send({ msg1, msg2 });
}


}   // namespace
