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

// g++ -std=c++11 -o test_pw70_can_interface PW70CANInterface.cpp -pthread -DMAIN

#ifndef PW70CANINTERFACELINUX_H
#define PW70CANINTERFACELINUX_H

#include <cmath>
#include <thread>
#include <vector>
#include <functional>
#include <mutex>
#include <linux/can.h>

namespace aff
{

class PW70CANInterface
{
public:
  PW70CANInterface(std::function<void(double, double, void*)> limit_check_callback,
                   std::function<void(double, double, double, void*)> position_callback,
                   void* param, int freq);
  ~PW70CANInterface();

  void cleanup();
  bool send(const std::vector<struct can_frame>& frames);
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
  int s; // SocketCAN socket
  std::mutex socket_mutex;
  std::thread recv_thread;
  bool running;
  void receive_messages();

  std::function<void(double, double, void*)> limit_check_callback;
  std::function<void(double, double, double, void*)> position_callback;

  void* callbackParam;
};

}   // namespace

#endif // PW70CANINTERFACELINUX_H



#include <iostream>
#include <cstring>
#include <map>
#include <chrono>
#include <cstdint>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <endian.h>



namespace aff
{


PW70CANInterface::PW70CANInterface(std::function<void(double, double, void*)> limit_check_callback,
                                   std::function<void(double, double, double, void*)> position_callback,
                                   void* param, int freq)
  : limit_check_callback(limit_check_callback), position_callback(position_callback),
    running(true), callbackParam(param)
{
  // Open CAN socket
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("Error while opening socket");
    exit(EXIT_FAILURE);
  }

  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, "can0");
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
  {
    perror("Error in ioctl");
    exit(EXIT_FAILURE);
  }

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // Bind the socket
  if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    perror("Error in socket bind");
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
  std::cout << "Joining CAN receiver thread" << std::endl;
  running = false;
  if (recv_thread.joinable())
  {
    recv_thread.join();
  }

  // Close the socket
  std::cout << "Done joining CAN receiver thread - now closing socket" << std::endl;
  close(s);
  std::cout << "Dnoe closing socket - finished cleanup" << std::endl;
}

// Send method
bool PW70CANInterface::send(const std::vector<struct can_frame>& frames)
{
  std::lock_guard<std::mutex> lock(socket_mutex);
  for (const auto& frame : frames)
  {
    int nbytes = write(s, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame))
    {
      perror("Error sending CAN frame");
      return false;
    }
  }
  return true;
}

// Receive messages method
void PW70CANInterface::receive_messages()
{
  struct can_frame frame = {};
  bool pan_updated = false;
  bool tilt_updated = false;

  float pan_value_radians = 0.0f;
  float tilt_value_radians = 0.0f;

  while (running)
  {
    int nbytes;
    {
      // Lock the socket mutex for thread-safe access
      std::lock_guard<std::mutex> lock(socket_mutex);
      frame = {};
      nbytes = read(s, &frame, sizeof(struct can_frame));
    }

    if (nbytes > 0)
    {
      // Process the message
      uint32_t id = frame.can_id & CAN_SFF_MASK;

      if (id == 0x70D || id == 0x70E)
      {
        if (frame.data[0] == 0x07 && frame.data[1] == 0x95)
        {
          uint32_t angle_bytes;
          std::memcpy(&angle_bytes, &frame.data[2], sizeof(uint32_t));
          angle_bytes = le32toh(angle_bytes);
          float angle_degrees = *reinterpret_cast<float*>(&angle_bytes);

          if (id == 0x70D)
          {
            tilt_value_radians = -(M_PI/180.0)*angle_degrees;  // Set tilt flipped.
            tilt_updated = true;
          }
          else if (id == 0x70E)
          {
            pan_value_radians = (M_PI/180.0)*angle_degrees;    // Set pan.
            pan_updated = true;
          }

          // Check if both pan and tilt have been updated
          if (pan_updated && tilt_updated)
          {
            // Get current time
            auto current_time = std::chrono::system_clock::now();
            double current_time_sec = std::chrono::duration<double>(current_time.time_since_epoch()).count();

            // Call the callback after both pan and tilt have been updated
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
    else if (nbytes == 0)
    {
      // End of file (socket closed)
      std::cerr << "Socket closed or EOF reached." << std::endl;
      running = false;
      break;
    }
    else
    {
      // nbytes < 0, an error occurred
      if (errno == EINTR)
      {
        // Interrupted system call, continue reading
        continue;
      }
      else
      {
        perror("Error reading CAN frame");
        running = false;
        break;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::cout << "Quitting CAN receiver thread" << std::endl;
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

  struct can_frame msg1 = {};
  msg1.can_id = 0x50D;
  msg1.can_dlc = can_state_cmd.size();
  std::copy(can_state_cmd.begin(), can_state_cmd.end(), msg1.data);

  struct can_frame msg2 = {};
  msg2.can_id = 0x50E;
  msg2.can_dlc = can_state_cmd.size();
  std::copy(can_state_cmd.begin(), can_state_cmd.end(), msg2.data);

  return send({msg1, msg2});
}

// Disable frequent position update method
bool PW70CANInterface::disable_frequent_position_update()
{
  struct can_frame msg1 = {};
  msg1.can_id = 0x50D;
  msg1.can_dlc = 2;
  msg1.data[0] = 0x01;
  msg1.data[1] = 0x95;

  struct can_frame msg2 = {};
  msg2.can_id = 0x50E;
  msg2.can_dlc = 2;
  msg2.data[0] = 0x01;
  msg2.data[1] = 0x95;

  return send({msg1, msg2});
}

// Stop method
bool PW70CANInterface::stop()
{
  struct can_frame msg1 = {};
  msg1.can_id = 0x50D;
  msg1.can_dlc = 2;
  msg1.data[0] = 0x01;
  msg1.data[1] = 0x91;

  struct can_frame msg2 = {};
  msg2.can_id = 0x50E;
  msg2.can_dlc = 2;
  msg2.data[0] = 0x01;
  msg2.data[1] = 0x91;

  return send({msg1, msg2});
}

// Fast stop method
bool PW70CANInterface::fast_stop()
{
  struct can_frame msg1 = {};
  msg1.can_id = 0x50D;
  msg1.can_dlc = 2;
  msg1.data[0] = 0x01;
  msg1.data[1] = 0x90;

  struct can_frame msg2 = {};
  msg2.can_id = 0x50E;
  msg2.can_dlc = 2;
  msg2.data[0] = 0x01;
  msg2.data[1] = 0x90;

  return send({msg1, msg2});
}

// Reference pan method
bool PW70CANInterface::reference_pan()
{
  struct can_frame msg = {};
  msg.can_id = 0x50E;
  msg.can_dlc = 2;
  msg.data[0] = 0x01;
  msg.data[1] = 0x92;

  return send({msg});
}

// Reference tilt method
bool PW70CANInterface::reference_tilt()
{
  struct can_frame msg = {};
  msg.can_id = 0x50D;
  msg.can_dlc = 2;
  msg.data[0] = 0x01;
  msg.data[1] = 0x92;

  return send({msg});
}

// Reset stop method
bool PW70CANInterface::reset_stop()
{
  struct can_frame msg1 = {};
  msg1.can_id = 0x50D;
  msg1.can_dlc = 2;
  msg1.data[0] = 0x01;
  msg1.data[1] = 0x8B;

  struct can_frame msg2 = {};
  msg2.can_id = 0x50E;
  msg2.can_dlc = 2;
  msg2.data[0] = 0x01;
  msg2.data[1] = 0x8B;

  return send({msg1, msg2});
}

// Set target velocity method
bool PW70CANInterface::set_target_velocity(double pan_velocity_radians, double tilt_velocity_radians)
{
  struct can_frame msg1 = {};
  msg1.can_id = 0x50D;
  msg1.can_dlc = 6;
  msg1.data[0] = 0x05;
  msg1.data[1] = 0xA0;
  float tilt_velocity_degrees = static_cast<float>(-(180.0/M_PI)*tilt_velocity_radians); // Tilt is flipped
  uint32_t tilt_velocity_le = htole32(*reinterpret_cast<uint32_t*>(&tilt_velocity_degrees));
  std::memcpy(&msg1.data[2], &tilt_velocity_le, sizeof(uint32_t));

  struct can_frame msg2 = {};
  msg2.can_id = 0x50E;
  msg2.can_dlc = 6;
  msg2.data[0] = 0x05;
  msg2.data[1] = 0xA0;
  float pan_velocity_degrees = static_cast<float>((180.0/M_PI)*pan_velocity_radians);
  uint32_t pan_velocity_le = htole32(*reinterpret_cast<uint32_t*>(&pan_velocity_degrees));
  std::memcpy(&msg2.data[2], &pan_velocity_le, sizeof(uint32_t));

  return send({msg1, msg2});
}

// Set target position method
bool PW70CANInterface::set_target_position(double pan_radians, double tilt_radians)
{
  struct can_frame msg1 = {};
  msg1.can_id = 0x50D;
  msg1.can_dlc = 6;
  msg1.data[0] = 0x05;
  msg1.data[1] = 0xB0;
  float tilt_pos_degrees = static_cast<float>(-(180.0/M_PI)*tilt_radians); // Tilt is flipped
  uint32_t tilt_pos_le = htole32(*reinterpret_cast<uint32_t*>(&tilt_pos_degrees));
  std::memcpy(&msg1.data[2], &tilt_pos_le, sizeof(uint32_t));

  struct can_frame msg2 = {};
  msg2.can_id = 0x50E;
  msg2.can_dlc = 6;
  msg2.data[0] = 0x05;
  msg2.data[1] = 0xB0;
  float pan_pos_degrees = static_cast<float>((180.0/M_PI)*pan_radians);
  uint32_t pan_pos_le = htole32(*reinterpret_cast<uint32_t*>(&pan_pos_degrees));
  std::memcpy(&msg2.data[2], &pan_pos_le, sizeof(uint32_t));

  return send({msg1, msg2});
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
  struct can_frame msg1 = {}, msg2 = {};

  msg1.can_id = 0x50D;
  msg1.can_dlc = 6;
  msg1.data[0] = 0x05;
  msg1.data[1] = 0xB5;
  float tilt_velocity_degrees = static_cast<float>(-(180.0/M_PI)*tilt_velocity_radians); // Tilt is flipped
  uint32_t tilt_velocity_le = htole32(*reinterpret_cast<uint32_t*>(&tilt_velocity_degrees));
  std::memcpy(&msg1.data[2], &tilt_velocity_le, sizeof(uint32_t));

  msg2.can_id = 0x50E;
  msg2.can_dlc = 6;
  msg2.data[0] = 0x05;
  msg2.data[1] = 0xB5;
  float pan_velocity_degrees = static_cast<float>((180.0/M_PI)*pan_velocity_radians);
  uint32_t pan_velocity_le = htole32(*reinterpret_cast<uint32_t*>(&pan_velocity_degrees));
  std::memcpy(&msg2.data[2], &pan_velocity_le, sizeof(uint32_t));

  return send({msg1, msg2});
}

}   // namespace
