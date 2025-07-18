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

#ifndef AFF_ROBONETWORKINFO_H
#define AFF_ROBONETWORKINFO_H

#include <string>
#include <vector>
#include <map>



namespace aff
{

/*******************************************************************************
 * Convenience class for maintaining networking information for the robots
 ******************************************************************************/
class RoboNetworkInfo
{
public:
  std::string robo_ip;
  int roboToRemotePort;
  int remoteToRoboPort;
  std::string robo_computer_ip; // Default: "localhost"
  std::string roboSender;       // Default: "tcp://*:5555"
  std::string roboReceiver;     // Default: "tcp://*:5556";
  std::string remoteReceiver;   // Default: "tcp://localhost:5555"
  std::string remoteSender;     // Default: "tcp://localhost:5556"
  std::string roboMode;
  std::vector<double> q_default_deg;

  static const RoboNetworkInfo* getNetworkInfo(const std::string roboName)
  {
    // 7-dof Jaco Gen3 (right mounted)
    RoboNetworkInfo rummenigge;
    rummenigge.robo_ip = "10.107.149.2";
    rummenigge.roboToRemotePort = 40002;// was 5555
    rummenigge.remoteToRoboPort = 40003;// was 5556
    rummenigge.robo_computer_ip = "localhost";
    rummenigge.roboSender = "tcp://*:" + std::to_string(rummenigge.roboToRemotePort);
    rummenigge.roboReceiver = "tcp://*:" + std::to_string(rummenigge.remoteToRoboPort);
    rummenigge.remoteReceiver = "tcp://" + rummenigge.robo_computer_ip + ":" + std::to_string(rummenigge.roboToRemotePort);
    rummenigge.remoteSender = "tcp://" + rummenigge.robo_computer_ip + ":" + std::to_string(rummenigge.remoteToRoboPort);
    rummenigge.roboMode = "LowLevel";
    rummenigge.q_default_deg = { 220.0, 80.0, 80.0, -100.0, 0.0, -40.0, -110.0 };

    // 7-dof Jaco Gen3 (left mounted)
    RoboNetworkInfo littbarski;
    littbarski.robo_ip = "10.107.149.3";
    littbarski.roboToRemotePort = 40004;// was 5557
    littbarski.remoteToRoboPort = 40005;// was 5558
    littbarski.robo_computer_ip = "localhost";
    littbarski.roboSender = "tcp://*:" + std::to_string(littbarski.roboToRemotePort);
    littbarski.roboReceiver = "tcp://*:" + std::to_string(littbarski.remoteToRoboPort);
    littbarski.remoteReceiver = "tcp://" + littbarski.robo_computer_ip + ":" + std::to_string(littbarski.roboToRemotePort);
    littbarski.remoteSender = "tcp://" + littbarski.robo_computer_ip + ":" + std::to_string(littbarski.remoteToRoboPort);
    littbarski.roboMode = "LowLevel";
    littbarski.q_default_deg = { -40.0, -80.0, -80.0, 100.0, 0.0, 40.0, 110.0 };

    // Just Jaco Gen3 simulation
    RoboNetworkInfo test_right = rummenigge;
    test_right.roboMode = "TestWithoutRobot";

    // Just Jaco Gen3 simulation
    RoboNetworkInfo test_left = littbarski;
    test_left.roboMode = "TestWithoutRobot";

    // PTU
    RoboNetworkInfo ptu;
    ptu.robo_ip = "";
    ptu.roboToRemotePort = 40006;
    ptu.remoteToRoboPort = 40007;
    ptu.robo_computer_ip = "localhost";
    ptu.roboSender = "tcp://*:" + std::to_string(ptu.roboToRemotePort);
    ptu.roboReceiver = "tcp://*:" + std::to_string(ptu.remoteToRoboPort);
    ptu.remoteReceiver = "tcp://" + ptu.robo_computer_ip + ":" + std::to_string(ptu.roboToRemotePort);
    ptu.remoteSender = "tcp://" + ptu.robo_computer_ip + ":" + std::to_string(ptu.remoteToRoboPort);
    ptu.roboMode = "";
    ptu.q_default_deg = { 0.0, 0.0 };

    // PTU (test mode without real ptu)
    RoboNetworkInfo ptu_test = ptu;
    ptu_test.roboMode = "TestWithoutRobot";



    static std::map<std::string, RoboNetworkInfo> nwInfo =
    {
      { "rummenigge", rummenigge },
      { "littbarski", littbarski },
      { "test_right", test_right },
      { "test_left",  test_left  },
      { "ptu",        ptu        },
      { "ptu_test",   ptu_test   }
    };

    auto it = nwInfo.find(roboName);

    if (it == nwInfo.end())
    {
      return nullptr;
    }

    return &it->second;
  }

};

}   // namespace

#endif