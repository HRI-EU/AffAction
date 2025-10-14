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

#ifndef AFF_ROBODRIVER_H
#define AFF_ROBODRIVER_H

#include <vector>
#include <map>
#include <mutex>



class RoboDriver
{
public:

protected:


  // Command data struct
  struct JointCommand
  {
    int index;
    double position_command;
    double vmax;
    double tmc;
    bool has_position_command;
    bool has_vmax;
    bool has_tmc;
  };

  struct RoboCommand
  {
    RoboCommand() : quitMe(false), newCommand(false)
    {
    }

    std::map<std::string, JointCommand> jointCommands;
    bool quitMe;
    bool newCommand;
  };

  mutable std::mutex cmdMtx;
  RoboCommand incomingCommand;

  RoboCommand getCommand() const
  {
    std::lock_guard<std::mutex> lock(cmdMtx);
    return this->incomingCommand;
  }

  virtual size_t getDOF() const = 0;

  virtual double getMinTMC() const = 0;

  virtual std::vector<double> getMaxVel() const = 0;

  virtual double getMaxVel(size_t index) const
  {
    return getMaxVel()[index];
  }

  std::vector<double> getDesiredQ() const
  {
    std::vector<double> q_des;


    return q_des;
  }

};

#endif
