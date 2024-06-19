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

#ifndef AFF_ACTIONGAZEATHAND_H
#define AFF_ACTIONGAZEATHAND_H


#include "ActionBase.h"



namespace aff
{

class ActionGazeAtHand : public ActionBase
{
public:

  ActionGazeAtHand(const ActionScene& domain,
                   const RcsGraph* graph,
                   std::vector<std::string> params);

  virtual ~ActionGazeAtHand();
  std::unique_ptr<ActionBase> clone() const override;
  std::string getActionCommand() const;
  void print() const;
  std::vector<std::string> getManipulators() const;

protected:

  std::vector<std::string> createTasksXML() const;
  tropic::TCS_sptr createTrajectory(double t_start, double t_end) const;

  std::string graspingHandName;
  std::string shakeEntityName;
  std::string shoulderBaseFrame;
  std::string cameraBaseFrame;
  std::string roboBaseFrame;
  std::string taskCenterXYZ, taskRelShoulderY, taskObjectOri;
  std::vector<std::string> usedManipulators;
};

}   // namespace aff



#endif // AFF_ACTIONGAZEATHAND_H
