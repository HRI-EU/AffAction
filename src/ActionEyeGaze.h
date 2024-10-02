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

#ifndef AFF_ACTIONEYEGAZE_H
#define AFF_ACTIONEYEGAZE_H


#include "ActionBase.h"



namespace aff
{

class ActionEyeGaze : public ActionBase
{
public:

  ActionEyeGaze(const ActionScene& domain,
                const RcsGraph* graph,
                std::vector<std::string> params);

  virtual ~ActionEyeGaze();
  std::unique_ptr<ActionBase> clone() const override;

  std::string getGazeTarget() const;
  static bool computePupilCoordinates(const RcsGraph* graph, double p_right[3], double p_left[3]);
  static bool setPupilSpeedWeight(RcsGraph* graph, double weight);

  static std::string getRightGazePointName();
  static std::string getLeftGazePointName();
  static std::string getGazePointName();
  static std::string getScreenName();
  static std::string getRightPupilName();
  static std::string getLeftPupilName();

protected:

  std::vector<std::string> createTasksXML() const;
  tropic::TCS_sptr createTrajectory(double t_start, double t_end) const;
  std::vector<std::string> getManipulators() const;
  double getDefaultDuration() const;
  std::string getActionCommand() const;

  std::string agentName;
  std::string gazeTarget;
  std::string gazeTargetInstance;
  std::string cameraFrame;
  std::string taskGaze;
  std::vector<std::string> usedManipulators;
  bool isGazeTargetInHand;
  bool keepTasksActiveAfterEnd;
};

}   // namespace aff



#endif // AFF_ACTIONEYEGAZE_H
