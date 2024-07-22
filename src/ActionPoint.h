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

#ifndef AFF_ACTIONPOINT_H
#define AFF_ACTIONPOINT_H


#include "ActionBase.h"



namespace aff
{

class ActionPoint : public ActionBase
{
public:

  ActionPoint(const ActionScene& domain,
              const RcsGraph* graph,
              std::vector<std::string> params);

  virtual ~ActionPoint();
  std::unique_ptr<ActionBase> clone() const override;
  std::string getActionCommand() const;
  size_t getNumSolutions() const;
  bool initialize(const ActionScene& domain, const RcsGraph* graph, size_t solutionRank);
  void print() const;
  double actionCost(const ActionScene& domain,
                    const RcsGraph* graph) const;

protected:

  std::vector<std::string> createTasksXML() const;
  tropic::TCS_sptr createTrajectory(double t_start, double t_end) const;
  std::vector<std::string> getManipulators() const;
  double pointDistance(const ActionScene& scene, const RcsGraph* graph,
                       const std::string& finger, const std::string& object) const;

  std::string pointBdyName;
  std::string pointerFrame;
  std::string shoulderFrame;
  std::string fingerJoints;
  std::string taskOri, taskFingers, taskFingerTip;
  std::vector<std::string> usedManipulators;
  bool keepTasksActiveAfterEnd;
  double pointDirection[3];
  double fingerTipPosition[3];
  std::vector<double> pointingFingerAngles;
  std::vector<std::tuple<std::string, std::string,double>> manipulatorEntityMap;
};

}   // namespace aff



#endif // AFF_ACTIONPOINT_H
