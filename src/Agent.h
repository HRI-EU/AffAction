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

#ifndef AFF_AGENT_H
#define AFF_AGENT_H

#include "Manipulator.h"


namespace aff
{

class ActionScene;

class Agent
{
public:
  std::string name;
  std::string type;
  std::vector<std::string> manipulators;

  Agent(const xmlNodePtr node, const ActionScene* scene);
  static Agent* createAgent(const xmlNodePtr node, ActionScene* scene);
  virtual void print() const;
  virtual ~Agent();
  virtual std::string isLookingAt() const;
};

class RobotAgent : public Agent
{
public:
  RobotAgent(const xmlNodePtr node, const ActionScene* scene);
  virtual ~RobotAgent();
  int getPanTilt(const RcsGraph* graph, const std::string& gazeTarget,
                 double panTilt[2], size_t maxIter, double eps,
                 double err[2]) const;
};

class HumanAgent : public Agent
{
public:
  bool isVisible;
  double lastTimeSeen;
  std::string tracker;
  std::vector<HTr> markers;   // Vector of tracked body links
  double defaultPos[3];
  double defaultRadius;
  std::string gazeTarget;
  std::string gazeTargetPrev;
  double gazeDirection[3];
  double headPosition[3];

  HumanAgent(const xmlNodePtr node, const ActionScene* scene);
  virtual ~HumanAgent();
  void setMarkers(const std::vector<HTr>& newMarkers);
  void setVisibility(const bool newVisibilty);
  void projectMarkersOnManipulators(const RcsGraph* graph);
  bool hasHead() const;
  bool getHeadTransform(HTr* A_HI) const;
  bool getHeadPosition(double pos[3]) const;
  bool getGazeDirection(double dir[3]) const;
  bool getHeadUpAxis(double dir[3]) const;
  std::string isLookingAt() const;
};

} // namespace aff

#endif // AFF_AGENT_H
