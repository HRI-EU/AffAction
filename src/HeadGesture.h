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

#ifndef AFF_HEADGESTURE_H
#define AFF_HEADGESTURE_H

#include <ControllerBase.h>

#include <vector>
#include <string>


namespace aff
{

class HeadGesture
{
public:

  HeadGesture(const std::string& name, double duration, std::vector<int> jointIds);
  virtual ~HeadGesture();
  std::vector<double> stepPrecise(const Rcs::ControllerBase* controller, MatNd* taskActivations, RcsGraph* targetGraph, double dt);
  void step(const RcsGraph* graph, RcsGraph* targetGraph, double dt);
  void start();
  void setAmplitude(double amplitude);
  void setNumTurns(int numTurns);
  std::string getName() const;

protected:
  void updateHeuristic(const RcsGraph* graph, RcsGraph* targetGraph,
                       double pan_gesture, double tilt_gesture);
  virtual std::vector<double> computePanTilt(double t) = 0;

  std::string name;
  double t_gesture;
  double gestureDuration;
  double amplitude;
  int numTurns;
  int panJointId;
  int tiltJointId;
  std::vector<int> jointIds;
};

class HeadNod : public HeadGesture
{
public:
  HeadNod(const std::string& name, double duration, std::vector<int> jointIds);
  std::vector<double> computePanTilt(double t);
};

class HeadShake : public HeadGesture
{
public:
  HeadShake(const std::string& name, double duration, std::vector<int> jointIds);
  std::vector<double> computePanTilt(double t);
};



}   // namespace aff


#endif   // AFF_HEADGESTURE_H
