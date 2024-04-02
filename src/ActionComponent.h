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

#ifndef AFF_ACTIONCOMPONENT_H
#define AFF_ACTIONCOMPONENT_H

#include "ComponentBase.h"
#include "ActionScene.h"

#include <ControllerBase.h>
#include <TrajectoryPredictor.h>


namespace aff
{

class ActionComponent : public ComponentBase
{
public:

  ActionComponent(EntityBase* parent, const RcsGraph* graph,
                  const RcsBroadPhase* broadphase);
  virtual ~ActionComponent();

  const ActionScene* getDomain() const;
  ActionScene* getDomain();
  void setLimitCheck(bool enable);
  void setMultiThreaded(bool enable);
  bool getLimitCheck() const;
  bool getMultiThreaded() const;

  void setFinalPoseRunning(bool enable);
  bool isFinalPoseRunning() const;

private:

  void onTextCommand(std::string text);
  void onPrint();
  void onRender();
  void onToggleFastPrediction();
  void onSetDebugRendering(bool enable);
  void onStop();

  void actionThread(std::string text);
  ActionScene domain;
  const RcsGraph* graph;
  const RcsBroadPhase* broadphase;
  bool limitsEnabled;
  bool multiThreaded;

  // For animation of predictions
  std::vector<TrajectoryPredictor::PredictionResult> predictions;
  mutable std::mutex renderMtx;
  mutable std::mutex actionThreadMtx;

  RcsGraph* animationGraph;
  int animationTic;
  int animationIdx;
  bool startingFinalPose;

  // Avoid copying this class
  ActionComponent(const ActionComponent&);
  ActionComponent& operator=(const ActionComponent&);
};

}

#endif   // AFF_ACTIONCOMPONENT_H
