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

#include "AnimationSequence.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>

#define NUM_ANIMATION_CHANNELS (3)


namespace aff
{

AnimationSequence::AnimationSequence(EntityBase* parent, const RcsGraph* graph_) :
  ComponentBase(parent), animationGraph(NULL), animationTic(0), animationIncrement(3),
  predictionIdx(-1), channel(0)
{
  this->animationGraph = RcsGraph_clone(graph_);
  predictions.resize(NUM_ANIMATION_CHANNELS);

  subscribe("Render", &AnimationSequence::onRender);
  subscribe("ToggleFastPrediction", &AnimationSequence::onToggleFastPrediction);
  subscribe("SetDebugRendering", &AnimationSequence::onSetDebugRendering);
  subscribe("AnimateSequence", &AnimationSequence::onAnimateSequence);
  subscribe("ZapAnimation", &AnimationSequence::onZapAnimation);
}

AnimationSequence::~AnimationSequence()
{
  RcsGraph_destroy(this->animationGraph);
}

void AnimationSequence::onToggleFastPrediction()
{
  animationTic = 0;
  predictionIdx++;
  if (predictionIdx>=predictions[channel].size())
  {
    predictionIdx = -1;
    onSetDebugRendering(false);
  }

  RLOG(0, "Setting predictionIdx to %d", predictionIdx);
}

void AnimationSequence::onRender()
{
  if ((predictionIdx==-1) ||
      (predictionIdx>=predictions[channel].size()) ||
      (predictions[channel][predictionIdx].bodyTransforms.empty()))
  {
    predictionIdx = -1;
    animationTic = 0;
    return;
  }

  // Copy all transforms from the prediction to the graph
  size_t dblsPerRow = animationGraph->nBodies*12;
  size_t nRows = predictions[channel][predictionIdx].bodyTransforms.size() / dblsPerRow;
  MatNd predArr = MatNd_fromPtr(nRows, dblsPerRow, predictions[channel][predictionIdx].bodyTransforms.data());

  double* src = MatNd_getRowPtr(&predArr, animationTic);
  RCSGRAPH_FOREACH_BODY(animationGraph)
  {
    memcpy(&BODY->A_BI, src, sizeof(HTr));
    src += 12;
  }

  animationTic += animationIncrement;

  if (animationTic >= nRows)
  {
    animationTic = 0;
  }

  std::string col = predictions[channel][predictionIdx].success ? "setGhostModeGreen" : "setGhostModeRed";
  getEntity()->publish<std::string,std::string>("RenderCommand", "FastPrediction", col);
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "FastPrediction", animationGraph);
}

void AnimationSequence::onSetDebugRendering(bool enable)
{
  if (enable==false)
  {
    getEntity()->publish<std::string,std::string>("RenderCommand", "FastPrediction", "erase");
    predictionIdx = -1;
  }

}

void AnimationSequence::onAnimateSequence(std::vector<TrajectoryPredictor::PredictionResult> pred,
                                          int animationChannel)
{
  RLOG(0, "onAnimateSequence with %zu predictions will be assigned to channel %d", pred.size(), animationChannel);
  predictions[animationChannel] = std::move(pred);
}

void AnimationSequence::onZapAnimation()
{
  channel++;
  if (channel >= NUM_ANIMATION_CHANNELS)
  {
    channel = 0;
  }

  predictionIdx = predictions[channel].empty() ?  - 1 : 0;
  animationTic = 0;

  RLOG(0, "Zapping to animation channel %d", channel);
}

}   // namespace aff
