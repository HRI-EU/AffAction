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

#ifndef AFF_PREDICTIONTREE_H
#define AFF_PREDICTIONTREE_H

#include "TrajectoryPredictor.h"

#include <Rcs_graph.h>

#include <unordered_map>

namespace aff
{

class PredictionTreeNode
{
public:

  bool success;
  double quality;
  int idx;
  std::string actionText;
  PredictionTreeNode* parent;
  std::vector<PredictionTreeNode*> children;
  RcsGraph* graph;

  PredictionTreeNode();
  PredictionTreeNode(PredictionTreeNode* parent, const TrajectoryPredictor::PredictionResult* predictionResult);

  ~PredictionTreeNode();

  PredictionTreeNode* addChild(const TrajectoryPredictor::PredictionResult& pr);
};

class PredictionTree
{
public:

  PredictionTreeNode* root;

  PredictionTree();
  ~PredictionTree();

  void printNodesAtEachDepth();
  void printTreeVisual(const PredictionTreeNode* node, int indent);
  std::vector<PredictionTreeNode*> getNodesAtDepth(int depth, bool successfullOnly);
  std::pair<double, std::vector<PredictionTreeNode*>> findSmallestCostPath(int targetDepth);
  int getMaxDepth() const;
  int getMaxDepthRecursive(const PredictionTreeNode* node) const;

private:
  void findSmallestCostPathRecursive(const PredictionTreeNode* node, int currentDepth, double currentCost,
                                     std::vector<PredictionTreeNode*>& currentPath, int targetDepth,
                                     std::pair<double, std::vector<PredictionTreeNode*>>& bestPath);
};

}; // namespace aff

#endif  // AFF_PREDICTIONTREE_H
