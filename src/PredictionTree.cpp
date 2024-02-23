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

#include "PredictionTree.h"

#include <Rcs_macros.h>

#include <iomanip>
#include <queue>
#include <algorithm>


namespace aff
{

PredictionTreeNode::PredictionTreeNode() : success(false), quality(0.0), idx(-1), parent(nullptr), graph(nullptr)
{
  actionText = "root";
}

PredictionTreeNode::~PredictionTreeNode()
{
  RcsGraph_destroy(graph);

  for (auto& child : children)
  {
    delete child;
  }
}

PredictionTreeNode::PredictionTreeNode(PredictionTreeNode* parent, const TrajectoryPredictor::PredictionResult* pr) :
  success(false), quality(0.0), idx(-1), parent(nullptr), graph(nullptr)
{
  if (pr != nullptr)
  {
    this->graph = pr->graph;
    this->actionText = pr->actionText;
    this->quality = pr->quality();
    this->success = pr->success;
    this->idx = pr->idx;
  }
  else
  {
    RLOG(0,"Could not create PredicitonTreeNode, invalid PredictionResult provided!");
  }

  this->parent = parent;
}

PredictionTreeNode* PredictionTreeNode::addChild(const TrajectoryPredictor::PredictionResult& pr)
{
  PredictionTreeNode* newChild = new PredictionTreeNode(this, &pr);
  children.push_back(newChild);
  return newChild;
}

//=============================================================================
//
//
//=============================================================================

PredictionTree::PredictionTree()
{
  root = new PredictionTreeNode();
  root->success = true;
}

PredictionTree::~PredictionTree()
{
  delete root;
}

std::vector<PredictionTreeNode*> PredictionTree::getNodesAtDepth(int depth, bool successfullOnly = true)
{
  std::vector<PredictionTreeNode*> nodesAtDepth;

  // Return an empty vector if the tree is empty or depth is invalid
  if (root == nullptr || depth < 0)
  {
    return nodesAtDepth;
  }

  // Use a queue for level-order traversal
  std::queue<PredictionTreeNode*> nodeQueue;
  nodeQueue.push(root);

  int currentDepth = 0;

  while (!nodeQueue.empty() && currentDepth <= depth)
  {
    int nodesAtCurrentLevel = nodeQueue.size();

    while (nodesAtCurrentLevel > 0)
    {
      PredictionTreeNode* currentNode = nodeQueue.front();
      nodeQueue.pop();

      if (currentDepth == depth)
      {
        nodesAtDepth.push_back(currentNode);
      }

      for (PredictionTreeNode* child : currentNode->children)
      {
        if (!successfullOnly || child->success)
        {
          nodeQueue.push(child);
        }
      }

      nodesAtCurrentLevel--;
    }

    currentDepth++;
  }

  return nodesAtDepth;
}

void PredictionTree::printNodesAtEachDepth()
{
  if (root == nullptr)
  {
    std::cout << "Tree is empty." << std::endl;
    return;
  }

  std::queue<PredictionTreeNode*> nodeQueue;
  nodeQueue.push(root);

  int depth = 0;

  while (!nodeQueue.empty())
  {
    int levelSize = nodeQueue.size();
    std::cout << "PredictionTree: Depth " << depth << ": " << levelSize << " nodes" << std::endl;

    for (int i = 0; i < levelSize; ++i)
    {
      PredictionTreeNode* current = nodeQueue.front();
      nodeQueue.pop();

      for (PredictionTreeNode* child : current->children)
      {
        nodeQueue.push(child);
      }
    }

    ++depth;
  }
}

void PredictionTree::printTreeVisual(const PredictionTreeNode* node, int indent)
{
  if (node == nullptr)
  {
    return;
  }

  // root case
  if (node->parent == nullptr)
  {
    std::cout << "Root" << " : " << "0" << std::endl;
  }
  else
    // Print the current node's value
  {
    std::cout << std::setw(indent) << "|     " << std::endl;
    std::cout << std::setw(indent) << "|     " << std::endl;
    std::cout << std::setw(indent) << "+---> ";
    if (node->success)
    {
      std::cout << node->idx << " : " << node->actionText << " : " << node->quality << std::endl;
    }
    else
    {
      std::cout << node->idx << " : " << node->actionText << " : " << "FAIL" << std::endl;
    }

  }

  // Recursively print children
  for (const PredictionTreeNode* child : node->children)
  {
    printTreeVisual(child, indent + 6);
  }
}

int PredictionTree::getMaxDepth() const
{
  return getMaxDepthRecursive(root);
}

int PredictionTree::getMaxDepthRecursive(const PredictionTreeNode* node) const
{
  if (!node)
  {
    return 0; // Depth of an empty tree is 0
  }

  // Calculate the depth of each child recursively
  std::vector<int> childDepths;
  for (const auto& child : node->children)
  {
    childDepths.push_back(getMaxDepthRecursive(child));
  }

  // If node has no children, its depth is 1
  if (childDepths.empty())
  {
    return 1;
  }

  // Return the maximum depth among children + 1 for the current node
  return 1 + *std::max_element(childDepths.begin(), childDepths.end());
}

void PredictionTree::findSmallestCostPathRecursive(const PredictionTreeNode* node,
                                                   int currentDepth,
                                                   double currentCost,
                                                   std::vector<PredictionTreeNode*>& currentPath,
                                                   int targetDepth,
                                                   std::pair<double, std::vector<PredictionTreeNode*>>& bestPath)
{
  if (!node || !node->success)
  {
    return; // Skip unsuccessful nodes
  }

  // Add the current node to the current path
  currentPath.push_back(const_cast<PredictionTreeNode*>(node));

  // Update the current cost
  currentCost += node->quality;

  // Check if the target depth is reached or exceeded
  if (currentDepth >= targetDepth)
  {
    for (size_t i=0; i<currentPath.size(); ++i)
    {
      RLOG(1, "currentPath[%zu] = %s", i, currentPath[i]->actionText.c_str());
    }

    // Check if the current path has a smaller cost than the best path found so far
    if (currentCost < bestPath.first)
    {
      bestPath.first = currentCost;
      bestPath.second = currentPath;
      RLOG_CPP(1, "Current solution has cost: " << currentCost << " *** new winner ***");
    }
    else
    {
      RLOG_CPP(1, "Current solution has cost: " << currentCost);
    }

    RLOG(1, "Popping '%s'", currentPath.back()->actionText.c_str());
    currentPath.pop_back();
    return;
  }

  // Recursive exploration of children
  for (const auto& child : node->children)
  {
    findSmallestCostPathRecursive(child, currentDepth + 1, currentCost, currentPath, targetDepth, bestPath);
  }

  // Remove the current node from the current path (backtrack)
  RLOG(1, "Popping '%s'", currentPath.back()->actionText.c_str());
  currentPath.pop_back();
}

std::pair<double, std::vector<PredictionTreeNode*>> PredictionTree::findSmallestCostPath(int targetDepth)
{
  std::pair<double, std::vector<PredictionTreeNode*>> bestPath;
  std::vector<PredictionTreeNode*> initialPath;

  // cannot exceed max tree depth
  targetDepth = std::min(targetDepth, this->getMaxDepth());

  // INF initial cost
  bestPath.first = DBL_MAX;

  findSmallestCostPathRecursive(root, 0, 0.0, initialPath, targetDepth, bestPath);

  // Erase root from best path
  if (!bestPath.second.empty())
  {
    bestPath.second.erase(bestPath.second.begin());
  }

  return bestPath;
}

}; // namespace aff
