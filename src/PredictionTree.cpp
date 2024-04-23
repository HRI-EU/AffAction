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
#include "ActionFactory.h"
#include "ConcurrentExecutor.h"

#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>

#include <iomanip>
#include <queue>
#include <algorithm>
#include <memory>
#include <fstream>


// When computing the duration scaling factor to touch the velocity limits,
// the max. joint velocity over all joints and all time steps will exactly
// be at the limit. This scaling factor extends the duration so that there
// is a margin between the max. reached joint velocity and its limit.
#define TURBO_DURATION_SCALER (1.25)


namespace aff
{
size_t PredictionTreeNode::uniqueIdCount = 0;

PredictionTreeNode::PredictionTreeNode() :
  success(false), cost(0.0), accumulatedCost(0.0), idx(-1), uniqueId(uniqueIdCount++),
  level(0), parent(nullptr), graph(nullptr), threadNumber(0)
{
}

PredictionTreeNode::PredictionTreeNode(PredictionTreeNode* parent_,
                                       const TrajectoryPredictor::PredictionResult& pr) :
  success(pr.success), cost(pr.cost()), accumulatedCost(0.0), idx(pr.idx), uniqueId(uniqueIdCount++),
  bodyTransforms(success ? pr.bodyTransforms : std::vector<double>()),
  parent(parent_), graph(pr.graph), threadNumber(0)
{
  accumulatedCost = parent->accumulatedCost + cost;
  level = parent->level+1;
}

PredictionTreeNode::~PredictionTreeNode()
{
  RcsGraph_destroy(graph);

  for (auto& child : children)
  {
    delete child;
  }
}

std::string PredictionTreeNode::actionCommand() const
{
  return resolvedActionCommand;//action ? action->getActionCommand() : "ROOT";
}

// The lesser function for sorting a vector of results. The failure -
// success comparisons ensure that the first-ranked solutions are valid.
bool PredictionTreeNode::lesser(const PredictionTreeNode* a,
                                const PredictionTreeNode* b)
{
  if (a->success && !b->success)
  {
    return true;
  }

  if (!a->success && b->success)
  {
    return false;
  }

  return a->accumulatedCost < b->accumulatedCost;
}

PredictionTreeNode* PredictionTreeNode::addChild(const TrajectoryPredictor::PredictionResult& pr)
{
  PredictionTreeNode* newChild = new PredictionTreeNode(this, pr);
  children.push_back(newChild);
  return newChild;
}

size_t PredictionTreeNode::size(bool recursive) const
{
  size_t nBytes = 0;

  nBytes += sizeof(bool);
  nBytes += 2*sizeof(double);
  nBytes += 2*sizeof(int);
  nBytes += bodyTransforms.size()*sizeof(double);
  nBytes += sizeof(PredictionTreeNode*);
  nBytes += children.size()*sizeof(PredictionTreeNode*);
  nBytes += RcsGraph_sizeInBytes(graph);

  if (recursive)
  {
    for (const auto& child : children)
    {
      nBytes += child->size(recursive);
    }
  }

  return nBytes;
}

void PredictionTreeNode::print() const
{
  std::vector<int> levelIdx;
  levelIdx.push_back(idx);
  const PredictionTreeNode* ptr = parent;
  while (ptr)
  {
    levelIdx.push_back(ptr->idx);
    ptr = ptr->parent;
  }

  std::reverse(levelIdx.begin(), levelIdx.end());

  //std::lock_guard<std::mutex> lock(staticLock);
  for (size_t i = 0; i < levelIdx.size(); ++i)
  {
    std::cout << levelIdx[i];

    if (i<levelIdx.size()-1)
    {
      std::cout << " - ";
    }

  }

  std::cout << "   thread: " << threadNumber;
  // std::cout << "   launchedThreads: " << launchedThreads;
  std::cout << std::endl;
}



//=============================================================================
//
//
//=============================================================================

PredictionTree::PredictionTree() : root(nullptr), t_calc(0.0)
{
  root = new PredictionTreeNode();
  root->success = true;
}

PredictionTree::~PredictionTree()
{
  delete root;
}

size_t PredictionTree::size() const
{
  RCHECK(root);
  size_t nBytes = root->size(true);

  return nBytes;
}

void PredictionTree::getNodes(std::vector<PredictionTreeNode*>& collection, PredictionTreeNode* node) const
{
  if (!node)
  {
    node = root;
  }

  collection.insert(collection.end(), node->children.begin(), node->children.end());

  for (const auto& child : node->children)
  {
    getNodes(collection, child);
  }

}

size_t PredictionTree::getNumNodes() const
{
  std::vector<PredictionTreeNode*> collection;
  getNodes(collection, nullptr);
  return collection.size();
}

void PredictionTree::getLeafNodes(std::vector<PredictionTreeNode*>& collection, bool onlySuccessfulOnes, PredictionTreeNode* node) const
{
  if (!node)
  {
    node = root;
  }

  if (node->children.empty() && (node->level==incomingActionSequence.size()))
  {
    if ((!onlySuccessfulOnes) || (onlySuccessfulOnes&&node->success))
    {
      collection.push_back(node);
    }
  }

  for (const auto& child : node->children)
  {
    getLeafNodes(collection, onlySuccessfulOnes, child);
  }

}

size_t PredictionTree::getNumValidPaths() const
{
  std::vector<PredictionTreeNode*> successfulLeafs;
  getLeafNodes(successfulLeafs, true, root);
  return successfulLeafs.size();
}

std::vector<PredictionTreeNode*> PredictionTree::getNodesAtDepth(size_t depth, bool successfullOnly = true) const
{
  std::vector<PredictionTreeNode*> nodesAtDepth;

  // Use a queue for level-order traversal
  std::queue<PredictionTreeNode*> nodeQueue;
  nodeQueue.push(root);

  int currentDepth = 0;

  // Traverse the tree level by level until reaching the specified depth or exhausting all levels
  while (!nodeQueue.empty() && currentDepth <= depth)
  {
    int nodesAtCurrentLevel = nodeQueue.size();

    // Process nodes at the current level
    while (nodesAtCurrentLevel > 0)
    {
      PredictionTreeNode* currentNode = nodeQueue.front();
      nodeQueue.pop();

      // Add the node to the result if it is at the specified depth
      if (currentDepth == depth)
      {
        nodesAtDepth.push_back(currentNode);
      }

      // Add child nodes to the queue for further traversal
      for (PredictionTreeNode* child : currentNode->children)
      {
        // Include the child node in the traversal if success condition is met or success condition is not enforced
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

void PredictionTree::printNodesAtEachDepth() const
{
  // Check if the tree is empty
  if (root == nullptr)
  {
    std::cout << "Tree is empty." << std::endl;
    return;
  }

  // Initialize a queue for level-order traversal starting from the root
  std::queue<PredictionTreeNode*> nodeQueue;
  nodeQueue.push(root);

  int depth = 0;

  // Perform breadth-first traversal of the tree
  while (!nodeQueue.empty())
  {
    // Get the number of nodes at the current level
    int levelSize = nodeQueue.size();

    // Print the depth and the number of nodes at the current level
    std::cout << "PredictionTree: Depth " << depth << ": " << levelSize << " nodes" << std::endl;

    // Process nodes at the current level
    for (int i = 0; i < levelSize; ++i)
    {
      // Retrieve the current node from the front of the queue
      PredictionTreeNode* current = nodeQueue.front();
      nodeQueue.pop();

      // Add child nodes of the current node to the queue for further traversal
      for (PredictionTreeNode* child : current->children)
      {
        nodeQueue.push(child);
      }
    }

    // Move to the next depth level
    ++depth;
  }
}

bool PredictionTree::toDotFile(const std::string& filename) const
{
  std::ofstream fd;
  fd.open(filename);

  if (!fd.good())
  {
    RLOG_CPP(4, "Couldn't open file '" << filename << "'");
    return false;
  }

  // File header
  fd << "digraph G {\nbgcolor=\"white\"\n";
  fd << "graph [fontname=\"fixed\"];\n";
  fd << "node [fontname=\"fixed\"];\n";
  fd << "edge [fontname=\"fixed\"];\n";
  fd << "rankdir = \"LR\";\n";

  // Collect all nodes
  std::vector<PredictionTreeNode*> nodes;
  nodes.push_back(root);
  getNodes(nodes);

  for (const auto& n : nodes)
  {
    auto aCmd = n->actionCommand();//n->action ? n->action->getActionCommand() : "ROOT";
    auto words = Rcs::String_split(aCmd, " ");
    std::string label;
    for (size_t i = 0; i < words.size(); ++i)
    {
      label += words[i];
      label += " ";
      if (i == 2)
      {
        label += "\\n";
      }
    }
    label += "\\n(level " + std::to_string(n->level) + " of " +
             std::to_string(incomingActionSequence.size()) + ")";

    fd << n->uniqueId << "[label=\"" << label << "\" ";

    if (n->children.empty())
    {
      fd << "style=filled ";

      if (n->success && (n->level==incomingActionSequence.size()))
      {
        fd << "color=\"green\"];\n";
      }
      else
      {
        fd << "color=\"red\"];\n";
      }

    }
    else
    {
      fd << "style=filled color=gray];\n";
    }

  }

  for (const auto& n : nodes)
  {
    for (const auto& c : n->children)
    {
      fd << n->uniqueId << "->" << c->uniqueId << " [label=\"" << c->cost
         << "\\n" << c->accumulatedCost << "\"];\n";
    }
  }

  // File end
  fd << "}\n";
  fd.close();

  return true;
}

void PredictionTree::printTreeVisual(const PredictionTreeNode* node, int indent) const
{
  // Check if the node is nullptr, if so, return without printing
  if (node == nullptr)
  {
    return;
  }

  // Handle the root node separately
  if (node->parent == nullptr)
  {
    std::cout << "Root" << " : " << "0" << std::endl;
  }
  else
    // Print the current node's value with proper indentation
  {
    std::cout << std::setw(indent) << "|     " << std::endl;
    std::cout << std::setw(indent) << "|     " << std::endl;
    std::cout << std::setw(indent) << "+---> ";

    // Check if the node is successful or not, and print accordingly
    if (node->success)
    {
      std::cout << node->idx << " : " << node->actionCommand() << " : " << node->cost << std::endl;
    }
    else
    {
      std::cout << node->idx << " : " << node->actionCommand() << " : " << "FAIL" << std::endl;
    }

  }

  // Recursively print children of the current node
  for (const PredictionTreeNode* child : node->children)
  {
    printTreeVisual(child, indent + 6);
  }

  if (node==root)
  {
    auto sln = findSolutionPath();
    bool solutionFound = !sln.empty();
    if (!sln.empty() && (sln.size() != incomingActionSequence.size()))
    {
      solutionFound = false;
      RLOG(0, "Mismatch in solution size and incoming sequence: %zu %zu",
           sln.size(), incomingActionSequence.size());
    }

    std::cout << std::endl << "Incoming sequence: " << std::endl;
    for (size_t i = 0; i < incomingActionSequence.size(); i++)
    {
      std::cout << "  Action #" << i << ": '" << incomingActionSequence[i] << "'";
      // if (!sln.empty())
      // {
      //   std::cout << "  becomes: '" << sln[i]->action->getActionCommand() << "'";
      // }
      std::cout << std::endl;
    }

    std::cout << std::endl << "Solution path: " << std::endl;
    for (size_t i = 0; i < sln.size(); i++)
    {
      std::cout << "  Resolved #" << i << ": `" << sln[i]->actionCommand() << "'";
      std::cout << std::endl;
    }

    std::cout << "Number of nodes: " << getNumNodes() << std::endl;
    std::cout << "Number of valid solution paths: " << getNumValidPaths() << std::endl;
    if (!sln.empty())
    {
      std::cout << "Minimum cost: " << sln.back()->accumulatedCost << std::endl;
    }
    std::cout << "Calculation time[sec]: " << t_calc << std::endl << std::endl;
  }
}


// Wrapper function
int PredictionTree::getMaxDepth() const
{
  return getMaxDepthRecursive(root);
}

int PredictionTree::getMaxDepthRecursive(const PredictionTreeNode* node) const
{
  // Check if the node is nullptr, if so, return 0 indicating depth of an empty tree
  if (!node)
  {
    return 0;
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
                                                   std::pair<double, std::vector<PredictionTreeNode*>>& bestPath) const
{
  // Skip unsuccessful nodes or nodes with nullptr
  if (!node || !node->success)
  {
    return;
  }

  // Add the current node to the current path
  currentPath.push_back(const_cast<PredictionTreeNode*>(node));

  // Update the current cost
  currentCost += node->cost;

  // Check if the target depth is reached or exceeded
  if (currentDepth >= targetDepth)
  {
    // Output the current path for debug
    for (size_t i=0; i<currentPath.size(); ++i)
    {
      RLOG(0, "currentPath[%zu] = %s", i, currentPath[i]->actionCommand().c_str());
    }

    // Check if the current path has a smaller cost than the best path found so far
    if (currentCost < bestPath.first)
    {
      bestPath.first = currentCost;
      bestPath.second = currentPath;
      RLOG_CPP(0, "Current solution has cost: " << currentCost << " *** new winner ***");
    }
    else
    {
      RLOG_CPP(0, "Current solution has cost: " << currentCost);
    }

    // Remove the current node from the current path (backtrack)
    RLOG(0, "Popping '%s'", currentPath.back()->actionCommand().c_str());
    currentPath.pop_back();
    return;
  }

  // Recursive exploration of children
  for (const auto& child : node->children)
  {
    findSmallestCostPathRecursive(child, currentDepth + 1, currentCost, currentPath, targetDepth, bestPath);
  }

  // Remove the current node from the current path (backtrack)
  RLOG(0, "Popping '%s'", currentPath.back()->actionCommand().c_str());
  currentPath.pop_back();
}

std::pair<double, std::vector<PredictionTreeNode*>> PredictionTree::findSmallestCostPath(int targetDepth) const
{
  std::pair<double, std::vector<PredictionTreeNode*>> bestPath;
  std::vector<PredictionTreeNode*> initialPath;

  // Limit target depth to the maximum tree depth
  targetDepth = std::min(targetDepth, this->getMaxDepth());

  // Lower cost is better -> set initial cost to infinity
  bestPath.first = DBL_MAX;

  // Recursively search for the smallest cost path starting from the root node
  findSmallestCostPathRecursive(root, 0, 0.0, initialPath, targetDepth, bestPath);

  // Erase root from best path
  if (!bestPath.second.empty())
  {
    bestPath.second.erase(bestPath.second.begin());
  }

  return bestPath;
}

std::vector<PredictionTreeNode*> PredictionTree::findSolutionPath(size_t index, bool onlySuccessfulOnes) const
{
  std::vector<PredictionTreeNode*> leafs, bestPath;

  // Get all successful leaf nodes and order so that the best one is at the first index
  getLeafNodes(leafs, onlySuccessfulOnes);

  if (leafs.empty() || index >= leafs.size())
  {
    return std::vector<PredictionTreeNode*>();
  }

  std::sort(leafs.begin(), leafs.end(), PredictionTreeNode::lesser);

  if (!leafs.empty())
  {
    PredictionTreeNode* nodePtr = leafs[index];
    bestPath.push_back(nodePtr);

    while (nodePtr->parent)
    {
      nodePtr = nodePtr->parent;
      bestPath.push_back(nodePtr);
    }

    bestPath.pop_back();   // Remove root
    std::reverse(bestPath.begin(), bestPath.end());
  }

  return bestPath;
}

std::unique_ptr<PredictionTree> PredictionTree::planActionTree(ActionScene& domain,
                                                               RcsGraph* graph,
                                                               const RcsBroadPhase* broadphase,
                                                               std::vector<std::string> actions,
                                                               size_t stepsToPlan,
                                                               size_t maxNumThreads,
                                                               double dt,
                                                               bool earlyExit,
                                                               std::string& errMsg)
{
  std::unique_ptr<PredictionTree> predictionTree = std::make_unique<PredictionTree>();
  predictionTree->t_calc = Timer_getSystemTime();

  // Strip whitespaces from action commands
  for (size_t i = 0; i < actions.size(); i++)
  {
    Rcs::String_trim(actions[i]);
  }

  predictionTree->incomingActionSequence = actions;

  PredictionTreeNode* currentPredictionNode = predictionTree->root;
  RcsGraph* localGraph = RcsGraph_clone(graph); // Create a local copy of the graph to work with
  const RcsGraph* lookaheadGraph = localGraph;
  int uniqueNodeId = 0;

  // Print sequence
  RLOG(1, "planActionSequence: Sequence to plan:");
  for (size_t i = 0; i < actions.size(); i++)
  {
    RLOG_CPP(1, "Action #" << i << ": `" << actions[i] << "'");
  }

  // Limit the number of steps to lookahead to the sequence length.
  // Predicting less that sequence length steps will guarantee a succesfull
  // execution of the sequence up to that number of actions.
  stepsToPlan = std::min(stepsToPlan, actions.size());

  for (size_t s = 0; s < stepsToPlan; s++)
  {
    // Start predicting the desired number of steps ahead
    std::string text = actions[s];
    REXEC(1)
    {
      RLOG_CPP(1, "Current action #" << s << " `" << text << "'");
      predictionTree->printNodesAtEachDepth();
    }

    std::vector<PredictionTreeNode*> currentStepNodes = predictionTree->getNodesAtDepth(s, true);
    RLOG_CPP(1, "Current depth: " << s << "; Number of branches to explore: " << currentStepNodes.size());

    if (currentStepNodes.size() == 0)
    {
      // In this case, there are no successfull leaf nodes to expand.
      // The sequence will fail after 's' steps in this case.
      RLOG_CPP(0, "Sequence prediction failing after " << s << " steps! No action will be performed!");
      break;
    }

    std::string explanation;// = "Success";
    std::vector<std::string> actionStrings = Rcs::String_split(text, "+");
    std::unique_ptr<ActionBase> action;

    for (PredictionTreeNode* currentPredictionNode : currentStepNodes)
    {
      // Load the graph from the
      if (s > 0)
      {
        lookaheadGraph = currentPredictionNode->graph;
        RCHECK(lookaheadGraph);
      }

      // We are here if a '+' has been detected in the command string. This is a
      // parallel action where the actions that are separated by the '+' sign get
      // instantiated by the MultiStringAction.
      if (actionStrings.size() > 1)
      {
        actionStrings.insert(actionStrings.begin(), "multi_string");
        action = std::unique_ptr<ActionBase>(ActionFactory::create(domain, lookaheadGraph, actionStrings, explanation));
        errMsg += explanation;
      }
      // This is the "normal" action that only has space-separated words.
      else
      {
        std::vector<std::string> words = Rcs::String_split(text, " ");
        action = std::unique_ptr<ActionBase>(ActionFactory::create(domain, lookaheadGraph, words, explanation));
        errMsg += explanation;
      }

      // Early exit if the action could not be created. The particular reason
      // depends on the action and is returned in the explanation string.
      if ((!action) || (action->getNumSolutions() == 0))
      {
        RLOG(0, "Could not create action! Early exit triggered! Explanation: %s", explanation.c_str());
        break;
      }

      double minCost = DBL_MAX;
      std::string minMessage;
      std::vector<TrajectoryPredictor::PredictionResult> predResults(action->getNumSolutions());
      std::vector<std::future<void>> futures;

      // Thread pool with of either number of solutions or available hardware threads (whichever is smaller)
      size_t nThreads = std::thread::hardware_concurrency();

      if (maxNumThreads != 0)
      {
        nThreads = std::min(nThreads, maxNumThreads);
      }

      ConcurrentExecutor predictExecutor(std::min(nThreads, action->getNumSolutions()));

      // enque each solution to be predicted
      for (size_t i = 0; i < action->getNumSolutions(); ++i)
      {
        // Create a new unique_ptr<ActionBase> for each lambda
        const ActionBase* aPtr = action.get();

        futures.push_back(predictExecutor.enqueue([i, &domain, broadphase, lookaheadGraph, aPtr, dt, earlyExit, &predResults]
        {
          const double scaleDurationHint = 1.0;
          auto localAction = aPtr->clone();
          RLOG_CPP(1, "Starting prediction " << i + 1 << " from " << localAction->getNumSolutions());
          localAction->initialize(domain, lookaheadGraph, i);
          double dt_predict = Timer_getSystemTime();
          predResults[i] = localAction->predict(domain, lookaheadGraph, broadphase,
                                                scaleDurationHint * localAction->getDurationHint(), dt, earlyExit);
          predResults[i].idx = i;
          dt_predict = Timer_getSystemTime() - dt_predict;
          predResults[i].message += " command: " + localAction->getActionCommand();
          RLOG(1, "[%s] Action \"%s\" try %zu: took %.1f msec, cost=%f\n\tMessage: %s",
               predResults[i].success ? "SUCCESS" : "FAILURE", localAction->getName().c_str(), i,
               1.0e3 * dt_predict, predResults[i].cost(), predResults[i].message.c_str());
          predResults[i].action = localAction.release();

          // Scale duration to make motion as fast as possible
          if (predResults[i].action->turboMode())
          {
            double newDuration = predResults[i].action->getDurationHint() * predResults[i].scaleJointSpeeds;
            predResults[i].action->setDuration(newDuration* TURBO_DURATION_SCALER);
          }
        }));
      }

      // wait for the predictions to finish
      for (auto& future : futures)
      {
        future.wait();
      }

      // sort predictions
      RLOG_CPP(1, "Done threaded prediction - Sorting " << predResults.size() << " predictions");
      std::sort(predResults.begin(), predResults.end(), TrajectoryPredictor::PredictionResult::lesser);

      // print prediction results
      RLOG_CPP(1, "Printing predictions");
      for (const auto& r : predResults)
      {
        if (r.idx != -1)
        {
          const int verbosityLevel = RcsLogLevel;
          r.print(1);
          auto child = currentPredictionNode->addChild(r);
          child->uniqueId = uniqueNodeId++;
          child->resolvedActionCommand = r.action->getActionCommand();
          RLOG(1, "Tree size[MB]: %.3f", 1.0e-6*predictionTree->size());
        }
      }
      RLOG_CPP(1, predResults.size() << " predictions have been added to the prediction tree");
    }   // for (const auto& currentPredictionNode : currentStepNodes)

    // Here we are done with the current level's parent nodes. We therefore delete its graph
    if (!currentStepNodes.empty())
    {
      RcsGraph_destroy(currentStepNodes[0]->graph);
      currentStepNodes[0]->graph = nullptr;
    }

  }   // for (size_t s = 0; s < stepsToPlan; s++)

  RLOG(1, "Finally destroying graph '%s'", RcsGraph_getConfigFile(localGraph));
  RcsGraph_destroy(localGraph);

  predictionTree->t_calc = Timer_getSystemTime() - predictionTree->t_calc;

  REXEC(0)
  {
    predictionTree->printTreeVisual(predictionTree->root, 0);
    predictionTree->toDotFile("PredictionTree.dot");
  }

  const int treeDepth = predictionTree->getMaxDepth() - 1; //exclude root
  if (treeDepth < stepsToPlan)
  {
    RLOG(0, "Cannot predict a successfull path for the provided sequence! Failure encountered after %d actions.", treeDepth);
    return nullptr;
  }


  return std::move(predictionTree);
}


// void PredictionTree::DFS_MT(ActionScene& scene,
//                             const RcsBroadPhase* broadphase,
//                             std::vector<std::string> actionSequenceStrings,
//                             double dt,
//                             size_t nThreads,
//                             bool earlyExit,
//                             PredictionTreeNode* node,
//                             bool& finished)
// {
//   std::string err;
//   std::vector<std::string> actionParams = Rcs::String_split(actionSequenceStrings[node->level], " ");
//   auto action_ = std::unique_ptr<ActionBase>(ActionFactory::create(scene, node->graph, actionParams, err));

//   for (size_t i = 0; i < (action_ ? action_->getNumSolutions() : 0); ++i)
//   {
//     auto action = action_->clone();
//     action->initialize(scene, node->graph, i);
//     bool actionEarlyExit = true;
//     auto res = action->predict(scene, node->graph, broadphase, action->getDurationHint(), dt, actionEarlyExit);

//     // Scale duration to make motion as fast as possible
//     if (action->turboMode())
//     {
//       double newDuration = action->getDurationHint() * res.scaleJointSpeeds;
//       action->setDuration(newDuration * TURBO_DURATION_SCALER);
//     }

//     // Now the node->graph is correctly initialized for the next tree child
//     PredictionTreeNode* child = new PredictionTreeNode(node, res);
//     child->resolvedActionCommand = action->getActionCommand();
//     node->children.push_back(child);

//     // We stop the recursion in case we found a successful leaf node on the last level.
//     if (earlyExit && child->success && child->level == actionSequenceStrings.size())
//     {
//       finished = true;
//       break;
//     }

//     // Descent to next recursion level
//     if (child->level < actionSequenceStrings.size() && child->success)
//     {
//       DFS(scene, broadphase, actionSequenceStrings, dt, nThreads, earlyExit, child, finished);
//       if (finished)
//       {
//         break;
//       }
//     }
//   }

// }

void PredictionTree::DFS(ActionScene& scene,
                         const RcsBroadPhase* broadphase,
                         std::vector<std::string> actionSequenceStrings,
                         double dt,
                         size_t nThreads,
                         bool earlyExit,
                         PredictionTreeNode* node,
                         bool& finished)
{
  std::string err;
  std::vector<std::string> actionParams = Rcs::String_split(actionSequenceStrings[node->level], " ");
  auto action = std::unique_ptr<ActionBase>(ActionFactory::create(scene, node->graph, actionParams, err));

  for (size_t i=0; i<(action ? action->getNumSolutions() : 0); ++i)
  {
    action->initialize(scene, node->graph, i);
    bool actionEarlyExit = true;
    auto res = action->predict(scene, node->graph, broadphase, action->getDurationHint(), dt, actionEarlyExit);

    // Scale duration to make motion as fast as possible
    double merk = action->getDurationHint();
    if (action->turboMode())
    {
      double newDuration = action->getDurationHint() * res.scaleJointSpeeds;
      action->setDuration(newDuration*TURBO_DURATION_SCALER);
    }

    // Now the node->graph is correctly initialized for the next tree child
    PredictionTreeNode* child = new PredictionTreeNode(node, res);
    child->resolvedActionCommand = action->getActionCommand();
    node->children.push_back(child);
    action->setDuration(merk);

    // We stop the recursion in case we found a successful leaf node on the last level.
    if (earlyExit && child->success && child->level==actionSequenceStrings.size())
    {
      finished = true;
      break;// return;
    }

    // Descent to next recursion level
    if (child->level<actionSequenceStrings.size() && child->success)
    {
      DFS(scene, broadphase, actionSequenceStrings, dt, nThreads, earlyExit, child, finished);
      if (finished)
      {
        break;
      }
    }
  }

}


std::unique_ptr<PredictionTree> PredictionTree::planActionTreeDFT(ActionScene& domain,
                                                                  RcsGraph* graph,
                                                                  const RcsBroadPhase* broadphase,
                                                                  std::vector<std::string> actions,
                                                                  size_t stepsToPlan,
                                                                  size_t maxNumThreads,
                                                                  double dt,
                                                                  bool earlyExit,
                                                                  std::string& errMsg)
{
  // Thread pool with of either number of solutions or available hardware threads (whichever is smaller)
  size_t nThreads = std::thread::hardware_concurrency();

  if (maxNumThreads != 0)
  {
    nThreads = std::min(nThreads, maxNumThreads);
  }

  // Create tree
  std::unique_ptr<PredictionTree> tree = std::make_unique<PredictionTree>();
  tree->t_calc = Timer_getSystemTime();

  // Strip whitespaces from action commands
  for (size_t i = 0; i < actions.size(); i++)
  {
    Rcs::String_trim(actions[i]);
  }

  tree->incomingActionSequence = actions;
  tree->root->graph = RcsGraph_clone(graph);

  bool isFinished = false;
  tree->DFS(domain, broadphase, actions, dt, nThreads, earlyExit, tree->root, isFinished);
  tree->t_calc = Timer_getSystemTime() - tree->t_calc;

  REXEC(0)
  {
    tree->printTreeVisual(tree->root, 0);
    tree->toDotFile("PredictionTreeDFS.dot");
  }

  return tree;
}








/*******************************************************************************
 *
 ******************************************************************************/
static int launchedThreads = 0;
static int maxThreads = std::thread::hardware_concurrency();
static int globalThreadNumber = 0;
static int startedThreads = 0;
static int stoppedThreads = 0;
static std::thread::id mainThreadId;
static std::map<std::thread::id,int> threadIdMap;
std::mutex staticLock;

static void DFSMT(ActionScene& scene,
                  const RcsBroadPhase* broadphase,
                  PredictionTreeNode* node,
                  const std::vector<std::string>& levelCommands,
                  double dt,
                  bool earlyExit,
                  bool& finished);

static void expand(ActionScene& scene,
                   const RcsBroadPhase* broadphase,
                   PredictionTreeNode* node,
                   const std::vector<std::string>& levelCommands,
                   double dt,
                   bool earlyExit,
                   bool& finished,
                   int solutionIndex,
                   std::unique_ptr<ActionBase> action,
                   bool iAmThreaded)
{
  if (iAmThreaded)
  {
    REXEC(1)
    {
      std::lock_guard<std::mutex> lock(staticLock);
      RLOG_CPP(0, "Started thread " << globalThreadNumber
               << " parent: " << node->threadNumber);
    }
    threadIdMap[std::this_thread::get_id()] = globalThreadNumber;
    globalThreadNumber++;
    startedThreads++;
  }

  bool actionEarlyExit = true;
  action->initialize(scene, node->graph, solutionIndex);
  auto res = action->predict(scene, node->graph, broadphase, action->getDurationHint(), dt, actionEarlyExit);
  res.idx = solutionIndex;

  // Scale duration to make motion as fast as possible
  if (action->turboMode())
  {
    double newDuration = action->getDurationHint() * res.scaleJointSpeeds;
    action->setDuration(newDuration * TURBO_DURATION_SCALER);
  }

  // Now the node->graph is correctly initialized for the next tree child
  PredictionTreeNode* child = new PredictionTreeNode(node, res);
  child->resolvedActionCommand = action->getActionCommand();
  //child->threadIdx = std::this_thread::get_id();
  child->threadNumber = globalThreadNumber;
  node->children.push_back(child);

  REXEC(0)
  {
    child->print();
  }

  // We stop the recursion in case we found a successful leaf node on the last level.
  if (earlyExit && child->success && child->level == levelCommands.size())
  {
    finished = true;
    return;
  }

  // Descent to next recursion level
  if ((child->level < levelCommands.size()) && child->success)
  {
    DFSMT(scene, broadphase, child, levelCommands, dt, earlyExit, finished);
    if (finished)
    {
      return;
    }
  }

}







void DFSMT(ActionScene& scene,
           const RcsBroadPhase* broadphase,
           PredictionTreeNode* node,
           const std::vector<std::string>& levelCommands,
           double dt,
           bool earlyExit,
           bool& finished)
{
  std::string err;
  std::vector<std::string> actionParams = Rcs::String_split(levelCommands[node->level], " ");
  auto action = std::unique_ptr<ActionBase>(ActionFactory::create(scene, node->graph, actionParams, err));

  if (!action)
  {
    RLOG_CPP(0, "Could not create this action: '" << levelCommands[node->level] << "'");
    return;
  }

  std::vector<std::thread> workers;

  for (size_t i = 0; i < action->getNumSolutions(); ++i)
  {
    bool threadMe = false;
    if (launchedThreads < maxThreads-1)
    {
      threadMe = true;
    }

    if (!threadMe)
    {
      expand(scene, broadphase, node, levelCommands, dt, earlyExit, finished, i,
             std::unique_ptr<ActionBase>(action->clone()), false);
    }
    else
    {
      launchedThreads++;
      workers.push_back(std::thread(expand, std::ref(scene), broadphase, node, levelCommands, dt, earlyExit,
                                    std::ref(finished), (int)i,
                                    std::unique_ptr<ActionBase>(action->clone()), true));
    }

    if (finished)
    {
      break;
    }

  }

  for (size_t i = 0; i < workers.size(); ++i)
  {
    REXEC(1)
    {
      std::lock_guard<std::mutex> lock(staticLock);
      RLOG_CPP(0, "Stopping thread " << threadIdMap[workers[i].get_id()]
               << " - " << launchedThreads-1 << " running");
    }
    workers[i].join();
    launchedThreads--;
    stoppedThreads++;
  }

}


std::unique_ptr<PredictionTree> PredictionTree::planActionTreeDFT_MT(ActionScene& domain,
                                                                     RcsGraph* graph,
                                                                     const RcsBroadPhase* broadphase,
                                                                     std::vector<std::string> actions,
                                                                     size_t stepsToPlan,
                                                                     size_t maxNumThreads,
                                                                     double dt,
                                                                     bool earlyExit,
                                                                     std::string& errMsg)
{
  // Thread pool with of either number of solutions or available hardware threads (whichever is smaller)
  size_t nThreads = 2*std::thread::hardware_concurrency();

  if (maxNumThreads != 0)
  {
    nThreads = std::min(nThreads, maxNumThreads);
  }

  maxThreads = nThreads;
  RLOG_CPP(0, "Planning with " << maxThreads << " threads - early exit is "
           << (earlyExit ? "TRUE" : "FALSE"));

  // Create tree
  std::unique_ptr<PredictionTree> tree = std::make_unique<PredictionTree>();
  tree->t_calc = Timer_getSystemTime();

  // Strip whitespaces from action commands
  for (size_t i = 0; i < actions.size(); i++)
  {
    Rcs::String_trim(actions[i]);
  }

  tree->incomingActionSequence = actions;
  tree->root->graph = RcsGraph_clone(graph);

  bool isFinished = false;
  DFSMT(domain, broadphase, tree->root, actions, dt, earlyExit, isFinished);
  tree->t_calc = Timer_getSystemTime() - tree->t_calc;

  REXEC(0)
  {
    tree->toDotFile("PredictionTreeDFS.dot");
  }

  REXEC(1)
  {
    tree->printTreeVisual(tree->root, 0);
  }

  RLOG_CPP(0, "Took " << tree->t_calc << " secs for " << tree->getNumNodes() << " nodes");
  RLOG_CPP(0, "Started threads: " << startedThreads << " stopped threads: " << stoppedThreads);

  return tree;
}




}; // namespace aff
