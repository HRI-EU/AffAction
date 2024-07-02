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

#include "ActionScene.h"
#include "TrajectoryPredictor.h"

#include <Rcs_graph.h>


namespace aff
{

/**
 * @brief Represents a node in the prediction tree.
 *
 * Each node corresponds to an action and can have children representing different variants/solutions of that action.
 */
class PredictionTreeNode
{
public:
  bool success; /**< Flag indicating whether the action was successfully executed. */
  double cost ; /**< Cost of the action execution. */
  double accumulatedCost; /**< Sum of cost up to this node. */
  int idx; /**< Solution index of the action. */
  int uniqueId; /**< Unique identifier for nodes in tree. */
  int level; /**< 0 for root, increasing for each child level. */
  std::vector<double> bodyTransforms;

  PredictionTreeNode* parent; /**< Pointer to the parent node. */
  std::vector<PredictionTreeNode*> children; /**< Vector of child nodes. */
  RcsGraph* graph; /**< Pointer to the Rcs graph associated with the action. */
  std::string resolvedActionCommand;
  ActionResult feedbackMsg;
  int threadNumber;
  bool fatalError;
  static size_t uniqueIdCount;

  /**
   * @brief Default constructor.
   */
  PredictionTreeNode();

  /**
   * @brief Constructor with parent and prediction result.
   * @param parent Pointer to the parent node.
   * @param predictionResult Pointer to the prediction result to use for initialization.
   */
  PredictionTreeNode(PredictionTreeNode* parent,
                     const TrajectoryPredictor::PredictionResult& predictionResult);

  /**
   * @brief Disallow copying and assigning.
   */
  PredictionTreeNode& operator=(const PredictionTreeNode&) = delete;
  PredictionTreeNode(const PredictionTreeNode&) = delete;

  /**
   * @brief Destructor.
   */
  ~PredictionTreeNode();

  /**
  * @brief Returns the unique action command with all parameters.
  */
  std::string actionCommand() const;

  /**
  * @brief Returns the size of the node in bytes.
  */
  size_t size(bool recursive) const;

  /**
   * @brief The lesser function for sorting a vector of results. Sorting
   *        will be in ascending order (smalles costs at first index)
   */
  static bool lesser(const PredictionTreeNode* a, const PredictionTreeNode* b);

  /**
   * @brief Adds a child node to the current node.
   * @param pr Prediction result representing the child node.
   * @return Pointer to the newly created child node.
   */
  PredictionTreeNode* addChild(const TrajectoryPredictor::PredictionResult& pr);

  void print() const;
};

/**
 * @brief Represents a tree structure for storing action sequences.
 *
 * The tree consists of nodes, where each node represents an action and its solution variants.
 */
class PredictionTree
{
public:

  enum class SearchType
  {
    BFS,
    DFS,
    DFSMT
  };

  PredictionTreeNode* root; /**< Pointer to the root node of the tree. */
  double t_calc;
  std::vector<std::string> incomingActionSequence;

  /**
   * @brief Default constructor.
   */
  PredictionTree();

  /**
   * @brief Destructor.
   */
  ~PredictionTree();

  /**
  * @brief Returns the size of the node in bytes.
  */
  size_t size() const;

  /**
   * @brief Prints the number of nodes at each depth of the tree.
   *
   * This function performs a breadth-first traversal of the tree to print the number of nodes
   * at each depth level. It starts from the root and traverses the tree level by level, printing
   * the depth and the number of nodes at each level.
   * If the tree is empty, it prints a message indicating that the tree is empty.
   */
  void printNodesAtEachDepth() const;

  /**
   * @brief Returns a vector of all tree nodes.
   */
  void getNodes(std::vector<PredictionTreeNode*>& collection, PredictionTreeNode* node=nullptr) const;

  /**
   * @brief Returns the number of tree nodes.
   */
  size_t getNumNodes() const;

  /**
   * @brief Recursively prints the tree in a visual format.
   *
   * This function prints the tree structure in a visual format with indentation to represent
   * the hierarchical relationship between nodes. It recursively traverses the tree starting
   * from the provided node and prints each node along with its children.
   * If the provided node is nullptr, the function returns without printing anything.
   *
   * Example output (initial sequence: get block_1, put_block_1 table_r_far):
   * `get` has 2 solutions - 2 nodes at level 1
   * `put` has 3 solutions in both cases - 3 nodes on each branc at level 2
   *
   * Root : 0
   *   |
   *   |
   *   +---> 0 : get block_1 hand_right PowerGrasp : 0.0155627
   *         |
   *         |
   *         +---> 2 : put block_1 table_r_far : 0.00995091
   *         |
   *         |
   *         +---> 0 : put block_1 table_r_far : 0.020125
   *         |
   *         |
   *         +---> 1 : put block_1 table_r_far : FAIL
   *   |
   *   |
   *   +---> 1 : get block_1 hand_right OrientedGrasp : 0.0204494
   *         |
   *         |
   *         +---> 1 : put block_1 table_r_far : 0.00967657
   *         |
   *         |
   *         +---> 0 : put block_1 table_r_far : 0.0197976
   *         |
   *         |
   *         +---> 2 : put block_1 table_r_far : FAIL
   *
   * // End of example output
   *
   * @param node The node from which to start printing the tree.
   * @param indent The current indentation level for formatting the output.
   */
  void printTreeVisual(const PredictionTreeNode* node, int indent) const;

  /**
   * @brief Writes a dot file of the tree. It can be inspected with any
   *        external dot file viewer (e.g. dotty). Returns true for success,
   *        false otherwise.
   */
  bool toDotFile(const std::string& filename) const;

  /**
   * @brief Retrieves nodes at a specified depth in the tree.
   *
   * @param depth The depth at which to retrieve nodes.
   * @param successfullOnly If true, only successful nodes will be included in the result.
   * @return A vector containing nodes at the specified depth.
   *
   * This function performs a breadth-first search to find nodes at a given depth in the tree.
   * It traverses the tree level by level until it reaches the specified depth or exhausts all levels.
   * Nodes are added to the result vector if they are at the specified depth and meet the success condition.
   * If the tree is empty or the depth is invalid (negative), an empty vector is returned.
   */
  std::vector<PredictionTreeNode*> getNodesAtDepth(size_t depth, bool successfulOnly) const;

  /**
   * @brief Finds the smallest cost path in the prediction tree up to a specified depth.
   *
   * This function finds the smallest cost path in the prediction tree up to a specified depth.
   * It initializes the best path with a maximum cost and recursively searches for the smallest
   * cost path starting from the root node. The search is limited to the specified target depth.
   * After finding the best path, it removes the root node from the path and returns the result.
   *
   * @param targetDepth The target depth up to which the search is performed.
   * @return A pair containing the cost of the smallest path and the nodes on that path.
   */
  std::pair<double, std::vector<PredictionTreeNode*>> findSmallestCostPath(int targetDepth) const;

  /**
   * @brief Finds the i-th smallest cost path in the prediction tree.
   *
   * This function finds the i-th smallest cost path in the prediction tree, where i is
   * parameter index. If index is out of range of the available paths, an empty vector is
   * returned.
   *
   * @param index The best path is index=0, the second-best is index=1 etc.
   * @param onlySuccessfulOnes If false, all leafs are considered, if true only successful ones.
   * @return The i-th best path where i is parameter index.
   */
  std::vector<PredictionTreeNode*> findSolutionPath(size_t index = 0, bool onlySuccessfulOnes = true) const;
  std::vector<std::string> findSolutionPathAsStrings(size_t index = 0, bool onlySuccessfulOnes = true) const;

  std::vector<ActionResult> getSolutionErrorStrings(size_t index = 0) const;

  /**
   * @brief Gets the maximum depth of the tree.
   * @return Maximum depth of the tree.
   */
  int getMaxDepth() const;

  /**
  * @brief Returns the number of successful solution paths.
  */
  size_t getNumValidPaths() const;

  /**
   * @brief Tree search function.
   *
   * @param sType            Type for search algorithm, see enum SearchType
   * @param scene            Reference to scene object
   * @param graph            Pointer to scene's underlying graph
   * @param broadphase       Broadphase collision model of graph
   * @param actionCommands   Sequence of action commands, separated by semicolon
   * @param maxNumThreads    Numer of threads to distribute. A value of 0 means auto-select.
   * @param dt               Prediction time step
   * @param earlyExitSearch  True: quit search on first successful solution, false: compute whole tree
   * @param earlyExitAction  True: Stop predicting action once error encountered, false: Always compute full action duration
   * @param errMsg           Text string comprising error messages
   * @return Search tree or nullptr in case of errors.
  */
  static std::unique_ptr<PredictionTree> planActionTree(SearchType sType,
                                                        ActionScene& scene,
                                                        RcsGraph* graph,
                                                        const RcsBroadPhase* broadphase,
                                                        std::vector<std::string> actionCommands,
                                                        double dt,
                                                        size_t maxNumThreads=0,
                                                        bool earlyExitSearch=true,
                                                        bool earlyExitAction=true);

  static void setTurboDurationScaler(double value);

  static double getTurboDurationScaler();

  static double getDefaultTurboDurationScaler();

private:

  /**
   * @brief Returns a vector of all leaf nodes.
   */
  void getLeafNodes(std::vector<PredictionTreeNode*>& collection,
                    bool onlySuccessfulOnes=false,
                    PredictionTreeNode* node = nullptr) const;

  /**
   * @brief Recursively finds the smallest cost path in the prediction tree.
   *
   * This function recursively explores the prediction tree to find the smallest cost path
   * up to a specified target depth. It maintains the current path and cost, updating them
   * as it traverses the tree. When it reaches the target depth, it compares the cost of the
   * current path with the best path found so far and updates the best path if necessary.
   *
   * @param node The current node being explored.
   * @param currentDepth The current depth in the tree.
   * @param currentCost The current cost of the path.
   * @param currentPath The current path being explored.
   * @param targetDepth The target depth to search up to.
   * @param bestPath The best path found so far, consisting of cost and nodes.
   */
  void findSmallestCostPathRecursive(const PredictionTreeNode* node,
                                     int currentDepth,
                                     double currentCost,
                                     std::vector<PredictionTreeNode*>& currentPath,
                                     int targetDepth,
                                     std::pair<double, std::vector<PredictionTreeNode*>>& bestPath) const;

  /**
   * @brief Recursively calculates the maximum depth of the tree.
   *
   * This function calculates the maximum depth of the tree starting from the provided node.
   * If the provided node is nullptr, it returns 0, indicating the depth of an empty tree.
   * Otherwise, it recursively calculates the depth of each child node and returns the maximum
   * depth among the children, plus 1 for the current node.
   *
   * @param node The node from which to start calculating the maximum depth.
   * @return The maximum depth of the subtree rooted at the provided node.
   */
  int getMaxDepthRecursive(const PredictionTreeNode* node) const;
};

}; // namespace aff

#endif  // AFF_PREDICTIONTREE_H
