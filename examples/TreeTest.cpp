/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include <Rcs_macros.h>
#include <Rcs_cmdLine.h>
#include <Rcs_timer.h>
#include <Rcs_basicMath.h>

#include <SegFaultHandler.h>

#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>


RCS_INSTALL_ERRORHANDLERS


/*******************************************************************************
 *
 ******************************************************************************/
static int launchedThreads = 0;
static int maxThreads = std::thread::hardware_concurrency();
static int branchingFactor = 5;
static int depth = 4;
static int globalThreadNumber = 0;
static int startedThreads = 0;
static int stoppedThreads = 0;
static std::thread::id mainThreadId;
static std::map<std::thread::id,int> threadIdMap;


/*******************************************************************************
 *
 ******************************************************************************/
static std::vector<std::string> createAlphabet()
{
  std::vector<std::string> alphabet;
  for (char c = 'A'; c <= 'Z'; ++c)
  {
    alphabet.push_back(std::string(1, c));
  }
  return alphabet;
}

/*******************************************************************************
 *
 ******************************************************************************/
class Action
{
public:
  static std::unique_ptr<Action> create()
  {
    return std::make_unique<Action>();
  }

  size_t getNumSolutions() const
  {
    return branchingFactor;
  }

  void initialize(size_t i)
  {
  }

  bool predict() const
  {
    double t_wait = Math_getRandomNumber(0.0, 0.05);
    Timer_waitDT(t_wait);
    return true;
  }

  Action* clone()
  {
    return new Action(*this);
  }

};

/*******************************************************************************
 *
 ******************************************************************************/
class PredictionNode
{
public:
  PredictionNode(PredictionNode* node) :
    parent(node), success(false), level(0), solutionIdx(0), threadNumber(0), threadIdx(std::this_thread::get_id())
  {
    level = parent ? parent->level + 1 : 0;
  }

  size_t numNodes() const
  {
    size_t n = children.size();

    for (const auto& child : children)
    {
      n += child->numNodes();
    }

    return n;
  }

  ~PredictionNode()
  {
    for (size_t i=0; i<children.size(); ++i)
    {
      delete children[i];
    }
  }

  void print() const
  {
    std::vector<size_t> levelIdx;
    levelIdx.push_back(solutionIdx);
    const PredictionNode* ptr = parent;
    while (ptr)
    {
      levelIdx.push_back(ptr->solutionIdx);
      ptr = ptr->parent;
    }

    std::reverse(levelIdx.begin(), levelIdx.end());

    std::lock_guard<std::mutex> lock(staticLock);
    for (size_t i = 0; i < levelIdx.size(); ++i)
    {
      std::cout << levelIdx[i];

      if (i<levelIdx.size()-1)
      {
        std::cout << " - ";
      }

    }

    std::cout << "   thread: " << threadNumber;
    std::cout << "   launchedThreads: " << launchedThreads;
    std::cout << std::endl;
  }

  static void expand(PredictionNode* node, const std::vector<std::string>& levelCommands,
                     bool earlyExit, bool& finished, int solutionIndex, std::unique_ptr<Action> action,
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

    action->initialize(solutionIndex);
    bool success = action->predict();

    // Now the node->graph is correctly initialized for the next tree child
    PredictionNode* child = new PredictionNode(node);
    child->success = success;
    child->solutionIdx = solutionIndex;
    child->threadIdx = std::this_thread::get_id();
    child->threadNumber = globalThreadNumber;
    node->children.push_back(child);

    REXEC(2)
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
      DFS(child, levelCommands, earlyExit, finished);
      if (finished)
      {
        return;
      }
    }

  }

  static void DFS(PredictionNode* node, const std::vector<std::string>& levelCommands,
                  bool earlyExit, bool& finished)
  {
    auto action = Action::create();
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
        expand(node, levelCommands, earlyExit, finished, i, std::unique_ptr<Action>(action->clone()), false);
      }
      else
      {
        launchedThreads++;
        workers.push_back(std::thread(expand, node, levelCommands, earlyExit, std::ref(finished),
                                      (int)i, std::unique_ptr<Action>(action->clone()), true));
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
        RLOG_CPP(0, "Stopping thread " << threadIdMap[workers[i].get_id()] << " - " << launchedThreads-1 << " running");
      }
      workers[i].join();
      launchedThreads--;
      stoppedThreads++;
    }

  }


protected:

  PredictionNode* parent;
  std::vector<PredictionNode*> children;
  bool success;
  size_t level;
  int solutionIdx;
  int threadNumber;
  std::thread::id threadIdx;
  static std::mutex staticLock;
};

std::mutex PredictionNode::staticLock;

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  mainThreadId = std::this_thread::get_id();
  bool finished = false;

  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel);
  argP.getArgument("-branchingFactor", &branchingFactor, "Default: %d", branchingFactor);
  argP.getArgument("-depth", &depth, "Default: %d", depth);
  argP.getArgument("-maxThreads", &maxThreads, "Default: %d", maxThreads);
  bool earlyExit = argP.hasArgument("-earlyExit", "Quit on first encountered solution");

  auto abc = createAlphabet();
  std::vector<std::string> levelCommands;

  for (int i=0; i<depth; ++i)
  {
    levelCommands.push_back(abc[i]);
  }

  auto root = std::make_unique<PredictionNode>(nullptr);

  double t_tree = Timer_getSystemTime();
  PredictionNode::DFS(root.get(), levelCommands, earlyExit, finished);
  t_tree = Timer_getSystemTime() - t_tree;

  argP.print();
  RLOG_CPP(0, "Took " << t_tree << " secs for " << root->numNodes() << " nodes");
  RLOG_CPP(0, "Started threads: " << startedThreads << " stopped threads: " << stoppedThreads);

  return 0;
}
