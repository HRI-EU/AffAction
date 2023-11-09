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

#include "ActionComponent.h"

#if defined (SMILEACTIONS_WITH_OPTIM)
#include "ActionProblem.h"
#endif

#include <ActionFactory.h>
#include <ActionBase.h>
#include <ActionGaze.h>// \todo(MG): Remove from here.
#include <ActionPut.h>// \todo(MG): Remove from here.
#include <ConcurrentExecutor.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_utilsCPP.h>
#include <TaskFactory.h>
#include <Rcs_timer.h>
#include <Rcs_utils.h>

#include <fstream>
#include <iostream>
#include <cstdio>
#include <algorithm>
#include <random>// \todo(MG) remove if HACK is gone


// \todo(MG): This is only for testing and should be 1 and 1
const double scaleDurationHint = 1.0;   // More than 1 makes trajectories longer
const double scaleDtPrediction = 1.0;   // Higher makes predictions compute faster


namespace aff
{

ActionComponent::ActionComponent(EntityBase* parent, const RcsGraph* graph_,
                                 const RcsBroadPhase* broadphase_) :
  ComponentBase(parent), graph(graph_), broadphase(broadphase_), limitsEnabled(true),
  animationGraph(NULL), animationTic(0), animationIdx(-1)
{
  subscribe("TextCommand", &ActionComponent::onTextCommand);
  subscribe("Print", &ActionComponent::onPrint);
  subscribe("Render", &ActionComponent::onRender);
  subscribe("ToggleFastPrediction", &ActionComponent::onToggleFastPrediction);
  subscribe("SetDebugRendering", &ActionComponent::onSetDebugRendering);
  subscribe("Stop", &ActionComponent::onStop);

  this->domain = ActionScene::parse(graph->cfgFile);
  RCHECK(domain.check(graph));

  if (File_exists("action_sequence.txt"))
  {
    RLOG(1, "File action_sequence.txt exists - deleting");
    remove("action_sequence.txt");
  }

  this->animationGraph = RcsGraph_clone(graph_);
}

ActionComponent::~ActionComponent()
{
  RcsGraph_destroy(this->animationGraph);
}

void ActionComponent::onStop()
{
  // Wait until the actionThread has finished
  std::lock_guard<std::mutex> lock(actionThreadMtx);
}

void ActionComponent::onTextCommand(std::string text)
{
  RMSG_CPP("RECEIVED: " << text);

  // All sequences must have been resolved before this point.
  RCHECK_MSG(text.find(';') == std::string::npos,
             "Received string with semicolon: '%s'", text.c_str());

  // reset and get_state are taken care of somewhere else
  //if ((text!="reset") && (text!="get_state"))
  if ((!STRNEQ(text.c_str(), "reset", 5)) && (text!="get_state"))
  {
    std::thread t1(&ActionComponent::actionThread, this, text);
    t1.detach();
  }

}

void ActionComponent::onPrint()
{
  domain.print();
}

void ActionComponent::actionThread(std::string text)
{
  // Reentrancy lock
  std::lock_guard<std::mutex> lock(actionThreadMtx);

  std::string explanation = "Success";
  std::vector<std::string> actionStrings = Rcs::String_split(text, "+");
  std::unique_ptr<ActionBase> action;

  // We are here if a '+' has been detected in the command string. This is a
  // parallel action where the actions that are separated by the '+' sign get
  // instantiated by the MultiStringAction.
  if (actionStrings.size()>1)
  {
    actionStrings.insert(actionStrings.begin(), "multi_string");
    action = std::unique_ptr<ActionBase>(ActionFactory::create(domain, graph, actionStrings, explanation));
  }
  // This is the "normal" action that only has space-separated words.
  else
  {
    std::vector<std::string> words = Rcs::String_split(text, " ");
    action = std::unique_ptr<ActionBase>(ActionFactory::create(domain, graph, words, explanation));
  }

  // Early exit if the action could not be created. The particular reason
  // depends on the action and is returned in the explanation string.
  if ((!action) || (action->getNumSolutions()==0))
  {
    getEntity()->publish("ActionResult", false, 0.0, explanation);
    return;
  }


  bool predictMe = true;

  if (predictMe)
  {
    double minCost = DBL_MAX;
    std::string minMessage;
    std::vector<TrajectoryPredictor::PredictionResult> predResults(action->getNumSolutions());

    if (getMultiThreaded())
    {
      RLOG_CPP(0, "Using multi-threaded prediction");

      std::vector<std::future<void>> futures;

      // Thread pool with of either number of solutions or available hardware threads (whichever is smaller)
      ConcurrentExecutor predictExecutor(
        (size_t)std::thread::hardware_concurrency() > action->getNumSolutions() ?
        action->getNumSolutions() : (size_t)std::thread::hardware_concurrency());

      // enque each solution to be predicted
      for (size_t i = 0; i < action->getNumSolutions(); ++i)
      {
        // Create a new unique_ptr<ActionBase> for each lambda
        //auto localAction = action->clone();
        const ActionBase* aPtr = action.get();

        //futures.push_back(predictExecutor.enqueue([i, this, localAction = std::move(localAction), &predResults]
        futures.push_back(predictExecutor.enqueue([i, this, aPtr, &predResults]
        {
          auto localAction = aPtr->clone();
          RLOG_CPP(0, "Starting prediction " << i+1 << " from " << localAction->getNumSolutions());
          localAction->initialize(domain, graph, i);
          double dt_predict = Timer_getSystemTime();
          predResults[i] = localAction->predict(graph, broadphase, scaleDurationHint*localAction->getDurationHint(), getEntity()->getDt());
          predResults[i].idx = i;
          dt_predict = Timer_getSystemTime() - dt_predict;
          predResults[i].message += " command: " + localAction->getActionCommand();
          RLOG(0, "[%s] Action \"%s\" try %zu: took %.1f msec, jlCost=%f, collCost=%f\n\tMessage: %s",
               predResults[i].success ? "SUCCESS" : "FAILURE", localAction->getName().c_str(), i,
               1.0e3 * dt_predict, predResults[i].jlCost, predResults[i].collCost,
               predResults[i].message.c_str());
        }));
      }

      // wait for the predictions to finish
      for (auto& future : futures)
      {
        future.wait();
      }
    }
    else
    {
      // predict each solution sequentially
      for (size_t i = 0; i < action->getNumSolutions(); ++i)
      {
        RLOG_CPP(1, "Using single-threaded prediction");

        RLOG_CPP(1, "Starting prediction " << i << " from " << action->getNumSolutions());
        action->initialize(domain, graph, i);
        double dt_predict = Timer_getSystemTime();
        predResults[i] = action->predict(graph, broadphase, scaleDurationHint*action->getDurationHint(), getEntity()->getDt());
        predResults[i].idx = i;
        dt_predict = Timer_getSystemTime() - dt_predict;
        RLOG(0, "[%s] Action \"%s\" try %zu: took %.1f msec, jlCost=%f, collCost=%f\n\tMessage: %s",
             predResults[i].success ? "SUCCESS" : "FAILURE", action->getName().c_str(), i,
             1.0e3 * dt_predict, predResults[i].jlCost, predResults[i].collCost,
             predResults[i].message.c_str());
      }
    }

    for (size_t i = 0; i < action->getNumSolutions(); ++i)
    {
      if (predResults[i].success)
      {
        if (predResults[i].quality() < minCost)
        {
          minCost = predResults[i].quality();

          // Uncomment the next line for early exit. It means that in the case
          // of only failures, the ranking is not correct, since we don't
          // consider the same time range for all actions.
          // break;
        }
      }
    }

    RLOG_CPP(0, "Sorting " << predResults.size() << " predictions");
    std::sort(predResults.begin(), predResults.end(), TrajectoryPredictor::PredictionResult::lesser);


    // HACK for action get shuffle where to put
    //if (dynamic_cast<ActionPut*>(action.get()))
    //{
    //  RLOG(-1, "WARNING: The put action does select a random target");
    //  std::vector<TrajectoryPredictor::PredictionResult> successResults;

    //  for (size_t i=0; i<predResults.size(); ++i)
    //  {
    //    if (predResults[i].success)
    //    {
    //      successResults.push_back(predResults[i]);
    //    }
    //  }

    //  std::random_device rd;
    //  std::mt19937 g(rd());

    //  std::shuffle(successResults.begin(), successResults.end(), g);
    //  predResults = successResults;
    //}

    // END Hack


    REXEC(0)
    {
      RLOG_CPP(0, "Printing predictions");
      for (const auto& r : predResults)
      {
        if (r.idx != -1)
        {
          const int verbosityLevel = 0;
          r.print(verbosityLevel);
        }
      }
    }

    // We ignore the action request if no valid prediction was found. We don't
    // necessarily need to do this here, since there is a final check in the
    // TrajectoryComponent. This is a bit more accessible, since we can "see"
    // the trajectory with the 'd' key.
    // if ((!predResults[0].success) && getLimitCheck())
    // {
    //   getEntity()->publish("ActionResult", false, 0.0, predResults[0].message);
    //   return;
    // }

    // We initialize the action with the best prediction that was found.
    RLOG_CPP(0, "Initializing with solution " << predResults[0].idx);
    action->initialize(domain, graph, predResults[0].idx);

    // Memorize the predictions for debug visualization. This runs concurrently
    // with the onRender method, so we are quick about it with swapping, and
    // make it mutually exclusive.
    std::lock_guard<std::mutex> lock(renderMtx);
    // renderMtx.lock();
    animationIdx = -1;
    std::swap(predictions, predResults);
    // renderMtx.unlock();

  }   // if (predictMe)







#if defined (SMILEACTIONS_WITH_OPTIM)
  bool optimizeMe = false;

  if (optimizeMe)
  {
    RLOG(1, "Starting optimization");
    std::vector<double> u = optimize(action.get(), graph,
                                     5.0*getEntity()->getDt(), "Power", 100);
  }
#endif

  std::vector<std::string> taskVec = action->createTasksXML();

  if (taskVec.empty())
  {
    explanation = "ERROR: Found invalid tasks for action REASON: That's an error in the program. SUGGESTION: File a bug report for my program. DEVELOPER: " + action->getActionCommand();
    getEntity()->publish("ActionResult", false, 0.0, explanation);
    return;
  }

  // We need to publish the new task vector and the trajectory. The new
  // trajectory is started a few time steps in the future so that the
  // step function does properly initialize the last few values for a
  // smooth initialization.
  const double delay = 5.0*getEntity()->getDt();
  tropic::TCS_sptr tSet = action->createTrajectory(delay, scaleDurationHint*action->getDurationHint()+delay);

  // From here on, the action will start going.
  getEntity()->publish("FreezePerception", true);
  getEntity()->publish("ChangeTaskVector", taskVec, action->getManipulators());
  getEntity()->publish("CheckAndSetTrajectory", tSet);
  getEntity()->publish("Speak", action->explain());

  REXEC(1)
  {
    std::ofstream writer("action_sequence.txt", std::ios::app);

    if (!writer.good())
    {
      RLOG_CPP(1, "Error opening file action_sequence.txt");
    }
    else
    {
      writer << text << std::endl;
      writer.close();
    }
  }

  // \todo(MG): HACK for foveated objects. We currently do this here, since the
  // actions only receive const references to the affordance models ans scenes.
  // For multi-agent settings, we also should consider several foveated items etc.
  ActionGaze* aGaze = dynamic_cast<ActionGaze*>(action.get());
  if (aGaze)
  {
    domain.foveatedEntity = aGaze->getGazeTarget();
    RLOG_CPP(0, "Now gazing at " << domain.foveatedEntity);
  }
}

const ActionScene* ActionComponent::getDomain() const
{
  return &domain;
}

ActionScene* ActionComponent::getDomain()
{
  return &domain;
}

void ActionComponent::setLimitCheck(bool enable)
{
  limitsEnabled = enable;
}

void ActionComponent::setMultiThreaded(bool enable)
{
  multiThreaded = enable;
}

bool ActionComponent::getLimitCheck() const
{
  return limitsEnabled;
}

bool ActionComponent::getMultiThreaded() const
{
  return multiThreaded;
}

void ActionComponent::onToggleFastPrediction()
{
  animationTic = 0;
  animationIdx++;
  if (animationIdx>=predictions.size())
  {
    animationIdx = -1;
    onSetDebugRendering(false);
  }

  RLOG(0, "Setting animationIdx to %d", animationIdx);
}

void ActionComponent::onRender()
{
  if ((animationIdx==-1) || (animationIdx>=predictions.size()))
  {
    animationIdx = -1;
    animationTic = 0;
    return;
  }

  std::lock_guard<std::mutex> guard(renderMtx);

  // Copy all transforms from the prediction to the graph
  size_t dblsPerRow = animationGraph->nBodies*12;
  size_t nRows = predictions[animationIdx].bodyTransforms.size() / dblsPerRow;
  MatNd predArr = MatNd_fromPtr(nRows, dblsPerRow, predictions[animationIdx].bodyTransforms.data());

  double* src = MatNd_getRowPtr(&predArr, animationTic);
  RCSGRAPH_FOREACH_BODY(animationGraph)
  {
    memcpy(&BODY->A_BI, src, sizeof(HTr));
    src += 12;
  }

  animationTic += 1;// \todo(MG) was 10

  if (animationTic >= nRows)
  {
    animationTic = 0;
  }

  std::string col = predictions[animationIdx].success ? "setGhostModeGreen" : "setGhostModeRed";
  getEntity()->publish<std::string,std::string>("RenderCommand", "FastPrediction", col);
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "FastPrediction", animationGraph);
}

void ActionComponent::onSetDebugRendering(bool enable)
{
  if (enable==false)
  {
    getEntity()->publish<std::string,std::string>("RenderCommand", "FastPrediction", "erase");
    animationIdx = -1;
  }

}

}   // namespace aff
