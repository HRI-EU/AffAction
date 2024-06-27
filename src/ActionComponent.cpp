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
#include <ConcurrentExecutor.h>
#include <PredictionTree.h>
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




namespace aff
{

ActionComponent::ActionComponent(EntityBase* parent, const RcsGraph* graph_,
                                 const RcsBroadPhase* broadphase_) :
  ComponentBase(parent), domain(graph_->cfgFile), graph(graph_),
  broadphase(broadphase_), limitsEnabled(true), multiThreaded(true),
  startingFinalPose(false), earlyExitPrediction(true)
{
  subscribe("Print", &ActionComponent::onPrint);
  subscribe("Stop", &ActionComponent::onStop);

  domain.initializeKinematics(graph);
  RCHECK(domain.check(graph));
}

ActionComponent::~ActionComponent()
{
}

void ActionComponent::onStop()
{
  // Wait until the actionThread has finished
  std::lock_guard<std::mutex> lock(actionThreadMtx);
}

void ActionComponent::onPrint()
{
  domain.print();
}

void ActionComponent::actionThread(std::string text)
{
  RLOG(0, "actionThread text: `%s`", text.c_str());

  // Reentrancy lock
  std::lock_guard<std::mutex> lock(actionThreadMtx);

  ActionResult explanation;
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
    std::vector<ActionResult> fbmsg;
    fbmsg.push_back(explanation);
    getEntity()->publish("ActionResult", false, 0.0, fbmsg);
    return;
  }


  std::vector<TrajectoryPredictor::PredictionResult> predictions(action->getNumSolutions());

  if (getMultiThreaded() && (action->getNumSolutions()>1))
  {
    std::vector<std::future<void>> futures;

    // Thread pool with of either number of solutions or available hardware threads (whichever is smaller)
    size_t maxThreads = std::min((size_t)std::thread::hardware_concurrency(), action->getNumSolutions());
    ConcurrentExecutor predictExecutor(maxThreads);

    // enque each solution to be predicted
    for (size_t i = 0; i < action->getNumSolutions(); ++i)
    {
      const ActionBase* aPtr = action.get();

      futures.push_back(predictExecutor.enqueue([i, this, aPtr, &predictions]
      {
        auto localAction = aPtr->clone();
        localAction->initialize(domain, graph, i);
        RLOGS(1, "Starting prediction %zu from %zu: %s", i, localAction->getNumSolutions() - 1,
              localAction->getActionCommand().c_str());
        double dt_predict = Timer_getSystemTime();
        predictions[i] = localAction->predict(domain, graph, broadphase, localAction->getDuration(),
                                              getEntity()->getDt(), earlyExitPrediction);
        predictions[i].idx = i;
        dt_predict = Timer_getSystemTime() - dt_predict;
        predictions[i].feedbackMsg.developer += " command: " + localAction->getActionCommand();
        RLOGS(1, "%s for prediction %zu from %zu",
              (predictions[i].success ? "SUCCESS" : "FAILURE"), i, localAction->getNumSolutions()-1);
        RLOGS(2, "[%s] Action \"%s\" try %zu: took %.1f msec, jlCost=%f, collCost=%f, actionCost=%f\n\tMessage: %s",
              predictions[i].success ? "SUCCESS" : "FAILURE", localAction->getName().c_str(), i,
              1.0e3 * dt_predict, predictions[i].jlCost, predictions[i].collCost, predictions[i].actionCost,
              predictions[i].feedbackMsg.toString().c_str());
      }));
    }

    // wait for the predictions to finish
    size_t count = 0;
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
      RLOG_CPP(1, "Starting single-threaded prediction " << i << " from " << action->getNumSolutions());
      action->initialize(domain, graph, i);
      double dt_predict = Timer_getSystemTime();
      predictions[i] = action->predict(domain, graph, broadphase, action->getDuration(),
                                       getEntity()->getDt(), earlyExitPrediction);
      predictions[i].idx = i;
      dt_predict = Timer_getSystemTime() - dt_predict;
      RLOG(1, "[%s] Action \"%s\" try %zu: took %.1f msec, jlCost=%f, collCost=%f\n\tMessage: %s",
           predictions[i].success ? "SUCCESS" : "FAILURE", action->getName().c_str(), i,
           1.0e3 * dt_predict, predictions[i].jlCost, predictions[i].collCost,
           predictions[i].feedbackMsg.toString().c_str());
    }
  }

  RLOG_CPP(1, "Sorting " << predictions.size() << " predictions");
  std::sort(predictions.begin(), predictions.end(), TrajectoryPredictor::PredictionResult::lesser);

  REXEC(0)
  {
    RLOG_CPP(0, "Printing predictions");
    for (const auto& r : predictions)
    {
      if (r.idx != -1)
      {
        const int verbosityLevel = 2;
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
  //   getEntity()->publish("ActionResult", false, 0.0, predResults[0].message (Fixme to FeedbackMessage));
  //   return;
  // }

  // We initialize the action with the best prediction that was found.
  RLOG_CPP(0, "Initializing with solution " << predictions[0].idx);
  action->initialize(domain, graph, predictions[0].idx);

  REXEC(1)
  {
    action->toXML("action.xml");
  }

  // Debug visualization.
  getEntity()->publish("AnimateSequence", predictions, 1);







#if defined (SMILEACTIONS_WITH_OPTIM)
  bool optimizeMe = false;

  if (optimizeMe)
  {
    RLOG(1, "Starting optimization");
    std::vector<double> u = optimize(action.get(), graph,
                                     5.0*getEntity()->getDt(), "Power", 100);
  }
#endif



  // \todo(MG): HACK for foveated objects. We currently do this here, since the
  // actions only receive const references to the affordance models ans scenes.
  // For multi-agent settings, we also should consider several foveated items etc.
  ActionGaze* aGaze = dynamic_cast<ActionGaze*>(action.get());
  if (aGaze)
  {
    // We ignore the action request if no valid prediction was found. We don't
    // necessarily need to do this here, since there is a final check in the
    // TrajectoryComponent. This is a bit more accessible, since we can "see"
    // the trajectory with the 'd' key.
    if (predictions.empty() || (!predictions[0].success))
    {
      std::vector<ActionResult> errMsg(1);
      if (predictions.empty())
      {
        errMsg[0].error = "ERROR";
        errMsg[0].reason = "Gaze action could not find solution";
        errMsg[0].developer = std::string(__FILENAME__) + " line " + std::to_string(__LINE__);
      }
      else
      {
        errMsg[0] = predictions[0].feedbackMsg;
      }

      getEntity()->publish("ActionResult", false, 0.0, errMsg);
      return;
    }

    domain.foveatedEntity = aGaze->getGazeTarget();
    RLOG_CPP(0, "Now gazing at " << domain.foveatedEntity);
    getEntity()->publish("PtuLookAtCommand", domain.foveatedEntity);
    // return;
    // We let the function continue here so that the graphics is reflecting
    // the PTU motion. It might not be the same as the one that really happens.
  }




  std::vector<std::string> taskVec = action->createTasksXML();

  if (taskVec.empty())
  {
    std::vector<ActionResult> errMsg(1);
    errMsg[0].error = "Found invalid tasks for action";
    errMsg[0].reason = "That's an error in the program";
    errMsg[0].suggestion = "File a bug report for my program.";
    errMsg[0].developer = "Action command: " + action->getActionCommand();
    getEntity()->publish("ActionResult", false, 0.0, errMsg);
    return;
  }

  // We need to publish the new task vector and the trajectory. The new
  // trajectory is started a few time steps in the future so that the
  // step function does properly initialize the last few values for a
  // smooth initialization.
  const double delay = 5.0*getEntity()->getDt();
  auto tSet = action->createTrajectory(delay, action->getDuration()+delay);

  // From here on, the action will start going.
  if (!startingFinalPose)
  {
    getEntity()->publish("FreezePerception", true);
  }
  getEntity()->publish("ChangeTaskVector", taskVec, action->getManipulators());
  getEntity()->publish("CheckAndSetTrajectory", tSet);
}

const ActionScene* ActionComponent::getScene() const
{
  return &domain;
}

ActionScene* ActionComponent::getScene()
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

void ActionComponent::setFinalPoseRunning(bool enable)
{
  startingFinalPose = enable;
}

bool ActionComponent::isFinalPoseRunning() const
{
  return startingFinalPose;
}

void ActionComponent::setEarlyExitPrediction(bool enable)
{
  this->earlyExitPrediction = enable;
}

bool ActionComponent::getEarlyExitPrediction() const
{
  return this->earlyExitPrediction;
}

}   // namespace aff
