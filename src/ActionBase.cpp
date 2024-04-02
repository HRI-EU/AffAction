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

#include "ActionBase.h"

// For prediction tree
#include <ConcurrentExecutor.h>
#include <PredictionTree.h>
#include <ActionFactory.h>
#include <Rcs_utilsCPP.h>
#include <algorithm>


#include <TaskFactory.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_basicMath.h>
#include <Rcs_body.h>
#include <Rcs_timer.h>
#include <Rcs_parser.h>

#include <ctype.h>




namespace aff
{

const RcsBody* ActionBase::resolveBodyName(const RcsGraph* graph,
                                           std::string& bdyName)
{
  // Rapid exit in this case
  if (bdyName.empty())
  {
    return NULL;
  }

  const RcsBody* bdy = RcsGraph_getBodyByNameNoCase(graph, bdyName.c_str());

  if (!bdy)
  {
    RLOG(1, "Failed to find graph body for \"%s\"", bdyName.c_str());
    return NULL;
  }

  bdyName = std::string(bdy->name);

  return bdy;
}

ActionBase::ActionBase() : defaultDuration(10.0)
{
}

ActionBase::~ActionBase()
{
}

double ActionBase::getDurationHint() const
{
  return defaultDuration;
}

size_t ActionBase::addTasks(Rcs::ControllerBase* controller) const
{
  std::vector<std::string> xmlTask = createTasksXML();
  auto tasks = Rcs::TaskFactory::createTasks(xmlTask, controller->getGraph());
  // std::vector<Rcs::Task*> tasks = createTasks(controller->getGraph());

  // Check that all constructed tasks are valid
  size_t nTasksAdded = 0;
  for (size_t i = 0; i < tasks.size(); ++i)
  {
    bool taskAlreadyExists = false;
    for (size_t j = 0; j < controller->getNumberOfTasks(); ++j)
    {
      if (controller->getTaskName(j) == tasks[i]->getName())
      {
        taskAlreadyExists = true;
      }
    }

    if (!taskAlreadyExists)
    {
      controller->add(tasks[i]);
      nTasksAdded++;
    }
  }

  return nTasksAdded;
}

TrajectoryPredictor::PredictionResult ActionBase::predict(ActionScene& scene,
                                                          const RcsGraph* graph_,
                                                          const RcsBroadPhase* broadphase,
                                                          double duration,
                                                          double dt) const
{
  // Cloning graph and reading collision model takes approximately 20msec.
  double t_clone = Timer_getSystemTime();
  RcsGraph* graph = RcsGraph_clone(graph_);
  Rcs::ControllerBase controller(graph);   // Takes ownership of graph

  // Extract the collision model
  RcsBroadPhase* bp = RcsBroadPhase_clone(broadphase, graph);
  RcsBroadPhase_updateBoundingVolumes(bp);
  controller.setBroadPhase(bp);
  RcsCollisionMdl* cMdl = RcsCollisionModel_create(graph);
  controller.setCollisionMdl(cMdl);

  t_clone = Timer_getSystemTime() - t_clone;
  RLOG(1, "Graph cloning took %.2f msec", 1.0e3 * t_clone);

  addTasks(&controller);
  auto tc = std::make_unique<tropic::TrajectoryController<tropic::ViaPointTrajectory1D>>(&controller, 1.0);
  const double delay = 5.0*dt;
  auto tSet = createTrajectory(delay, duration+delay);
  aff::TrajectoryPredictor pred(tc.get());
  pred.setTrajectory(tSet);   // also clears it

  // Perform the actual prediction
  t_clone = Timer_getSystemTime();
  aff::TrajectoryPredictor::PredictionResult result = pred.predict(dt);

  // Add an action-specific cost. It is 0 per default, and can be used by
  // actions to bias the solution.
  result.actionCost = actionCost(scene, graph);
  t_clone = Timer_getSystemTime() - t_clone;
  RLOG(1, "Prediction took %.2f msec", 1.0e3 * t_clone);

#if 0
  // const ActionGet* ag = dynamic_cast<const ActionGet*>(this);
  // if (ag)
  {
    result.optimizationParameters = getInitOptimState(tc.get(), duration);
    REXEC(1)
    {
      VecNd_printComment("O-params: ",
                         result.optimizationParameters.data(),
                         result.optimizationParameters.size());
    }
  }
#endif

  // Determine which end effectors have been used. This is the case if the
  // joint mask has non-zero entries at indices of the mask computed by the
  // joint recursion
  // std::vector<std::string> usedEffectors;
  // for (const auto& m : domain.manipulators)
  // {
  //   MatNd* tmpMask = MatNd_create(graph->nJ, 1);
  //   const RcsBody* effector = RcsGraph_getBodyByName(graph, m.bdyName.c_str());
  //   RCHECK(effector);
  //   RcsGraph_computeJointRecursionMask(graph, effector, tmpMask);
  //   for (size_t i = 0; i < graph->nJ; ++i)
  //   {
  //     if ((tmpMask->ele[i] > 0.0) && (result.jMask[i] > 0.0))
  //     {
  //       usedEffectors.push_back(m.name);
  //       break;
  //     }
  //   }
  //   MatNd_destroy(tmpMask);
  // }
  // result.print();

  // RLOG_CPP(0, "These effectors are used: ");
  // for (const auto& ee : usedEffectors)
  // {
  //   RLOG_CPP(0, ee);
  // }

  REXEC(1)
  {
    RMSG("Prediction result:");
    result.print();
  }

  return result;
}

tropic::TCS_sptr ActionBase::createTrajectory() const
{
  return createTrajectory(0.0, getDurationHint());
}

std::string ActionBase::explain() const
{
  return getActionCommand();
}

void ActionBase::setName(const std::string& name)
{
  actionName = name;
}

void ActionBase::setActionParams(const std::vector<std::string>& params)
{
  actionParams = params;
}

std::string ActionBase::getName() const
{
  return actionName;
}

std::string ActionBase::getActionCommand() const
{
  std::string cmd = actionName;

  for (const auto& p : actionParams)
  {
    cmd += " ";
    cmd += p;
  }

  return cmd;
}

void ActionBase::print() const
{
  auto xmlTasks = createTasksXML();

  for (const auto& t : xmlTasks)
  {
    std::cout << t << std::endl;
  }

  std::cout << "Action default duration: " << getDurationHint() << std::endl;
}

std::vector<double> ActionBase::getOptimState() const
{
  return optimState;
}

void ActionBase::setOptimState(std::vector<double> state)
{
  optimState = state;
}

size_t ActionBase::getOptimDim() const
{
  return getOptimState().size();
}

std::vector<double> ActionBase::getInitOptimState(tropic::TrajectoryControllerBase* tc,
                                                  double duration) const
{
  return std::vector<double>();
}

bool ActionBase::initialize(const ActionScene& domain,
                            const RcsGraph* graph,
                            size_t solutionRank)
{
  return true;
}

size_t ActionBase::getNumSolutions() const
{
  return 1;
}

const AffordanceEntity* ActionBase::raycastSurface(const ActionScene& domain,
                                                   const AffordanceEntity* ntt,
                                                   const RcsGraph* graph,
                                                   HTr* surfTransform,
                                                   std::string& errMsg) const
{
  // Check if the body to drop exists
  const RcsBody* body = RcsGraph_getBodyByNameNoCase(graph, ntt->bdyName.c_str());
  if (!body)
  {
    errMsg = "ERROR: Failed to find " + ntt->bdyName + " to search surface below";
    return NULL;
  }

  // First we compute an axis-aligned bounding box around the body. We raycast
  // from its bottom. This avoids self-intersections with the object.
  double xyzMin[3], xyzMax[3], castFrom[3], dir[3], surfPt[3], dMin = 0.0;
  RcsGraph_computeBodyAABB(graph, body->id, RCSSHAPE_COMPUTE_DISTANCE, xyzMin, xyzMax, NULL);
  Vec3d_set(castFrom, 0.5*(xyzMin[0]+xyzMax[0]), 0.5*(xyzMin[1]+xyzMax[1]), xyzMin[2]-1.0e-8);
  Vec3d_set(dir, 0.0, 0.0, -1.0);
  const RcsBody* surfaceBdy = RcsBody_closestRigidBodyInDirection(graph, castFrom, dir, surfPt, &dMin);

  if (!surfaceBdy)
  {
    errMsg = "ERROR: no surface to drop on found under object " + ntt->name + " DEVELOPER: RcsBody not found";
    return NULL;
  }

  const AffordanceEntity* surface = domain.getAffordanceEntity(surfaceBdy->name);
  if (!surface)
  {
    errMsg = "ERROR: no surface to drop on found under object " + ntt->name + " DEVELOPER: AffordanceEntity not found";
    return NULL;
  }

  if (surfTransform)
  {
    HTr_copy(surfTransform, &body->A_BI);
    surfTransform->org[2] -= dMin;
  }

  return surface;
}



//agent.sim.planActionSequence("get fanta_bottle;put fanta_bottle lego_box; pose default")
std::vector<std::string> ActionBase::planActionSequence(ActionScene& domain,
                                                        RcsGraph* graph,
                                                        const RcsBroadPhase* broadphase,
                                                        std::vector<std::string> actions,
                                                        size_t stepsToPlan,
                                                        size_t maxNumThreads,
                                                        double dt,
                                                        std::string& errMsg)
{
  std::vector<std::string> predictedActions;  // Action sequence which will be returned
  std::unique_ptr<PredictionTree> predictionTree = std::make_unique<PredictionTree>();
  PredictionTreeNode* currentPredictionNode = predictionTree->root;
  RcsGraph* localGraph = RcsGraph_clone(graph); // Create a local copy of the graph to work with
  const RcsGraph* lookaheadGraph = localGraph;

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

    for (const auto& currentPredictionNode : currentStepNodes)
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
      if (actionStrings.size()>1)
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
      if ((!action) || (action->getNumSolutions()==0))
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

        futures.push_back(predictExecutor.enqueue([i, &domain, broadphase, lookaheadGraph, aPtr, dt, &predResults]
        {
          const double scaleDurationHint = 1.0;
          auto localAction = aPtr->clone();
          RLOG_CPP(1, "Starting prediction " << i+1 << " from " << localAction->getNumSolutions());
          localAction->initialize(domain, lookaheadGraph, i);
          double dt_predict = Timer_getSystemTime();
          predResults[i] = localAction->predict(domain, lookaheadGraph, broadphase,
                                                scaleDurationHint*localAction->getDurationHint(), dt);
          predResults[i].idx = i;
          predResults[i].actionText = localAction->getActionCommand();
          dt_predict = Timer_getSystemTime() - dt_predict;
          predResults[i].message += " command: " + localAction->getActionCommand();
          RLOG(1, "[%s] Action \"%s\" try %zu: took %.1f msec, jlCost=%f, collCost=%f\n\tMessage: %s",
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
          RLOG(1, "Adding: `%s`", r.actionText.c_str());
          currentPredictionNode->addChild(r);
        }
      }
      RLOG_CPP(1, predResults.size() << " predictions have been added to the prediction tree");
    }   // for (const auto& currentPredictionNode : currentStepNodes)

  }   // for (size_t s = 0; s < stepsToPlan; s++)

  REXEC(0)
  {
    predictionTree->printTreeVisual(predictionTree->root, 0);
  }

  const int treeDepth = predictionTree->getMaxDepth() - 1; //exclude root

  RLOG(1, "Finally destroying graph '%s'", localGraph->cfgFile);
  RcsGraph_destroy(localGraph);

  if (treeDepth < stepsToPlan)
  {
    RLOG(0, "Cannot predict a successfull path for the provided sequence! Failure encountered after %d actions.", treeDepth);
    return predictedActions;
  }
  else
  {
    std::pair<double, std::vector<aff::PredictionTreeNode*>> bestPath = predictionTree->findSmallestCostPath(treeDepth);

    RLOG(1, "Best route for the next %d actions: ", treeDepth);

    for (const auto& node : bestPath.second)
    {
      RLOG_CPP(1, node->actionText << " : " << node->quality);
      predictedActions.push_back(node->actionText);
    }

    RLOG_CPP(1, "best path total cost: " << bestPath.first);
  }

  REXEC(0)
  {
    RLOG(0, "\nCorrected actions: ");
    for (const auto& action : predictedActions)
    {
      std::cout << action << ";";
    }
    std::cout << std::endl;
  }

  return predictedActions;
}

double ActionBase::actionCost(const ActionScene& domain,
                              const RcsGraph* graph) const
{
  return 0.0;
}



}   // namespace aff
