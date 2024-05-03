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
#include "PredictionTree.h"

#include <TaskFactory.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_basicMath.h>
#include <Rcs_body.h>
#include <Rcs_timer.h>
#include <Rcs_parser.h>

#include <ctype.h>
#include <algorithm>




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

bool ActionBase::defaultTurbo = false;

void ActionBase::setTurboMode(bool enable)
{
  defaultTurbo = enable;
}

ActionBase::ActionBase() : defaultDuration(10.0), turbo(defaultTurbo)
{
}

ActionBase::~ActionBase()
{
}

void ActionBase::parseParams(std::vector<std::string>& params)
{

  auto it = std::find(params.begin(), params.end(), "duration");
  if (it != params.end())
  {
    defaultDuration = std::stod(*(it + 1));
    params.erase(it + 1);
    params.erase(it);
  }

  it = std::find(params.begin(), params.end(), "fast");
  if (it != params.end())
  {
    turbo = true;
    params.erase(it);
  }

}

double ActionBase::getDurationHint() const
{
  return defaultDuration;
}

bool ActionBase::turboMode() const
{
  return turbo;
}

void ActionBase::setDuration(double duration)
{
  defaultDuration = duration;
}
double ActionBase::actionCost(const ActionScene& domain,
                              const RcsGraph* graph) const
{
  return 0.0;
}

size_t ActionBase::addTasks(Rcs::ControllerBase* controller) const
{
  std::vector<std::string> xmlTask = createTasksXML();
  auto tasks = Rcs::TaskFactory::createTasks(xmlTask, controller->getGraph());

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
                                                          double dt,
                                                          bool earlyExit) const
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
  RLOG(5, "Graph cloning took %.2f msec", 1.0e3 * t_clone);

  addTasks(&controller);
  auto tc = std::make_unique<tropic::TrajectoryController<tropic::ViaPointTrajectory1D>>(&controller, 1.0);
  const double delay = 5.0*dt;
  auto tSet = createTrajectory(delay, duration+delay);
  aff::TrajectoryPredictor pred(tc.get());
  pred.setTrajectory(tSet);   // also clears it

  // Perform the actual prediction
  t_clone = Timer_getSystemTime();
  aff::TrajectoryPredictor::PredictionResult result = pred.predict(dt, earlyExit);

  // Add an action-specific cost. It is 0 per default, and can be used by
  // actions to bias the solution.
  result.actionCost = actionCost(scene, graph);

  t_clone = Timer_getSystemTime() - t_clone;
  RLOG(4, "Prediction took %.2f msec", 1.0e3 * t_clone);

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

  REXEC(4)
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

void ActionBase::setName(const std::string& name)
{
  actionName = name;
}

void ActionBase::setActionParams(const std::vector<std::string>& params)
{
  actionParams = params;
}

std::vector<std::string> ActionBase::getActionParams() const
{
  return actionParams;
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
  std::cout << "Turbo mode: " << (turbo ? "ON" : "OFF") << std::endl;
  std::cout << "Action command: " << getActionCommand() << std::endl;
  std::cout << "Action has " << getNumSolutions() << " solutions" << std::endl;
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

std::vector<std::string> ActionBase::planActionSequence(ActionScene& domain,
                                                        RcsGraph* graph,
                                                        const RcsBroadPhase* broadphase,
                                                        std::vector<std::string> actions,
                                                        size_t stepsToPlan,
                                                        size_t maxNumThreads,
                                                        double dt,
                                                        bool earlyExit,
                                                        std::string& errMsg)
{
  auto tree = PredictionTree::planActionTree(domain, graph, broadphase, actions,
                                             stepsToPlan, maxNumThreads, dt, earlyExit, errMsg);

  std::vector<std::string> predictedActions;  // Action sequence which will be returned

  if (tree)
  {
    auto sln = tree->findSolutionPath();
    for (const auto& nd : sln)
    {
      predictedActions.push_back(nd->actionCommand());
      RLOG_CPP(0, "Action: " << nd->actionCommand() <<
               " cost: " << nd->cost);
    }
  }

  return predictedActions;
}



}   // namespace aff
