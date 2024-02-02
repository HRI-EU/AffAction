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

#include "ConcurrentSceneQuery.h"
#include "SceneJsonHelpers.h"
#include "ExampleActionsECS.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_timer.h>



namespace aff
{

ConcurrentSceneQuery::ConcurrentSceneQuery(const ExampleActionsECS* sim_) :
  sim(sim_), graph(NULL), broadphase(NULL)
{
  RCHECK(sim);
  std::lock_guard<std::mutex> lock(sim->stepMtx);
  graph = RcsGraph_clone(sim->controller->getGraph());
  scene = *(sim->getScene());
}

ConcurrentSceneQuery::~ConcurrentSceneQuery()
{
  RcsGraph_destroy(graph);
  RcsBroadPhase_destroy(this->broadphase);
}

void ConcurrentSceneQuery::update(bool withBroadphase)
{
  std::lock_guard<std::mutex> lock(sim->stepMtx);
  RcsGraph_copy(this->graph, sim->controller->getGraph());
  this->scene = *(sim->getScene());

  if (withBroadphase)
  {
    RcsBroadPhase_destroy(this->broadphase);
    this->broadphase = RcsBroadPhase_clone(sim->controller->getBroadPhase(), graph);
  }
}

nlohmann::json ConcurrentSceneQuery::getSceneState()
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  nlohmann::json json;
  aff::getSceneState(json, &scene, graph);
  return json;
}

nlohmann::json ConcurrentSceneQuery::getOccludedObjectsForAgent(const std::string& agentName)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  return aff::getOccludedObjectsForAgent(agentName, &scene, graph);
}

nlohmann::json ConcurrentSceneQuery::getObjectOccludersForAgent(const std::string& agentName,
                                                                const std::string& objectName)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  return aff::getObjectOccludersForAgent(agentName, objectName, &scene, graph);
}

bool ConcurrentSceneQuery::isAgentBusy(const std::string& agentName, double distanceThreshold)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  return aff::isAgentBusy(agentName, &scene, graph, distanceThreshold);
}

std::vector<std::string> ConcurrentSceneQuery::planActionSequence(const std::vector<std::string>& actions, size_t numStepsToPlan, size_t maxThreads)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update(true);
  std::string errMsg;
  auto res =  ActionBase::planActionSequence(scene, graph, broadphase, actions, numStepsToPlan, maxThreads, sim->entity.getDt(), errMsg);

  return res;
}

std::vector<double> ConcurrentSceneQuery::getPanTilt(const std::string& roboAgent,
                                                     const std::string& gazeTarget)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();

  const aff::Agent* robo_ = scene.getAgent(roboAgent);
  const aff::RobotAgent* robo = dynamic_cast<const aff::RobotAgent*>(robo_);
  if (!robo)
  {
    RLOG(0, "Robot agent '%s' not found", roboAgent.c_str());
    return std::vector<double>();
  }

  double panTilt[2], err[2];
  size_t maxIter = 100;
  double eps = 1.0e-3;   // 1mm

  double t_calc = Timer_getSystemTime();

  std::string resolvedTarget;

  // Resolve if gaze Target is a scene entity
  const AffordanceEntity* gazeNtt = scene.getAffordanceEntity(gazeTarget);
  if (gazeNtt)
  {
    resolvedTarget = gazeNtt->bdyName;
  }
  RLOG_CPP(1, "Checking AffordanceEntity: " << resolvedTarget);

  // If no scene entity is found with the gaze target, we look for agents. In this
  // case, we look at the agent's head.
  if (resolvedTarget.empty())
  {
    const Agent* gazeAgent = scene.getAgent(gazeTarget);
    if (gazeAgent)
    {
      auto m = gazeAgent->getManipulatorsOfType(&scene, "head");
      if (!m.empty())
      {
        resolvedTarget = m[0]->bdyName;
      }
    }
  }
  RLOG_CPP(1, "Checking Agents: " << resolvedTarget);

  // If no agent is found with the gaze target, we look for manipulators
  if (resolvedTarget.empty())
  {
    const Manipulator* m = scene.getManipulator(gazeTarget);
    if (m)
    {
      resolvedTarget = m->bdyName;
    }
  }
  RLOG_CPP(1, "Checking Manipulators: " << resolvedTarget);


  // If none of these was found, we pass it straight through and hope it is a RcsBody name
  if (resolvedTarget.empty())
  {
    resolvedTarget = gazeTarget;
  }
  RLOG_CPP(1, "Before pan-tilt: " << resolvedTarget);

  // This calls the actual pan-tilt calculation
  int iter = robo->getPanTilt(graph, resolvedTarget, panTilt, maxIter, eps, err);

  // Can't retrieve joints / bodies etc.
  if (iter==-1)
  {
    RLOG(0, "Pan tilt computation failed due to missing joints / bodies / graph");
    return std::vector<double>();
  }

  // Couldn't converge to precision given in eps
  if ((err[0]>eps) || (err[1]>eps))
  {
    RLOG(0, "Pan tilt computation did not converge for gaze Target '%s' (resolvedTarget: '%s')",
         gazeTarget.c_str(), resolvedTarget.c_str());
    const RcsBody* gazeBdy = RcsGraph_getBodyByName(graph, resolvedTarget.c_str());
    if (gazeBdy)
    {
      RLOG(0, "Goal position: %f %f %f", gazeBdy->A_BI.org[0], gazeBdy->A_BI.org[1], gazeBdy->A_BI.org[2]);
    }
    else
    {
      RLOG(0, "Can't print coordinates - RcsBody not found");
    }

    return std::vector<double>();
  }

  t_calc = Timer_getSystemTime() - t_calc;

  RLOG(1, "pan=%.1f tilt=%.1f err=%f %f took %d iterations and %.1f msec",
       RCS_RAD2DEG(panTilt[0]), RCS_RAD2DEG(panTilt[1]), err[0], err[1], iter, 1.0e3*t_calc);

  return std::vector<double>(panTilt, panTilt+2);
}

}   // namespace aff
