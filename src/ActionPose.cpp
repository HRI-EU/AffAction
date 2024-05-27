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

#include "ActionPose.h"
#include "ActionFactory.h"
#include "CollisionModelConstraint.h"

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <ConnectBodyConstraint.h>
#include <VectorConstraint.h>

#include <Rcs_typedef.h>
#include <Rcs_graphParser.h>
#include <Rcs_macros.h>
#include <Rcs_utilsCPP.h>

#include <tuple>
#include <cfloat>



namespace aff
{
REGISTER_ACTION(ActionPose, "pose");

ActionPose::ActionPose(const ActionScene& domain,
                       const RcsGraph* graph,
                       std::vector<std::string> params) : solutionIndex(-1)
{
  parseParams(params);

  // Get the time stamp value as the second argument after the pose name.
  // The default of -1 means that no time stamp needs to be given in the
  // model state description.
  std::string mdlStateName = params[0];
  int timeStamp = -1;

  if (params.size() > 1)
  {
    try
    {
      timeStamp = std::stoi(params[1]);
    }
    catch (const std::invalid_argument& e)
    {
      throw ActionException(ActionException::ParamInvalid, "Time stamp for pose " + mdlStateName + " is invalid.",
                            "Check how you typed the time stamp parameter - it should be an integer value",
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__) +
                            ": time stamp is not an integer value: " + params[1] + " (" + e.what() + ")");
    }
    catch (const std::out_of_range& e)
    {
      throw ActionException(ActionException::ParamInvalid, "Time stamp for pose " + mdlStateName + " is out of range.",
                            "Check how you typed the time stamp parameter - it should be within the range of integer values",
                            std::string(__FILENAME__) + " " + std::to_string(__LINE__) +
                            ": time stamp is not in the integer range: " + params[1] + " (" + e.what() + ")");
    }
  }

  // \todo(MG): This should be the agent's manipulators.
  for (const auto& m : domain.manipulators)
  {
    usedManipulators.push_back(m.name);
  }

  auto mdlNames = Rcs::String_split(mdlStateName, ",");

  for (const auto& mdl : mdlNames)
  {
    poses.push_back(createPose(graph, mdl, timeStamp));
  }


  explanation = "Moving to pose " + mdlStateName;
}

ActionPose::~ActionPose()
{
}

bool ActionPose::initialize(const ActionScene& domain, const RcsGraph* graph, size_t solutionRank)
{
  if (solutionRank >= poses.size())
  {
    return false;
  }

  solutionIndex = solutionRank;

  //if (turbo)
  //{
  //  RLOG_CPP(0, "Model state: " << poses[solutionIndex].name);
  //  double ratio = computeMaxVel(graph);
  //  defaultDuration *= (ratio + 0.01);
  // RLOG(0, "New duration: %f", defaultDuration);
  //}

  return true;
}

ActionPose::ModelPose ActionPose::createPose(const RcsGraph* graph,
                                             const std::string& mdlStateName,
                                             int timeStamp) const
{
  ModelPose pose;
  pose.name = mdlStateName;
  pose.timeStamp = timeStamp;

  MatNd* q = MatNd_create(graph->dof, 1);
  MatNd_setElementsTo(q, DBL_MAX);
  bool success = RcsGraph_getModelStateFromXML(q, graph, mdlStateName.c_str(), timeStamp);

  if (!success)
  {
    MatNd_destroy(q);
    std::vector<std::string> modelStates = Rcs::RcsGraph_getModelStateNames(graph);
    std::string modelStateString;
    for (size_t i = 0; i != modelStates.size(); ++i)
    {
      modelStateString += modelStates[i];
      if (i < modelStates.size() - 1)
      {
        modelStateString += ", ";
      }
    }
    throw ActionException(ActionException::ParamNotFound, "Could not find pose " + mdlStateName,
                          "Use one out of these: " + modelStateString + ".",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__) +
                          ": Model state with name not found: " + mdlStateName + " with time stamp "
                          + std::to_string(timeStamp));
  }

  RCSGRAPH_FOREACH_JOINT(graph)
  {
    const double q_des_i = MatNd_get(q, JNT->jointIndex, 0);

    if (JNT->constrained || (q_des_i == DBL_MAX))
    {
      continue;
    }

    pose.jnts.push_back(std::make_tuple(std::string(JNT->name), q_des_i));
  }

  MatNd_destroy(q);

  return pose;
}

std::vector<std::string> ActionPose::createTasksXML() const
{
  std::vector<std::string> tasks;

  for (const auto& ji : poses[solutionIndex].jnts)
  {
    std::string jntName = std::get<0>(ji);
    std::string xmlTask = "<Task name=\"" + jntName + "\" " +
                          "controlVariable=\"Joint\" jnt=\"" + jntName + "\" />";
    tasks.push_back(xmlTask);
  }

  return tasks;
}

tropic::TCS_sptr ActionPose::createTrajectory(double t_start, double t_end) const
{
  auto a1 = std::make_shared<tropic::ActivationSet>();

  for (const auto& ji : poses[solutionIndex].jnts)
  {
    std::string jntName = std::get<0>(ji);
    double q_des = std::get<1>(ji);

    a1->addActivation(t_start, true, 0.5, jntName);
    a1->addActivation(t_end+0.5, false, 0.5, jntName);
    a1->add(t_end, q_des, 0.0, 0.0, 7, jntName + " 0");
  }

  return a1;
}

std::vector<double> ActionPose::computeMaxVel(const RcsGraph* graph, double& maxVelRatio) const
{
  std::vector<double> maxVel;
  double buf[2 * 5];
  MatNd desc = MatNd_fromPtr(2, 5, buf);

  for (const auto& ji : poses[solutionIndex].jnts)
  {
    std::string jntName = std::get<0>(ji);
    double q_des = std::get<1>(ji);
    const RcsJoint* jnt = RcsGraph_getJointByName(graph, jntName.c_str());
    RCHECK_MSG(jnt, "%s", jntName.c_str());
    double q_curr = graph->q->ele[jnt->jointIndex];
    double qd_curr = graph->q_dot->ele[jnt->jointIndex];

    MatNd_set(&desc, 0, 0, 0.0);
    MatNd_set(&desc, 0, 1, q_curr);
    MatNd_set(&desc, 0, 2, qd_curr);
    MatNd_set(&desc, 0, 3, 0.0);
    MatNd_set(&desc, 0, 4, 7);

    MatNd_set(&desc, 1, 0, getDefaultDuration());
    MatNd_set(&desc, 1, 1, q_des);
    MatNd_set(&desc, 1, 2, 0.0);
    MatNd_set(&desc, 1, 3, 0.0);
    MatNd_set(&desc, 1, 4, 7);

    Rcs::ViaPointSequence seq(&desc);
    double t_vmax;
    double vmax = seq.getMaxVelocity(t_vmax);
    NLOG(0, "Joint %s: speed_limit: %f   max_speed: %f at t=%.3f (from %.3f)",
         jnt->name, jnt->speedLimit, vmax, t_vmax, defaultDuration);
    maxVel.push_back(vmax);
    maxVelRatio = std::max(maxVelRatio, vmax/jnt->speedLimit);
  }

  return maxVel;
}

std::vector<std::string> ActionPose::getManipulators() const
{
  return usedManipulators;
}

std::unique_ptr<ActionBase> ActionPose::clone() const
{
  return std::make_unique<ActionPose>(*this);
}

size_t ActionPose::getNumSolutions() const
{
  return poses.size();
}

std::string ActionPose::getActionCommand() const
{
  std::string aCmd = "pose " + poses[solutionIndex].name;

  if (poses[solutionIndex].timeStamp != -1)
  {
    aCmd += " " + std::to_string(poses[solutionIndex].timeStamp);
  }

  aCmd += " duration " + std::to_string(getDuration());

  return aCmd;
}

}   // namespace aff
