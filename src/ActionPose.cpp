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

#include <tuple>



namespace aff
{
REGISTER_ACTION(ActionPose, "pose");

ActionPose::ActionPose(const ActionScene& domain,
                       const RcsGraph* graph,
                       std::vector<std::string> params) :
  ActionPose(graph, params[0], params.size() > 1 ? std::stoi(params[1]) : 0)
{
  for (const auto& m : domain.manipulators)
  {
    usedManipulators.push_back(m.name);
  }
}


ActionPose::ActionPose(const RcsGraph* graph,
                       const std::string& mdlStateName,
                       int timeStamp)
{
  MatNd* q = MatNd_create(graph->dof, 1);
  MatNd_setElementsTo(q, DBL_MAX);
  bool success = RcsGraph_getModelStateFromXML(q, graph, mdlStateName.c_str(), timeStamp);

  if (!success)
  {
    MatNd_destroy(q);
    throw ActionException("ERROR REASON: Could not find pose DEVELOPER: Model state with name not found: " + mdlStateName, ActionException::ParamNotFound);
  }

  RCSGRAPH_FOREACH_JOINT(graph)
  {
    const double q_des_i = MatNd_get(q, JNT->jointIndex, 0);
    const double q_curr_i = MatNd_get(graph->q, JNT->jointIndex, 0);

    if (JNT->constrained || (q_des_i==DBL_MAX))
    {
      continue;
    }

    // jnts.push_back(std::pair<std::string,double>(std::string(JNT->name), q_des_i));
    jnts.push_back(std::make_tuple(std::string(JNT->name), q_curr_i, q_des_i));
  }


  MatNd_destroy(q);

  explanation = "Moving to pose " + mdlStateName;
}

ActionPose::~ActionPose()
{
}

std::vector<std::string> ActionPose::createTasksXML() const
{
  std::vector<std::string> tasks;

  for (const auto& ji : jnts)
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

  for (const auto& ji : jnts)
  {
    std::string jntName = std::get<0>(ji);
    double q_curr = std::get<1>(ji);
    double q_des = std::get<2>(ji);

    a1->addActivation(t_start, true, 0.5, jntName);
    a1->addActivation(t_end+0.5, false, 0.5, jntName);
    //a1->add(t_start, q_curr, 0.0, 0.0, 7, jntName + " 0");
    a1->add(t_end, q_des, 0.0, 0.0, 7, jntName + " 0");
    // We don't deactivate the tasks so that the pose persists
  }



  return a1;
}

std::string ActionPose::explain() const
{
  return explanation;
}

std::vector<std::string> ActionPose::getManipulators() const
{
  return usedManipulators;
}

std::unique_ptr<ActionBase> ActionPose::clone() const
{
  return std::make_unique<ActionPose>(*this);
}


}   // namespace aff
