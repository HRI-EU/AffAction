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

#ifndef AFF_ACTIONPROBLEM_H
#define AFF_ACTIONPROBLEM_H

#include <ActionBase.h>

#include <Minimizer.h>
#include <OptimizationProblem.h>

#include <Rcs_macros.h>


namespace aff
{

class ActionProblem : public Rcs::OptimizationProblem
{
public:

  ActionProblem(ActionBase* action_,
                const RcsGraph* graph_,
                double dt_step) :
    action(action_), graph(graph_), dt(dt_step)
  {
  }

  virtual double cost(const double* x)
  {
    std::vector<double> u(x, x+action->getOptimDim());
    action->setOptimState(u);
    auto res = action->predict(graph, action->getDurationHint(), dt);
    double cost = 0.0 * res.jlCost + 1.0 * res.collCost;
    RLOG(1, "Cost: %f", cost);
    return cost;
  }

private:
  ActionBase* action;
  const RcsGraph* graph;
  double dt;
};

std::vector<double> optimize(ActionBase* action,
                             const RcsGraph* graph,
                             double dt_step,
                             const std::string& minimizerName,
                             int maxIter)
{
  Rcs::Minimizer::info();

  Rcs::OptimizationProblem* p = new ActionProblem(action, graph, dt_step);
  action->setOptimState(std::vector<double>());   // Remove optimization parameters
  auto res = action->predict(graph, action->getDurationHint(), dt_step);

  // Now we have the optimization parameters in res and store them in the action.
  // That's because predict() is const and we shouldn't change the class instance
  // inside it.
  action->setOptimState(res.optimizationParameters);

  // This is the dimension we have to assign to the problem
  p->setDim(action->getOptimDim());
  RLOG_CPP(0, "Created ActionProblem of dimension " << p->getDim());

  std::vector<double> u = action->getOptimState();

  int iterCounter = 0;
  double cost0 = p->cost(u.data());
  RLOG(0, "Initial cost: %g", cost0);

  Rcs::Minimizer* minimizer = Rcs::Minimizer::create(p, minimizerName.c_str());
  minimizer->maxIter = maxIter;
  minimizer->minimizeSelf(u.data());
  RLOG(0, "Finished minimization");

  double cost1 = p->cost(u.data());
  RLOG(0, "Final cost %g (initial: %f)\n", cost1, cost0);

  delete minimizer;
  delete p;

  return u;
}


}   // namespace

#endif   // AFF_ACTIONPROBLEM_H
