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

#ifndef AFF_TRAJECTORYPREDICTOR_H
#define AFF_TRAJECTORYPREDICTOR_H

#include <TrajectoryController.h>
#include <IkSolverRMR.h>

#include <iostream>


namespace aff
{
class TrajectoryPredictor
{
public:

  struct PredictionResult
  {
    PredictionResult(): idx(-1), success(false), minDist(0.0), jlCost(0.0), collCost(0.0), actionCost(0.0), elbowNS(0.0), wristNS(0.0), graph(nullptr)
    {
    }

    double cost() const
    {
      // Both terms are normalized between 0 and 1, therefore only hald the sum.
      return (jlCost + collCost + 8.0 * actionCost) / 10.0;
    }

    // The lesser function for sorting a vector of results. The failure -
    // success comparisons ensure that the first-ranked solutions are valid.
    static bool lesser(const PredictionResult& a, const PredictionResult& b)
    {
      if (a.success && !b.success)
      {
        return true;
      }

      if (!a.success && b.success)
      {
        return false;
      }

      return a.cost() < b.cost();
    }

    void print(int verbosityLevel=1) const
    {
      if (verbosityLevel<0)
      {
        return;
      }

      std::cout << "[" << __FILE__ << ": " << __FUNCTION__ << "("  << __LINE__ << ")]: ";
      std::cout << "PredictionResult " << std::to_string(idx) << ": ";

      if (success)
      {
        std::cout << "SUCCESS ";
      }
      else
      {
        std::cout << "FAILURE ";
      }

      if (verbosityLevel == 1)
      {
        std::cout << std::endl;
        std::cout << "minDist: " << minDist << std::endl;
        std::cout << "jlCost: " << jlCost << std::endl;
        std::cout << "collCost: " << collCost << std::endl;
        std::cout << "actionCost: " << actionCost << std::endl;
        std::cout << "cost: " << cost() << std::endl;
        std::cout << "message: " << message << std::endl;
        std::cout << "minDistBdy1: " << minDistBdy1 << std::endl;
        std::cout << "minDistBdy2: " << minDistBdy2 << std::endl;
        std::cout << "jMask: " << std::endl;
        for (size_t i = 0; i < jMask.size(); ++i)
        {
          std::cout << jMask[i];
        }
      }
      else
      {
        std::cout << "cost: " << cost();
      }
      std::cout << std::endl;
    }

    int idx;   // Store the index if inside a vector. Not so good.
    bool success;
    double minDist;
    double jlCost;
    double collCost;
    double actionCost;
    double elbowNS, wristNS;
    std::string message;
    std::string minDistBdy1, minDistBdy2;
    std::vector<double> jMask;
    std::vector<double> optimizationParameters;
    std::vector<double> bodyTransforms;
    std::vector<std::string> taskVec;
    std::string actionText;
    RcsGraph* graph;   // Will be passed to PredictionTreeNode and deleted there. \todo(MG)
  };

  /*! \brief Constructs class with TrajectoryController instance cloned from
   *         the passed controller.
   */
  TrajectoryPredictor(const tropic::TrajectoryControllerBase* controller);

  /*! \brief Destroys the instance and frees all internal memory.
   */
  virtual ~TrajectoryPredictor();

  /*! \brief Applies a deep copy of tSet to the class's internal trajectory
   *         controller. Before that, all trajectories are cleared.
   */
  void setTrajectory(tropic::TCS_sptr tSet);

  PredictionResult predict(double dt);
  void getPredictionArray(MatNd* tPred) const;
  bool check(bool jointLimits=true, bool collisions=true,
             bool speedLimits=true) const;
  void clearTrajectory();

  /*! \brief IK-Step with FK. Upon success, the grapg's state is set to the next
   *         state according to the IK command. In case of failure, the graph's
   *         state remains unchanged.
   *
   *  \return 0: success, -1: singular IK, -2: speed limit violation,
   *          -3: joint limit violation, -4: collision
   */
  static int computeIK(Rcs::IkSolverRMR* solver, const MatNd* a, const MatNd* x,
                       double dt, double alpha, double lambda,
                       double qFilt, double phase, bool speedLimitCheck, bool jointLimitCheck,
                       bool collisionCheck, bool withSpeedAccLimit,
                       bool verbose, MatNd* jMask, std::string& resMsg);

  tropic::TrajectoryControllerBase* tc;
  Rcs::IkSolverRMR* ikSolver;
  int predSteps;

private:

  MatNd* tStack;

  void initFromState(const MatNd* q, const MatNd* q_dot = NULL);

  /*! \brief Adds a null space penalty to dH to move the elbows away from the body
   */
  static void addElbowNullspace(const RcsGraph* graph, MatNd* dH);

  /*! \brief Adds a null space penalty to dH to move the wrists in front of the body
   */
  static void addWristNullspace(const RcsGraph* graph, MatNd* dH);

  static int checkState(const Rcs::ControllerBase* controller,
                        bool speedLimitCheck, bool jointLimitCheck,
                        bool collisionCheck, bool verbose,
                        std::string& resMsg);

  TrajectoryPredictor(const TrajectoryPredictor&);
  TrajectoryPredictor& operator=(const TrajectoryPredictor&);
};

}

#endif   // AFF_TRAJECTORYPREDICTOR_H
