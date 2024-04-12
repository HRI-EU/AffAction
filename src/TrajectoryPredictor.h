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
class ActionBase;

class TrajectoryPredictor
{
public:

  struct PredictionResult
  {
    PredictionResult();

    ~PredictionResult();

    double cost() const;

    // The lesser function for sorting a vector of results. The failure -
    // success comparisons ensure that the first-ranked solutions are valid.
    static bool lesser(const PredictionResult& a, const PredictionResult& b);

    void print(int verbosityLevel = 1) const;

    int idx;   // Store the index if inside a vector. Not so good.
    bool success;
    double minDist;
    double jlCost;
    double collCost;
    double actionCost;
    double scaleJointSpeeds;
    double elbowNS, wristNS;
    double t_predict;
    std::string message;
    std::string minDistBdy1, minDistBdy2;
    std::vector<double> jMask;
    std::vector<double> optimizationParameters;
    std::vector<double> bodyTransforms;
    RcsGraph* graph;   // Will be passed to PredictionTreeNode and deleted there. \todo(MG)
    ActionBase* action;
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

  PredictionResult predict(double dt, bool earlyExit=true);
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
