/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_TRAJECTORYCOMPONENT_H
#define RCS_TRAJECTORYCOMPONENT_H


#include "ComponentBase.h"
#include "TrajectoryPredictor.h"



namespace aff
{
/*! \brief Trajectory generation class. The trajectory is stepped within the
 *         ComputeTrajectory event.
 *
 *         The class publishes the following events:
 *         - TrajectoryMoving: Each time the trajectory finished or started,
 *                             this event publishes (true for started, false
 *                             for stopped)
 *         - SetBlending: Current blending value, published in each
 *                        trajectory step.
 *
 *         The class subscribes to the following events:
 *         - ClearTrajectory
 *         - EmergencyStop: Sets the internal eStop flag to true, and clears
 *                          the trajectory constraints. If the eStop flag is
 *                          set, no more trajectories are accepted by the
 *                          SetTrajectory event.
 *         - EmergencyRecover: Sets the internal eStop flag to false.
 *         - InitFromState: Sets the component's internal graph to the state
 *           published and initializes all trajectories. Constraints are all
 *           cleared.
 *         - ComputeTrajectory: Steps the trajectory with the entitie's time
 *                              step.
 *         - SetTrajectory: Applies the published constraints to the trajectory.
 *
 *  \todo: Re-think publishing SetBlending event in each step.
 */
class TrajectoryComponent : public ComponentBase
{
public:

  /*! \brief Constructs class with TrajectoryController instance cloned from
   *         the passed controller.
   *
   * \param[in] parent     Entity class responsible for event subscriptions
   * \param[in] controller Controller with all task variables and (possibly)
   *                       a collision model. It will be cloned.
   * \param[in] viaPtTrj   True for via-point polynomial trajectory, false for
   *                       zig-zag trajectory.
   * \param[in] horizon    Trajectory receding horizon length.
   */
  TrajectoryComponent(EntityBase* parent,
                      Rcs::ControllerBase* controller,
                      bool viaPtTrj=true,
                      double horizon=1.0,
                      bool checkTrajectory=true);

  /*! \brief Destroys the instance and frees all internal memory.
   */
  virtual ~TrajectoryComponent();

  /*! \brief Returns a reference to the classes internal activation vector.
   */
  const MatNd* getActivationPtr() const;

  /*! \brief Returns a reference to the classes internal trajectory command
   *         vector.
   */
  const MatNd* getTaskCommandPtr() const;

  /*! \brief Returns the motion end time of the trajectory generator. If no
   *         via points are added, it is 0.
   */
  double getMotionEndTime() const;

  const tropic::TrajectoryControllerBase* getTrajectoryController() const;

private:

  void stepTrajectory(double dt);
  void onEmergencyStop();
  void onEmergencyRecover();
  void onInitFromState(const RcsGraph* target);
  void onClearTrajectory();
  void onEnableTrajectoryCheck(bool enable);
  void onCheckAndSetTrajectory(tropic::TCS_sptr tSet);
  void onSetTrajectory(tropic::TCS_sptr tSet);
  void onSimulateTrajectory(tropic::TCS_sptr tSet);
  void onTaskVectorChangeSequential(std::vector<std::string> taskVec,
                                    std::vector<std::string> channels);
  void onStop();
  void onPrint();

  void checkerThread(tropic::TCS_sptr tSet, bool simulateOnly,
                     std::shared_ptr<TrajectoryPredictor> predictor);


  tropic::TrajectoryControllerBase* tc;
  double motionEndTime;
  double lastMotionEndTime;
  double motionDuration;
  MatNd* a_des;
  MatNd* x_des;
  bool enableTrajectoryCheck;
  bool eStop;

  std::mutex checkerThreadMtx;

  std::map<std::string, std::vector<std::string>> effectorTaskMap;

  TrajectoryComponent(const TrajectoryComponent&) = delete;
  TrajectoryComponent& operator=(const TrajectoryComponent&) = delete;
};

}

#endif   // RCS_TRAJECTORYCOMPONENT_H
