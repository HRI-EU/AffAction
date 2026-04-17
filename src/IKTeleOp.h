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

#ifndef AFF_IKTELEOP_H
#define AFF_IKTELEOP_H


#include "ComponentBase.h"
#include "ActionScene.h"

#include <IkSolverRMR.h>
#include <Rcs_filters.h>



namespace aff
{

/*! \brief Inverse kinematics class. Solves the inverse kinematics (resolved
 *         motion rate control) with the right inverse.
 */
class IKTeleOp : public ComponentBase
{
public:

  /*! \brief Constructs IKTeleOp with the task vector and collision model
   *         from the argument controller.
   *
   * \param[in] parent     Entity class responsible for event subscriptions
   * \param[in] controller Controller with all task variables and (possibly)
   *                       a collision model. It is a pointer and not owned
   *                       by this class.
   */
  IKTeleOp(EntityBase* parent, Rcs::ControllerBase* controller);

  /*! \brief Destroys the IkSolverRMR and all other allocated memory.
   */
  virtual ~IKTeleOp() = default;

  /*! \brief Returns a pointer to the internal joint position command array.
   *
   * \return Pointer to the internal joint position MatNd of
   *         dimension [RcsGraph::dof x 1]
   */
  const MatNd* getJointCommandPtr() const;

  /*! \brief Accessor to internal graph structure.
   *
   * \return Pointer to internal RcsGraph data structure.
   */
  RcsGraph* getGraph();

  /*! \brief Read-only accessor to internal graph structure.
   *
   * \return Const pointer to internal RcsGraph data structure.
   */
  const RcsGraph* getGraph() const;

  /*! \brief Enable or disable speed and acceleration limits. If it is enabled,
   *         the IK solver function will scale the resulting speeds and
   *         accelerations to always be below the values given in the joints
   *         xml description.
   *
   *  \param[in] enable   True for enabling, false for disabling.
   */
  void setEnableSpeedAccelerationLimit(bool enable);

  /*! \brief Enable or disable speed limit checks. If it is disabled, no
   *         EmergencyStop event will be triggered upon speed limit violations.
   *         It is strongly discouraged to disable when connecting to robots.
   *
   *  \param[in] enable   True for enabling, false for disabling speed limits.
   */
  void setSpeedLimitCheck(bool enable);

  /*! \brief Enable or disable joint limit checks. If it is disabled, no
   *         EmergencyStop event will be triggered upon joint limit violations.
   *         It is strongly discouraged to disable when connecting to robots.
   *
   *  \param[in] enable   True for enabling, false for disabling joint limits.
   */
  void setJointLimitCheck(bool enable);

  /*! \brief Enable or disable collision checks. If it is disabled, no
   *         EmergencyStop event will be triggered upon collisions.
   *         It is strongly discouraged to disable when connecting to robots.
   *
   *  \param[in] enable   True for enabling, false for disabling collision
   *                      checks.
   */
  void setCollisionCheck(bool enable);

  /*! \brief Sets the scaling factor for the null space motion.
   *
   *  \param[in] alpha   Null space scaling. When zero, no null space motion
   *                     will be part of the motion.
   */
  void setAlpha(double alpha);

  /*! \brief Gets the scaling factor for the null space motion.
   *
   *  \return Current null space scaling factor.
   */
  double getAlpha() const;

  /*! \brief Sets the regularization value.
   *
   *  \param[in] lambda  Regularization value to be applied in IK.
   */
  void setLambda(double lambda);

  /*! \brief Gets the regularization value.
   *
   *  \return Current regularization value.
   */
  double getLambda() const;

  void onTwistCommand(std::array<double, 6> twist, bool inWorldFrame);
  void onWrenchCommand(std::array<double, 6> wrench, bool inWorldFrame);
  void onRetargetCommand(RcsGraph* desired, RcsGraph* current, ActionScene* scene);
  double getJointSpeedScaling() const;

private:

  void onEmergencyStop();
  void onEmergencyRecover();
  void onInitFromState(const RcsGraph* target);
  void onRender();
  void onEnableRetargetting(bool enable);
  void print() const;
  void computeIK(const MatNd* a_des, const MatNd* x_des, const MatNd* lambdaArr);

  Rcs::IkSolverRMR ikSolver;
  bool eStop;
  double alpha;
  double lambda;
  bool speedLimitCheck;   ///< Default is on
  bool jointLimitCheck;   ///< Default is on
  bool collisionCheck;    ///< Default is on
  bool applySpeedAndAccLimits;    ///< Default is on
  Rcs::Ramp1D jointSpeedScaling;

  /*! \brief We disallow copying and assigning this class.
   */
  IKTeleOp(const IKTeleOp&) = delete;
  IKTeleOp& operator=(const IKTeleOp&) = delete;
};

}

#endif   // AFF_IKTELEOP_H
