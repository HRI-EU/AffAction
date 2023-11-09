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

#ifndef AFF_IKCOMPONENT_H
#define AFF_IKCOMPONENT_H


#include "ComponentBase.h"

#include <IkSolverRMR.h>



namespace aff
{

/*! \brief Inverse kinematics class. Solves the inverse kinematics (resolved
 *         motion rate control) with the right (exact) inverse, and an active
 *         set constraint solver extension that considers all joint limits and
 *         all pairs of a given collision model. The IK is solved within the
 *         SetTaskCommand event. There are a number of checks that lead to an
 *         emergency stop event to be published:
 *         - Joint speed violation
 *         - Joint limit violation
 *         - Collision detected
 *         - Singular IK (no solution found)
 *         The first three limit checks can be deactivated using the below
 *         described functions.
 *
 *         The class publishes the following events:
 *         - EmergencyStop: On joint, speed, collision violations, or if IK
 *                          has no solution.
 *         - InitFromState: On the TriggerInitFromDesiredState.
 *         - RenderGraph: With identifier "IK".
 *         - RenderCommand: On the first render call, "setGhostMode" is
 *                          published.
 *
 *         The class subscribes to the following events:
 *         - SetTaskCommand: Computes the IK and updates the internal joint
 *                           command vector.
 *         - EmergencyStop: Sets the internal eStop flag to true, and sets the
 *                          null space scaling factor alpha to 0, so that there
 *                          are no null space jumps on EmergencyRecover.
 *         - EmergencyRecover: Sets the internal eStop flag to false.
 *         - Render: Publishes the above mentioned RenderGraph and RenderCommand
 *                   events.
 *         - InitFromState: Sets the component's internal graph to the position
 *                          and velocity state published, and recomputes the
 *                          collision model with the new state.
 *         - TriggerInitFromDesiredState: Published an InitFromState Event that
 *                                        corresponds to this classes internal
 *                                        state.
 *         - Print: Prints this classes collision model (if exists) and model
 *                  state (taken from the current q-vector) to the console.
 *
 *         \todo:
 *         - In future, this class should be extended with an API to set and
 *           get the IK regularization, and with the left inverse.
 */
class IKComponent : public ComponentBase
{
public:

  enum IkSolverType
  {
    RMR = 0,
    ConstraintRMR,
    IkSolverTypeMaxIndex
  };

  /*! \brief Constructs IKComponent with the task vector and collision model
   *         from the argument controller.
   *
   * \param[in] parent     Entity class responsible for event subscriptions
   * \param[in] controller Controller with all task variables and (possibly)
   *                       a collision model. It will be cloned.
   */
  IKComponent(EntityBase* parent, Rcs::ControllerBase* controller,
              IkSolverType ik=ConstraintRMR);

  /*! \brief Destroys the IkSolverRMR and all other allocated memory.
   */
  virtual ~IKComponent();

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

  /*! \brief Performs the above event subscriptions and stores them internally.
   */
  virtual void subscribeAll();

  /*! \brief Publishes the internal model to be rendered solid. The default is
   *         "GhostMode". Needs to be called before processing any event of the
   *         GraphicsWindow.
   */
  virtual void renderSolidModel();

private:

  void onTaskCommand(const MatNd* a_des, const MatNd* x_des);
  void onEmergencyStop();
  void onEmergencyRecover();
  void onInitFromState(const RcsGraph* target);
  void onTriggerInitFromDesiredState();
  void onRender();
  void onSetBlending(double value);
  void onSetPhase(double value);
  void onLinkGenericBody(int gBodyId, std::string bodyName);
  void onTaskVectorChange(std::vector<std::string> taskVec,
                          std::vector<std::string> channels);
  void print() const;

  Rcs::ControllerBase* controller;
  Rcs::IkSolverRMR* ikSolver;
  MatNd* a_prev;
  bool eStop;
  double alphaMax;
  double alpha;
  double lambda;          ///< Default is 1.0e-6
  double blending;
  double phase;
  double qFilt;
  bool renderSolid;
  bool speedLimitCheck;   ///< Default is on
  bool jointLimitCheck;   ///< Default is on
  bool collisionCheck;    ///< Default is on
  bool applySpeedAndAccLimits;    ///< Default is on

  /*! \brief We disallow copying and assigning this class.
   */
  IKComponent(const IKComponent&);
  IKComponent& operator=(const IKComponent&);
};

}

#endif   // AFF_IKCOMPONENT_H
