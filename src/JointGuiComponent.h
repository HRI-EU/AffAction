/*******************************************************************************

  Copyright (c) by Honda Research Institute Europe GmbH

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

#ifndef DC_JOINTGUICOMPONENT_H
#define DC_JOINTGUICOMPONENT_H

#include "ComponentBase.h"

#include <JointWidget.h>
#include <Rcs_filters.h>


namespace aff
{
/*! \brief Joint gui class. Runs a JointWidget in its own thread, reads in
 *         the slider values, filters them, and provides them as command
 *         vector in a MatNd. Further, the gui displays the current joint
 *         angles communicated through the ComputeKinematics event.
 *
 *         The class publishes no events.
 *
 *         The class subscribes to the following events:
 *         - Start: Launches a JointWidget. This subscriber does not check if
 *                  another widget is already running.
 *         - Stop: Destroys all running JointWidgets
 *         - InitFromState: Sets the slider values of all running JointWidgets
 *                          to the values of the passed graph.
 *         - ComputeKinematics: Iterates all joint value filters and updates
 *                              the corresponding values.
 *         - EmergencyStop: Initializes all guis with the latest current state.
 *         - EmergencyRecover: Initializes all guis with the latest current
 *                             state.
 *
 */
class JointGuiComponent : public ComponentBase
{
  friend class JointUpdateCallback;

public:

  /*! \brief Constructs a JointGuiComponent instance and subscribes to all
   *         events described above. The constructor does not launch a gui.
   *         This happens only through the Start event.
   *
   *  \param[in] parent   Entity class responsible for event subscriptions
   *  \param[in] graph    Graph template for the gui. It will be cloned.
   *  \param[in] tmc      Time constant for 2nd order filter applied to sliders
   *  \param[in] vmax     Speed limit for 2nd order filter applied to sliders
   */
  JointGuiComponent(EntityBase* parent, const RcsGraph* graph,
                    double tmc=0.25, double vmax=60.0*3.1415/180.0);

  /*! \brief Unsubscribes all subscribers and frees all memory.
   */
  virtual ~JointGuiComponent();

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
  const RcsGraph* getGraph() const;

  /*! \brief Sets the Gui desired joint values to q_goal, and resets the Gui
   *         sliders so that they are consistent with the new values. If q_goal
   *         does not match the dimensions of the internal graph's q-vector,
   *         the function complains on debug level 1 and returns without doing
   *         anything.
   *
   *  \param[in] q_goal   Desired joint angles vector.
   */
  void setGoalPose(const MatNd* q_goal);

private:

  void guiCallback();
  void onStart();
  void onStop();
  void onInitialize(const RcsGraph* target);
  void onFilterAndUpdateGui(RcsGraph* from);
  void onRender();
  void onEmergencyStop();
  void onEmergencyRecover();
  void onGoalPose(std::string goalPose);
  void onSetSineAmplitude(double amplitude);

  RcsGraph* guiGraph = nullptr;
  MatNd* q_des = nullptr;
  MatNd* q_curr = nullptr;
  MatNd* q_des_filt = nullptr;
  Rcs::RampFilterND* filt = nullptr;
  Rcs::JointGui* jGui = nullptr;
  mutable pthread_mutex_t mtx;
  Rcs::Ramp1D sine_amplitude;

  JointGuiComponent(const JointGuiComponent&);
  JointGuiComponent& operator=(const JointGuiComponent&);
};

}

#endif   // DC_JOINTGUICOMPONENT_H
