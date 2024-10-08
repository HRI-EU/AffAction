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

#ifndef AFF_GRAPHCOMPONENT_H
#define AFF_GRAPHCOMPONENT_H


#include "ComponentBase.h"

#include <Rcs_graph.h>



namespace aff
{
/*! \brief Graph wrapper class. Provides methods to compute forward kinematics,
 *         to initialize, and to visualize graphs.
 *
 *         The class publishes the following events:
 *         - InitFromState: In the subscriber "TriggerInitFromState"
 *         - RenderGraph: Renders a graph with identifier "Physics"
 *         - RenderCommand: Only once, the "toggleGraphicsModel" and
 *                          "togglePhysicsModel" commands are called.
 *
 *         The class subscribes to the following events:
 *         - ComputeKinematics: Calculates the graph's forward kinematics
 *         - TriggerInitFromState: publishes an event "InitFromState" with his
 *                                 class's internal graph
 *         - Render: Publishes the above rendering events.
 *         - Print: Prints the graph's model state to the console.
 */
class GraphComponent : public ComponentBase
{
public:

  /*! \brief Constructs GraphComponent from config file.
   *
   * \param[in] parent    Entity class responsible for event subscriptions
   * \param[in] cfgFile   XML file with graph description.
   */
  GraphComponent(EntityBase* parent, const std::string& cfgFile);

  /*! \brief Constructs GraphComponent with a clone of the passed graph.
   *
   * \param[in] parent    Entity class responsible for event subscriptions
   * \param[in] graph     Underlying RcsGraph structure. The class will create
   *                      a copy of the argument.
   */
  GraphComponent(EntityBase* parent, const RcsGraph* graph);

  /*! \brief Unsubscribes and deletes all previously allocated memory.
   *         There is no thread that needs to be stopped.
   */
  virtual ~GraphComponent();

  /*! \brief Read-only accessor to internal graph structure.
   *
   * \return Const pointer to internal RcsGraph data structure.
   */
  RcsGraph* getGraph();

  /*! \brief Returns the name of this component.
   *
   * \return "GraphComponent"
   */
  std::string getName() const;

  /*! \brief Enable or disable publiching the Render event.
   *
   * \param[in] enable   True for publishing, false otherwise.
   */
  void setEnableRender(bool enable);

  /*! \brief Returns if the component is publishing the Render event.
   *
   * \return True for rendering, false for not.
   */
  bool getEnableRender() const;

private:

  virtual void subscribeAll();

  void onRender();
  void onPrint() const;
  void onForwardKinematics(RcsGraph* graph);
  void onTriggerInitFromState();

  // Shape modifications. There should go somewhere else, since they are not related to
  // this component's graph.
  RcsShape* getShape(RcsGraph* graph, std::string bodyName, size_t shapeIndex);
  void onChangeShapeHeight(RcsGraph* graph, std::string bodyName, double height);
  void onChangeShapeDiameter(RcsGraph* graph, std::string bodyName, double diameter);
  void onChangeShapeOrigin(RcsGraph* graph, std::string bodyName, std::vector<double> origin);
  void onChangeBodyOrigin(RcsGraph* graph, std::string bodyName, std::vector<double> origin);

  RcsGraph* graph;
  bool enableRender;


  GraphComponent(const GraphComponent&);
  GraphComponent& operator=(const GraphComponent&);
};

}

#endif   // AFF_GRAPHCOMPONENT_H
