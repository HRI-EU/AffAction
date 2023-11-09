/******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH
  Carl-Legien Str. 30
  63073 Offenbach/Main
  Germany

  UNPUBLISHED PROPRIETARY MATERIAL.
  ALL RIGHTS RESERVED

******************************************************************************/

#ifndef RCS_GRAPHCOMPONENT_H
#define RCS_GRAPHCOMPONENT_H


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
  void onInitFromState();

  RcsGraph* graph;
  bool renderingInitialized;
  bool enableRender;


  GraphComponent(const GraphComponent&);
  GraphComponent& operator=(const GraphComponent&);
};

}

#endif   // RCS_GRAPHCOMPONENT_H
