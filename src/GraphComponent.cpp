/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH
  Carl-Legien Str. 30
  63073 Offenbach/Main
  Germany

  UNPUBLISHED PROPRIETARY MATERIAL.
  ALL RIGHTS RESERVED

*******************************************************************************/

#include "GraphComponent.h"

#include <Rcs_joint.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>


namespace aff
{

GraphComponent::GraphComponent(EntityBase* parent, const std::string& cfgFile) :
  ComponentBase(parent), graph(RcsGraph_create(cfgFile.c_str())),
  renderingInitialized(false), enableRender(true)
{
  RCHECK(graph);
  subscribeAll();
}

GraphComponent::GraphComponent(EntityBase* parent, const RcsGraph* graph_) :
  ComponentBase(parent), graph(RcsGraph_clone(graph_)),
  renderingInitialized(false), enableRender(true)
{
  RCHECK(graph);
  subscribeAll();
}

GraphComponent::~GraphComponent()
{
  unsubscribe();
  RcsGraph_destroy(this->graph);
}

void GraphComponent::subscribeAll()
{
  subscribe("ComputeKinematics", &GraphComponent::onForwardKinematics);
  subscribe("TriggerInitFromState", &GraphComponent::onInitFromState);
  subscribe("Render", &GraphComponent::onRender);
  subscribe("Print", &GraphComponent::onPrint);
}

std::string GraphComponent::getName() const
{
  return std::string("GraphComponent");
}

void GraphComponent::onForwardKinematics(RcsGraph* graph)
{
  RcsGraph_setState(graph, NULL, NULL);
}

RcsGraph* GraphComponent::getGraph()
{
  return this->graph;
}

void GraphComponent::onInitFromState()
{
  RLOG(0, "GraphComponent::onInitFromState()");
  getEntity()->publish<const RcsGraph*>("InitFromState", graph);
}

void GraphComponent::onRender()
{
  if (!this->enableRender)
  {
    return;
  }

  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "Physics",
                                                    graph);

  if (this->renderingInitialized == false)
  {
    //getEntity()->publish<std::string,std::string>("RenderCommand", "Physics",
    //                                              "toggleGraphicsModel");
    //getEntity()->publish<std::string,std::string>("RenderCommand", "Physics",
    //                                              "togglePhysicsModel");
    this->renderingInitialized = true;
  }

}

void GraphComponent::setEnableRender(bool enable)
{
  this->enableRender = enable;
}

bool GraphComponent::getEnableRender() const
{
  return this->enableRender;
}

void GraphComponent::onPrint() const
{
  RcsGraph_fprintModelState(stdout, this->graph, this->graph->q, NULL, 0);
}

}   // namespace aff
