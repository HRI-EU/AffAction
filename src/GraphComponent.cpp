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
  subscribe("TriggerInitFromState", &GraphComponent::onTriggerInitFromState);
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

void GraphComponent::onTriggerInitFromState()
{
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
