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
#include <Rcs_Vec3d.h>


namespace aff
{

GraphComponent::GraphComponent(EntityBase* parent, const std::string& cfgFile) :
  ComponentBase(parent), graph(RcsGraph_create(cfgFile.c_str())), enableRender(true)
{
  RCHECK(graph);
  subscribeAll();
}

GraphComponent::GraphComponent(EntityBase* parent, const RcsGraph* graph_) :
  ComponentBase(parent), graph(RcsGraph_clone(graph_)), enableRender(true)
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
  subscribe("ChangeShapeHeight", &GraphComponent::onChangeShapeHeight);
  subscribe("ChangeShapeDiameter", &GraphComponent::onChangeShapeDiameter);
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

void GraphComponent::onChangeShapeHeight(std::string bodyName, double height)
{
  RcsShape* sh = getShape(bodyName, 0);

  if (sh)
  {
    sh->extents[2] = height;
  }
}

void GraphComponent::onChangeShapeDiameter(std::string bodyName, double diameter)
{
  RcsShape* sh = getShape(bodyName, 0);

  if (sh)
  {
    sh->extents[0] = diameter;
  }
}

void GraphComponent::onChangeShapeParameters(std::string bodyName, size_t shapeIndex, double extents[3])
{
  RcsShape* sh = getShape(bodyName, shapeIndex);

  if (sh)
  {
    Vec3d_copy(sh->extents, extents);
  }
}

RcsShape* GraphComponent::getShape(std::string bodyName, size_t shapeIndex)
{
  const RcsBody* bdy = RcsGraph_getBodyByName(graph, bodyName.c_str());

  if (!bdy)
  {
    RLOG_CPP(1, "Couldn't find body \"" << bodyName << "\" in graph");
    return nullptr;
  }

  if (shapeIndex >= bdy->nShapes)
  {
    RLOG_CPP(1, "Shape index " << shapeIndex << " out of range, only " << bdy->nShapes << " shapes in body");
    return nullptr;
  }

  return &bdy->shapes[shapeIndex];
}

void GraphComponent::onRender()
{
  if (this->enableRender)
  {
    getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "Physics", graph);
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
