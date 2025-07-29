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

#include "ComponentFactory.h"

#include <Rcs_macros.h>


namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
static std::map<std::string, ComponentFactory::CreatorFcn>& constructorMap()
{
  static std::map<std::string, ComponentFactory::CreatorFcn> cm;
  return cm;
}

/*******************************************************************************
*
******************************************************************************/
ComponentFactory::ComponentFactory()
{
}

/*******************************************************************************
 * Print all registered components to the console
 ******************************************************************************/
void ComponentFactory::print()
{
  auto it = constructorMap().begin();

  std::cout << constructorMap().size() << " registered components:\n";

  while (it != constructorMap().end())
  {
    std::cout << "  " << it->first << std::endl;
    it++;
  }
}

/*******************************************************************************
 * Creates the component for className and the given args
 ******************************************************************************/
ComponentBase* ComponentFactory::create(std::string parseArg,
                                        EntityBase* entity,
                                        const RcsGraph* graph,
                                        std::string extraArgs)
{
  return create(parseArg, entity, graph, nullptr, extraArgs);
}

ComponentBase* ComponentFactory::create(std::string parseArg,
                                        EntityBase* entity,
                                        const RcsGraph* graph,
                                        const ActionScene* scene,
                                        std::string extraArgs)
{
  ComponentContext ctx;
  ctx.entity = entity;
  ctx.graph = graph;
  ctx.scene = scene;
  ctx.extraArgs = extraArgs;

  ComponentBase* newComponent = nullptr;

  std::map<std::string, CreatorFcn>::iterator it;
  it = constructorMap().find(parseArg);

  if (it != constructorMap().end())
  {
    newComponent = it->second(ctx);
  }
  else
  {
    RLOG(1, "Couldn't find constructor for class \"%s\"", parseArg.c_str());
  }

  return newComponent;
}

/*******************************************************************************
 * This function is called through the registrar class. This happens before
 * main() is entered. Therefore, logging with debug levels doesn't make sense,
 * since the debug level has at that point not yet been parsed.
 ******************************************************************************/
void ComponentFactory::registerComponent(std::string name,
                                         CreatorFcn createFunction)
{
  auto it = constructorMap().find(name);
  if (it != constructorMap().end())
  {
    // No log level, this happens before main()
    RMSG_CPP("Overwriting a component creation function: '" << name << "'");
  }

  constructorMap()[name] = createFunction;
}


}   // namespace
