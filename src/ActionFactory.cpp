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

#include "ActionFactory.h"

#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>


namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
ActionFactory::ActionFactory()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string ActionFactory::printToString()
{
  std::string str = std::to_string(constructorMap().size()) +
                    " registered actions:\n";

  auto it = constructorMap().begin();
  while (it != constructorMap().end())
  {
    str += "  " + it->first + "\n";
    it++;
  }

  return str;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ActionFactory::print()
{
  std::cout << printToString();
}

/*******************************************************************************
 *
 ******************************************************************************/
ActionBase* ActionFactory::create(const ActionScene& domain,
                                  const RcsGraph* graph,
                                  std::vector<std::string> words,
                                  std::string& explanation)
{
  std::string aname = words[0];
  words.erase(words.begin());
  return ActionFactory::create(domain, graph, aname, words, explanation);
}

/*******************************************************************************
  *
  ******************************************************************************/
ActionBase* ActionFactory::create(const ActionScene& domain,
                                  const RcsGraph* graph,
                                  std::string actionName,
                                  std::vector<std::string> params,
                                  std::string& explanation)
{
  ActionBase* action = NULL;

  std::map<std::string, ActionMaker>::iterator it;
  it = constructorMap().find(actionName);

  if (it == constructorMap().end())
  {
    explanation = "ERROR REASON: The action " +  actionName + " does not exist";
    RLOG_CPP(1, explanation);
    return NULL;
  }

  try
  {
    action = it->second(domain, graph, params);
    action->setName(actionName);
    action->setActionParams(params);
  }
  catch (const std::exception& ex)
  {
    explanation = ex.what();
    RLOG_CPP(1, explanation);
    action = NULL;
  }
  catch (...)
  {
    explanation = "Failed to create action: unknown reason";
    RLOG_CPP(1, explanation);
    action = NULL;
  }

  return action;
}

/*******************************************************************************
 * This function is called through the registrar class. This happens before
 * main() is entered. Therefore, logging with debug levels doesn't make sense,
 * since the debug level has at that point not yet been parsed.
 ******************************************************************************/
void ActionFactory::registerAction(std::string name, ActionMaker createFunction)
{
  auto it = constructorMap().find(name);
  if (it != constructorMap().end())
  {
    // No log level, this happens before main()
    RMSG("Overwriting a constraint creation function: \"%s\"", name.c_str());
  }

  constructorMap()[name] = createFunction;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::map<std::string, ActionFactory::ActionMaker>& ActionFactory::constructorMap()
{
  static std::map<std::string, ActionFactory::ActionMaker> cm;
  return cm;
}



}   // namespace tropic
