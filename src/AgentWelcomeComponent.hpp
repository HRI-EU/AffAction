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

#if !defined (AFF_AGENTWELCOMECOMPONENT_H)
#define AFF_AGENTWELCOMECOMPONENT_H

#include "ComponentBase.h"
#include "SceneHelpers.h"

#include <Rcs_macros.h>

#include <vector>
#include <thread>
#include <algorithm>


namespace aff
{

class AgentWelcomeComponent : public ComponentBase
{
private:

  std::unique_ptr<ES::ScopedSubscription> agentChangedSub, renameAgentSub;

  //////////////////////////////////////////////////////////////////////////////
  // On an agent appearing, this callback launches a thread that triggers and
  // waits for face recognition and emits an event to change the agent's name
  // if recognized.
  //////////////////////////////////////////////////////////////////////////////
  ES::ScopedSubscription createAgentChangedSubscriber(EntityBase* entity,
                                                      const ActionScene* scene,
                                                      bool recognize)
  {
    return entity->subscribe("AgentChanged", [entity, scene, recognize]
                             (std::string agentName, bool appeared) mutable
    {
      std::thread([](EntityBase* entity, const ActionScene* scene, std::string agentName, bool appeared, bool recognize)
      {
        std::string text;
        RLOG_CPP(1, "AgentChanged");

        if (!recognize)
        {
          if (appeared)
          {
            text = "Hello";
          }
          else
          {
            text = "Bye bye";
          }
        }
        else   // with recognition
        {
          if (appeared)
          {
            RLOG_CPP(1, "Agent appeared");
            std::pair<std::string, std::string> res;
            res = recognize_agent_face(*entity, scene, agentName, 3, 2.0);
            std::string recognized = res.first;
            if (recognized.empty())
            {
              recognized = "unknown_person";
              text = "Hello, I don't think we met before.";
            }
            else if (recognized != res.second)
            {
              text = "Hello " + recognized + " nice to see you!";
            }
            else
            {
              text = "Hello again, " + recognized;
            }

            // It is better to publish it, because otherwise we might face
            // concurrency issues with reading and writing agent names.
            entity->publish("RenameAgent", res.second, recognized);
          }
          else   // disappered
          {
            text = "Bye " + agentName;
          }
        }   // recognize

        entity->publish("Speak", text);
      },
      entity, scene, std::move(agentName), appeared, recognize).detach();
    });

  }

  //////////////////////////////////////////////////////////////////////////////
  // Renames an agent with a new name
  //////////////////////////////////////////////////////////////////////////////
  ES::ScopedSubscription createAgentRenameSubscriber(EntityBase* entity,
                                                     const ActionScene* scene)
  {
    return entity->subscribe("RenameAgent", [scene]
                             (std::string from_name, std::string to_name) mutable
    {
      Agent* agent = nullptr;
      RLOG_CPP(1, "RenameAgent from '" << from_name << "' to '" << to_name << "'");

      for (auto& a : scene->agents)
      {
        if (a->name == from_name)
        {
          if (dynamic_cast<HumanAgent*>(a))
          {
            agent = a;
            break;
          }
          else
          {
            RLOG_CPP(1, "Can't rename robot agent '" << from_name << "'");
          }
        }
      }


      if (!agent)
      {
        RLOG_CPP(1, "Can't find agent '" << from_name
                 << "' - skipping renaming to '" << to_name << "'");
        return;
      }

      RLOG_CPP(1, "Renaming agent from '" << from_name << "' to '"
               << to_name << "'");

      agent->name = to_name;

      // Remove old name from types, and add new one
      RLOG_CPP(1, "Erasing from_name from types: " << from_name);
      agent->types.erase(std::remove(agent->types.begin(),
                                     agent->types.end(),
                                     from_name), agent->types.end());

      RLOG_CPP(1, "Adding to_name to types: " << to_name);
      agent->types.push_back(to_name);
    });

  }

public:
  AgentWelcomeComponent(EntityBase* parent, const ActionScene* scene, bool recognize=true) : ComponentBase(parent)
  {
    parent->withProcessLock([this, parent, scene, recognize]()
    {
      agentChangedSub = std::make_unique<ES::ScopedSubscription>(
                          createAgentChangedSubscriber(parent, scene, recognize));

      if (recognize)
      {
        renameAgentSub = std::make_unique<ES::ScopedSubscription>(
                           createAgentRenameSubscriber(parent, scene));
      }
    });

  }

  virtual ~AgentWelcomeComponent()
  {
    getEntity()->withProcessLock([this]()
    {
      agentChangedSub->unsubscribe();
      renameAgentSub->unsubscribe();
    });
  }

};

}   // namespace

#endif // AFF_AGENTWELCOMECOMPONENT_H
