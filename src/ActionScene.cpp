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

#include "ActionScene.h"

#include <Rcs_typedef.h>
#include <Rcs_shape.h>
#include <Rcs_stlParser.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_utils.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_body.h>
#include <Rcs_resourcePath.h>

#include <algorithm>
#include <exception>

/*

The ActionScene class comprises all manipulators and affordance models.
These classes are independent of any action.

*/

namespace aff
{

ActionScene::ActionScene()
{
}

ActionScene::ActionScene(const std::string& xmlFile)
{
  xmlDocPtr doc = NULL;
  xmlNodePtr node = NULL;

  if (File_exists(xmlFile.c_str()))
  {
    node = parseXMLFile(xmlFile.c_str(), "Graph", &doc);
  }
  else
  {
    node = parseXMLMemory(xmlFile.c_str(), xmlFile.length()+1, &doc);
  }

  if (node)
  {
    parseRecursive(node, this);
    parseAgents(node, this);
  }
  else
  {
    RLOG(4, "Failed to read xml file \"%s\"", xmlFile.c_str());
  }

  xmlFreeDoc(doc);
}

ActionScene::ActionScene(const ActionScene& copyFromMe)
{
  entities = copyFromMe.entities;
  manipulators = copyFromMe.manipulators;
  foveatedEntity = copyFromMe.foveatedEntity;

  for (size_t i=0; i<copyFromMe.agents.size(); ++i)
  {
    agents.push_back(copyFromMe.agents[i]->clone());
  }

}

ActionScene::~ActionScene()
{
  for (size_t i=0; i<agents.size(); ++i)
  {
    delete agents[i];
  }

}

ActionScene& ActionScene::operator= (const ActionScene& copyFromMe)
{
  if (this == &copyFromMe)
  {
    return *this;
  }

  entities = copyFromMe.entities;
  manipulators = copyFromMe.manipulators;

  for (size_t i=0; i<agents.size(); ++i)
  {
    delete agents[i];
  }
  agents.clear();

  for (size_t i=0; i<copyFromMe.agents.size(); ++i)
  {
    agents.push_back(copyFromMe.agents[i]->clone());
  }

  foveatedEntity = copyFromMe.foveatedEntity;

  return *this;
}

bool ActionScene::reload(const std::string& xmlFile)
{
  entities.clear();
  manipulators.clear();
  foveatedEntity.clear();

  char cfgFile[RCS_MAX_FILENAMELEN] = "";
  bool fileExists = Rcs_getAbsoluteFileName(xmlFile.c_str(), cfgFile);

  if (!fileExists)
  {
    RLOG_CPP(1, "XML file " << xmlFile << " not found in resource paths");
    return false;
  }

  xmlDocPtr doc = NULL;
  xmlNodePtr node = parseXMLFile(cfgFile, "Graph", &doc);

  if (node)
  {
    parseRecursive(node, this);
    parseAgents(node, this);
  }
  else
  {
    RLOG(4, "Failed to parse xml file \"%s\"", cfgFile);
    return false;
  }

  xmlFreeDoc(doc);

  return true;
}

void ActionScene::print() const
{
  std::cout << "Scene has " << entities.size() << " affordance models:"
            << std::endl;
  for (const auto& e : entities)
  {
    e.print();
  }

  std::cout << "Scene has " << manipulators.size() << " manipulators:"
            << std::endl;
  for (const auto& m : manipulators)
  {
    m.print();
  }

  for (const auto& a : agents)
  {
    a->print();
  }

}

bool ActionScene::check(const RcsGraph* graph) const
{
  bool success = true;

  for (auto& e : entities)
  {
    success = e.check(graph) && success;
  }

  for (auto& m : manipulators)
  {
    success = m.check(graph) && success;
  }

  for (auto& a : agents)
  {
    success = a->check(this, graph) && success;
  }

  return success;
}

void ActionScene::parseRecursive(const xmlNodePtr parentNode, ActionScene* scene)
{

  for (xmlNodePtr node = parentNode; node; node = node->next)
  {
    if (node->type == XML_ELEMENT_NODE)
    {
      if (xmlStrcmp(node->name, BAD_CAST "AffordanceModel")==0)
      {
        scene->entities.emplace_back(AffordanceEntity(node));
      }
      else if (xmlStrcmp(node->name, BAD_CAST "Manipulator")==0)
      {
        scene->manipulators.emplace_back(Manipulator(node));
      }

    }

    parseRecursive(node->children, scene);
  }
}

void ActionScene::parseAgents(const xmlNodePtr parentNode, ActionScene* scene)
{
  // To be called after `parseRecursive`
  // Requires Manipulators to be already initialized
  for (xmlNodePtr node = parentNode; node; node = node->next)
  {
    if (node->type == XML_ELEMENT_NODE)
    {
      if (xmlStrcmp(node->name, BAD_CAST "Agent")==0)
      {
        scene->agents.emplace_back(Agent::createAgent(node, scene));
      }
    }
    parseAgents(node->children, scene);
  }
}

ActionScene ActionScene::parse(const std::string& xmlFile)
{
  ActionScene scene;
  xmlDocPtr doc = NULL;
  xmlNodePtr node = parseXMLFile(xmlFile.c_str(), "Graph", &doc);

  if (node)
  {
    parseRecursive(node, &scene);
    parseAgents(node, &scene);
  }
  else
  {
    RLOG(4, "Failed to read xml file \"%s\"", xmlFile.c_str());
  }

  xmlFreeDoc(doc);

  return std::move(scene);
}

std::string ActionScene::printAffordancesToString() const
{
  std::string str = std::to_string(entities.size()) +
                    " known affordances:\n";

  auto it = entities.begin();
  while (it != entities.end())
  {
    str += "  ";
    str += it->name;
    str += "\n";
    it++;
  }

  return str;
}

std::vector<const Manipulator*> ActionScene::getFreeManipulators(const RcsGraph* graph) const
{
  std::vector<const Manipulator*> freeManipulators;

  for (const auto& m : manipulators)
  {
    if (m.isEmpty(graph))
    {
      freeManipulators.push_back(&m);
    }
  }

  return freeManipulators;
}


std::vector<const Manipulator*> ActionScene::getOccupiedManipulators(const RcsGraph* graph) const
{
  std::vector<const Manipulator*> occupiedManipulators;

  for (const auto& m : manipulators)
  {
    if (!m.isEmpty(graph))
    {
      occupiedManipulators.push_back(&m);
    }
  }

  return occupiedManipulators;
}


const AffordanceEntity* ActionScene::getAffordanceEntity(const std::string& name) const
{
  if (name.empty())
  {
    return NULL;
  }

  for (size_t i=0; i<entities.size(); ++i)
  {
    // Return first entity whose types contains the requested name.
    if (std::find(entities[i].types.begin(), entities[i].types.end(), name) != entities[i].types.end())
    {
      return &entities[i];
    }
  }

  return NULL;
}

std::vector<const AffordanceEntity*> ActionScene::getAffordanceEntities(const std::string& name) const
{
  std::vector<const AffordanceEntity*> foundOnes;

  if (name.empty())
  {
    return foundOnes;
  }

  // Add entities whose types contains the requested name.
  for (size_t i=0; i<entities.size(); ++i)
  {
    if (std::find(entities[i].types.begin(), entities[i].types.end(), name) != entities[i].types.end())
    {
      foundOnes.push_back(&entities[i]);
    }
  }

  return foundOnes;
}

std::vector<const SceneEntity*> ActionScene::getSceneEntities(const std::string& name) const
{
  std::vector<const SceneEntity*> res;

  auto v1 = getAffordanceEntities(name);

  if (!v1.empty())
  {
    res.assign(v1.begin(), v1.end());
  }

  auto v2 = getAgent(name);
  if (v2)
  {
    res.push_back(v2);
  }

  auto v3 = getManipulator(name);
  if (v3)
  {
    res.push_back(v3);
  }

  return res;
}


std::vector<const AffordanceEntity*> ActionScene::getAllChildren(const RcsGraph* graph,
                                                                 const AffordanceEntity* entity) const
{
  std::vector<const AffordanceEntity*> foundOnes;

  if (!entity)
  {
    return foundOnes;
  }

  RcsBody* bdy = RcsGraph_getBodyByName(graph, entity->bdyName.c_str());
  RCHECK(bdy);   // Should never happen \todo(MG): Remove if tested

  RCSBODY_TRAVERSE_CHILD_BODIES(graph, bdy)
  {
    for (const auto& e : entities)
    {
      if (std::string(BODY->name) == e.bdyName)
      {
        foundOnes.push_back(&e);
      }
    }
  }

  return foundOnes;
}

std::vector<const AffordanceEntity*> ActionScene::getDirectChildren(const RcsGraph* graph,
                                                                    const AffordanceEntity* parent) const
{
  std::vector<const AffordanceEntity*> foundOnes;

  if (!parent)
  {
    return foundOnes;
  }

  const RcsBody* parentBdy = RcsGraph_getBodyByName(graph, parent->bdyName.c_str());
  RCHECK(parentBdy);

  // There might be several affordances sharing the same frame. We therefore
  // collect all unique frames, and then search through their children.
  std::vector<std::string> parentFrames;
  for (const Affordance* parentAffordance : parent->affordances)
  {
    if (std::find(parentFrames.begin(), parentFrames.end(), parentAffordance->frame) == parentFrames.end())
    {
      parentFrames.push_back(parentAffordance->frame);
    }
  }


  // Outer loop traverses all parent affordances
  for (const auto& parentFrame : parentFrames)
  {
    const RcsBody* parentAffFrame = RcsGraph_getBodyByName(graph, parentFrame.c_str());
    // RLOG(0, "Checking affordance %s (%d)", parentAffFrame->name, parentAffFrame->id);

    // Inner one loops through all entities
    for (const auto& childCandidate : entities)
    {
      const RcsBody* childCandidateBdy = RcsGraph_getBodyByName(graph, childCandidate.bdyName.c_str());
      RCHECK_MSG(childCandidateBdy, "%s", childCandidate.bdyName.c_str());
      // RLOG(0, "Checking child entity %s (%d)", childCandidate.bdyName.c_str(), childCandidateBdy->id);

      if (childCandidateBdy->parentId == parentAffFrame->id)
      {
        // RLOG(0, "Adding %s", childCandidateBdy->name);
        foundOnes.push_back(&childCandidate);
      }
    }
  }

  // RLOG_CPP(0, "Found " << foundOnes.size() << " children");

  return foundOnes;
}

const AffordanceEntity* ActionScene::getParentAffordanceEntity(const RcsGraph* graph,
                                                               const AffordanceEntity* child) const
{
  std::string errMsg;

  if (!child)
  {
    errMsg = "Entity to be checked for retrieving parent is NULL";
    RLOG_CPP(1, errMsg);
    return NULL;
  }

  const AffordanceEntity* parent = NULL;

  // This is the "root" body of an Affordance entity
  const RcsBody* childBdy = RcsGraph_getBodyByName(graph, child->bdyName.c_str());
  RCHECK(childBdy);   // Should never happen \todo(MG): Remove if tested


  for (const auto& parentCandidate : entities)
  {
    const RcsBody* parentCandidateBdy = RcsGraph_getBodyByName(graph, parentCandidate.bdyName.c_str());
    RCHECK(parentCandidateBdy);

    // \todo(MG): This loop is not needed - the isChild() method does traverse up the tree.
    for (const Affordance* parentAffordance : parentCandidate.affordances)
    {
      const RcsBody* parentAffFrame = RcsGraph_getBodyByName(graph, parentAffordance->frame.c_str());

      if (parentAffFrame && RcsBody_isChild(graph, childBdy, parentAffFrame))
      {
        parent = &parentCandidate;
      }

    }
  }

  return parent;
}

const Manipulator* ActionScene::getManipulator(const std::string& name) const
{
  for (size_t i=0; i<manipulators.size(); ++i)
  {
    // Return first manipulators whose types contains the requested name.
    if (std::find(manipulators[i].types.begin(), manipulators[i].types.end(), name) != manipulators[i].types.end())
    {
      return &manipulators[i];
    }
  }

  return NULL;
}

const Manipulator* ActionScene::getManipulator(const Capability* capability) const
{
  for (size_t i = 0; i < manipulators.size(); ++i)
  {
    for (const auto c : manipulators[i].capabilities)
    {
      if (c==capability)
      {
        return &manipulators[i];
      }
    }
  }

  return NULL;
}

std::vector<const Manipulator*> ActionScene::getManipulatorsOfType(const std::string& type) const
{
  std::vector<const Manipulator*> typeManipulators;

  // Add manipulators whose types contains the requested name.
  for (const auto& m : manipulators)
  {
    if (std::find(m.types.begin(), m.types.end(), type) != m.types.end())
    {
      typeManipulators.push_back(&m);
    }
  }

  return typeManipulators;
}

const Manipulator* ActionScene::getGraspingHand(const RcsGraph* graph,
                                                const AffordanceEntity* entity) const
{
  if (!entity)
  {
    RLOG_CPP(1, "Entity to be checked for being grasped is NULL");
    return NULL;
  }

  const RcsBody* objBdy = RcsGraph_getBodyByName(graph, entity->bdyName.c_str());
  if (!objBdy)
  {
    RLOG_CPP(1, "Object '" << entity->bdyName << "' has no corresponding body in graph");
    return NULL;
  }


  for (const auto& hand : manipulators)
  {
    auto capabilities = getCapabilities<GraspCapability>(&hand);

    for (const auto& grasp : capabilities)
    {
      const RcsBody* graspFrame = RcsGraph_getBodyByName(graph, grasp->frame.c_str());

      if (graspFrame && RcsBody_isChild(graph, objBdy, graspFrame))
      {
        return &hand;
      }

    }
  }

  return NULL;
}

std::tuple<const Manipulator*,const AffordanceEntity*> ActionScene::getGraspingHand(const RcsGraph* graph, std::vector<const AffordanceEntity*> entities) const
{
  if (entities.empty())
  {
    RLOG_CPP(1, "No entity to be checked for being grasped is NULL");
    return std::tuple<const Manipulator*,const AffordanceEntity*>(NULL, NULL);
  }

  for (size_t i=0; i<entities.size(); ++i)
  {
    const Manipulator* graspingHand = getGraspingHand(graph, entities[i]);

    if (graspingHand)
    {
      return std::make_tuple(graspingHand, entities[i]);
    }

  }

  return std::tuple<const Manipulator*,const AffordanceEntity*>(NULL, NULL);
}

const Agent* ActionScene::getAgent(const std::string& name) const
{
  for (size_t i=0; i<agents.size(); ++i)
  {
    if (std::find(agents[i]->types.begin(), agents[i]->types.end(), name) != agents[i]->types.end())
    {
      return agents[i];
    }
  }

  return NULL;
}

Agent* ActionScene::getAgent(const std::string& name)
{
  for (size_t i=0; i<agents.size(); ++i)
  {
    if (std::find(agents[i]->types.begin(), agents[i]->types.end(), name) != agents[i]->types.end())
    {
      return agents[i];
    }
  }

  return NULL;
}

void ActionScene::initializeKinematics(const RcsGraph* graph)
{
  for (auto& m : manipulators)
  {
    m.computeBaseJointName(this, graph);
  }

}

} // namespace aff
