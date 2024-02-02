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

#include "SceneJsonHelpers.h"

#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_typedef.h>
#include <Rcs_shape.h>
#include <Rcs_body.h>
#include <Rcs_timer.h>

#include <mutex>
#include <queue>




namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
void getSceneState(nlohmann::json& stateJson,
                   const ActionScene* scene,
                   const RcsGraph* graph)
{
  std::string logStr;

  // Step 1: Collect agent data including their affordances.
  nlohmann::json agentsJson;
  for (const auto& a : scene->agents)
  {
    logStr += a->name + "; ";
    nlohmann::json agentJson;
    agentJson["name"] = a->name;
    agentJson["instance_id"] = a->instanceId;
    agentJson["types"] = a->types;
    agentJson["visible"] = a->isVisible();

    RcsBody* bdy = RcsGraph_getBodyByName(graph, a->bdyName.c_str());
    RCHECK_MSG(bdy, "%s", a->bdyName.c_str());

    agentJson["position"] = std::vector<double>(bdy->A_BI.org, bdy->A_BI.org+3);
    double ea[3];
    Mat3d_toEulerAngles(ea, bdy->A_BI.rot);
    agentJson["euler_xyzr"] = std::vector<double>(ea, ea+3);

    nlohmann::json manipulatorsJson;
    for (const auto& mName : a->manipulators)
    {
      nlohmann::json manipulatorJson;

      const auto m = scene->getManipulator(mName);
      manipulatorJson["name"] = m->name;
      manipulatorJson["instance_id"] = m->instanceId;
      manipulatorJson["types"] = m->types;
      manipulatorJson["holds_object"] = m->getChildrenOfManipulator(scene, graph, false);
      manipulatorJson["is_held_by_object"] = std::vector<std::string>();

      RcsBody* bdy = RcsGraph_getBodyByName(graph, m->bdyName.c_str());
      RCHECK(bdy);

      manipulatorJson["position"] = std::vector<double>(bdy->A_BI.org, bdy->A_BI.org+3);
      double ea[3];
      Mat3d_toEulerAngles(ea, bdy->A_BI.rot);
      manipulatorJson["euler_xyzr"] = std::vector<double>(ea, ea+3);

      manipulatorsJson += manipulatorJson;

    }
    agentJson["manipulators"] = manipulatorsJson;

    agentsJson += agentJson;
  }
  stateJson["agents"] = agentsJson;
  //RLOG_CPP(1, "Agents: " << logStr);
  logStr.clear();

  // Step 2: Collect affordance entity data
  nlohmann::json entitiesJson;
  for (const auto& e : scene->entities)
  {

    logStr += e.name + "; ";

    nlohmann::json entityJson;
    RcsBody* bdy = RcsGraph_getBodyByName(graph, e.bdyName.c_str());
    RCHECK(bdy);

    // To determine the color, we traverse all body shapes and look for the
    // color of the first "visual" object (the one with the computeType
    // RCSSHAPE_COMPUTE_GRAPHICS). If there is no visible shape, we return
    // "unknown" and emit a warning on debug level 1.
    std::string bodyColor = "unknown";
    for (unsigned int i = 0; i < bdy->nShapes; ++i)
    {
      if (RcsShape_isOfComputeType(&bdy->shapes[i], RCSSHAPE_COMPUTE_GRAPHICS))
      {
        bodyColor = bdy->shapes[i].color;
        break;
      }
    }

    if (bodyColor == "unknown")
    {
      RLOG_CPP(1, "Couldn't find color for entity " << e.name);
    }

    entityJson["name"] = e.name;
    entityJson["instance_id"] = e.instanceId;
    entityJson["types"] = e.types;
    entityJson["color"] = bodyColor;


    // Add shapes with position and Euler angles
    /*

    "salt_bottle": {
        "color": "WHITE",
        "euler_xyzr": [0.0, 0.0, 0.0],
        "fill_level": 0.5,
        "holds_liquid": ["salt"],
        "holds_object": [],
        "is_held_by_object": ["shelf"],
        "position": [0.03, -0.1, 0.971],
        "type": "bottle",
        "volume": 0.5,
        "collision_shapes" : [{"type": "BOX",    "position": [0, 0, 1], "euler_xyzr": [1, 2, 3], "extents":  [1, 2, 3]},
                              {"type": "SPHERE", "position": [0, 0, 1], "euler_xyzr": [1, 2, 3], "extents":  [1, 2, 3]},
                              {"type": "BOX",    "position": [0, 0, 1], "euler_xyzr": [1, 2, 3], "extents":  [1, 2, 3]},
                              {"type": "BOX",    "position": [0, 0, 1], "euler_xyzr": [1, 2, 3], "extents":  [1, 2, 3]},
                              {"type": "BOX",    "position": [0, 0, 1], "euler_xyzr": [1, 2, 3], "extents":  [1, 2, 3]},
                              {"type": "BOX",    "position": [0, 0, 1], "euler_xyzr": [1, 2, 3], "extents":  [1, 2, 3]},
                              {"type": "MESH",   "position": [0, 0, 1], "euler_xyzr": [1, 2, 3], "file":  "meshfile.stl"}]
    },

    */

    nlohmann::json shapeJson;
    RCSBODY_TRAVERSE_SHAPES(bdy)
    {
      if (!RcsShape_isOfComputeType(SHAPE, RCSSHAPE_COMPUTE_PHYSICS))
      {
        continue;
      }

      HTr A_CI;
      HTr_transform(&A_CI, &bdy->A_BI, &SHAPE->A_CB);
      double ea[3];
      Mat3d_toEulerAngles(ea, bdy->A_BI.rot);

      nlohmann::json j2 =
      {
        {"type", RcsShape_name(SHAPE->type)},
        {"position", std::vector<double>(A_CI.org, A_CI.org+3)},
        {"euler_xyzr", std::vector<double>(ea, ea+3)},
        {"extents", std::vector<double>(SHAPE->extents, SHAPE->extents+3)},
      };
      shapeJson += j2;
    }
    entityJson["collision_shapes"] = shapeJson;

    // Only direct children, not the whole sub-tree
    entityJson["holds_object"] = std::vector<std::string>();
    auto collectedChildren = scene->getDirectChildren(graph, &e);
    for (auto child : collectedChildren)
    {
      // We use the name, not the bdyName (instance name) here
      entityJson["holds_object"].push_back(child->name);
    }


    const Manipulator* holdingHand = scene->getGraspingHand(graph, &e);
    const AffordanceEntity* holdingObject = scene->getParentAffordanceEntity(graph, &e);
    entityJson["is_held_by_object"] = std::vector<std::string>();
    std::string parentName = holdingHand ? holdingHand->name : holdingObject ? holdingObject->name : std::string();
    if (!parentName.empty())
    {
      entityJson["is_held_by_object"].push_back(parentName);
    }


    // Populate the transformation field
    {
      entityJson["position"] = std::vector<double>(bdy->A_BI.org, bdy->A_BI.org+3);
      double ea[3];
      Mat3d_toEulerAngles(ea, bdy->A_BI.rot);
      entityJson["euler_xyzr"] = std::vector<double>(ea, ea+3);
    }


    /* Here we populate the "holds_liquid" and volume values
    "tomato_sauce_bottle": {
            "type": "bottle",
            "volume": 0.5,
            "fill_level": 0.5,
            "holds_liquid": ["tomato_sauce"],
            "is_held_by_object": ["table"] <-- Should be held by the table
        },
     */

    entityJson["holds_liquid"] = std::vector<std::string>();
    std::vector<Containable*> containables = getAffordances<Containable>(&e);
    double volume = 0.0;
    double fill_level = 0.0;

    for (auto c : containables)
    {
      volume += c->maxVolume;
      for (const auto& l : c->liquidIngredients)
      {
        entityJson["holds_liquid"].push_back(l.first);
        fill_level += l.second;
      }
    }

    entityJson["fill_level"] = fill_level;
    entityJson["volume"] = volume;

    /* Here we populate the "closure" state. For this, we go through the Affordances and take the
       closure state from the first one we find. We give a warning if we see more than one Containables.
     "fridge": {
       "types": ["furniture"],
       "closure": "closed",
       "power": "on",
       "holds_object": ["milk_bottle", "cranberry_juice_bottle", "pineapple_juice_bottle"],
       "is_held_by_object": ["table"]
     },
      */
    std::vector<Openable*> openables = getAffordances<Openable>(&e);
    if (openables.size()>=1)
    {
      entityJson["closure"].push_back(!openables[0]->isOpen);
      if (openables.size()>1)
      {
        RLOG_CPP(1, "Object " << e.name << " has " << openables.size() <<
                 " Openables - using the closure state from the first one");
      }
    }

    entitiesJson += entityJson;

  }   // for (const auto& e : scene->entities)
  stateJson["entities"] = entitiesJson;

  // RLOG_CPP(1, "affordance entities: " << logStr);
  // RLOG_CPP(2, "JSON: " << stateJson.dump(4));
}

bool isAgentBusy(const std::string& agentName,
                 const ActionScene* scene,
                 const RcsGraph* graph,
                 double distanceThreshold)
{
  const HumanAgent* agent = dynamic_cast<const HumanAgent*>(scene->getAgent(agentName));

  if (!agent)
  {
    return false;
  }

  for (const auto& mName : agent->manipulators)
  {
    const aff::Manipulator* m = scene->getManipulator(mName);

    // if (getCapabilities<GraspCapability>(m).empty())
    // {
    //   continue;
    // }

    if (!m->isOfType("hand"))
    {
      continue;
    }

    // From here on, we know that the manipulator can grasp something
    const RcsBody* hand = RcsGraph_getBodyByName(graph, m->bdyName.c_str());
    RCHECK_MSG(hand, "Couldn't find RcsBody \"%s\" for manipulator \"%s\" of agent \"%s\"",
               m->bdyName.c_str(), m->name.c_str(), agentName.c_str());
    const double* handPos = hand->A_BI.org;

    for (const auto& ntt : scene->entities)
    {
      const RcsBody* nttBdy = RcsGraph_getBodyByName(graph, ntt.bdyName.c_str());

      if (!nttBdy)
      {
        RLOG(1, "No RcsBody found for %s", ntt.bdyName.c_str());
        continue;
      }

      // Ignore very large objects like a table
      double ignoreLarge = false;
      {
        const double d_limit = 1.0;  // 1m
        double xyzMin[3], xyzMax[3];
        bool hasAABB = RcsBody_computeAABB(nttBdy, RCSSHAPE_COMPUTE_DISTANCE, xyzMin, xyzMax);
        if (hasAABB)
        {
          double d_aabb = Vec3d_distance(xyzMin, xyzMax);
          if (d_aabb > d_limit)
          {
            ignoreLarge = true;
          }
        }

      }

      if (ignoreLarge)
      {
        RLOG(1, "Ignoring large object %s", ntt.bdyName.c_str());
        continue;
      }
      // End ignore large objects

      if (Vec3d_distance(hand->A_BI.org, nttBdy->A_BI.org) < distanceThreshold)
      {
        RLOG_CPP(0, agentName << " is busy with the " << nttBdy->name);
        return true;
      }

    }   // scene->entities

  }   // agent->manipulators

  return false;
}

// Returns empty json if no occluded objects exist, or an array of occluded objects
// for the agent: {"occluded": [{"name": "entity name 1", "instance_id": "entity id 1"},
//                              {"name": "entity name 2", "instance_id": "entity id 2"}]}
nlohmann::json getOccludedObjectsForAgent(const std::string& agentName,
                                          const ActionScene* scene,
                                          const RcsGraph* graph)
{
  nlohmann::json json;
  json["occluded"] = std::vector<nlohmann::json>();

  for (const auto& ntt : scene->entities)
  {
    nlohmann::json j = getObjectOccludersForAgent(agentName, ntt.name, scene, graph);
    if (!j["occluded_by"].empty())
    {
      nlohmann::json occludedJson;
      occludedJson["name"] = ntt.name;
      occludedJson["instance_id"] = ntt.instanceId;
      json["occluded"].push_back(occludedJson);
    }

  }

  return json;
}

// Returns empty json if not occluded, or occluding objects sorted by distance
// to eye (increasing): {"occluded_by": ["entity name 1", "entity name 2"] }
nlohmann::json getObjectOccludersForAgent(const std::string& agentName,
                                          const std::string& objectName,
                                          const ActionScene* scene,
                                          const RcsGraph* graph)
{
  nlohmann::json json;
  json["occluded_by"] = std::vector<nlohmann::json>();

  const HumanAgent* agent = dynamic_cast<const HumanAgent*>(scene->getAgent(agentName));

  if (!agent)
  {
    return json;
  }

  std::vector<const Manipulator*> heads = agent->getManipulatorsOfType(scene, "head");
  RCHECK_MSG(heads.size()<=1, "Agent \"%s\" has more than 1 manipulator of type head", agent->name.c_str());

  std::priority_queue<std::pair<double,nlohmann::json>,
      std::vector<std::pair<double,nlohmann::json>>,
      std::greater<std::pair<double,nlohmann::json>> > queue;

  for (const auto& m : heads)
  {
    const RcsBody* head = RcsGraph_getBodyByName(graph, m->bdyName.c_str());
    RCHECK_MSG(head, "Couldn't find RcsBody \"%s\" for head manipulator \"%s\" of agent \"%s\"",
               m->bdyName.c_str(), m->name.c_str(), agentName.c_str());
    const double* eyePos = head->A_BI.org;

    std::vector<const AffordanceEntity*> ntts = scene->getAffordanceEntities(objectName);
    RCHECK_MSG(ntts.size()<=1, "Currently we can't deal with non-unique entities (%s)", objectName.c_str());

    // Loop over all entities with the same name
    for (const auto& goalNtt : ntts)
    {
      const RcsBody* goalBdy = RcsGraph_getBodyByName(graph, goalNtt->bdyName.c_str());

      if (!goalBdy)
      {
        RLOG(1, "No RcsBody found for %s", goalNtt->bdyName.c_str());
        continue;
      }

      if (RcsBody_numDistanceShapes(goalBdy)==0)
      {
        continue;
      }

      double goalPos[3];
      Vec3d_copy(goalPos, goalBdy->A_BI.org);

      double xyzMin[3], xyzMax[3];
      bool hasAABB = RcsBody_computeAABB(goalBdy, RCSSHAPE_COMPUTE_DISTANCE, xyzMin, xyzMax);
      if (hasAABB)
      {
        for (int i=0; i<3; ++i)
        {
          goalPos[i] += 0.5*(xyzMax[i]-xyzMin[i]);
        }
      }

      double dir[3];
      Vec3d_sub(dir, goalPos, eyePos);

      // Create a virtual body that is the line of sight
      RcsBody vBdy;
      memset(&vBdy, 0, sizeof(RcsBody));
      HTr_setIdentity(&vBdy.A_BP);
      HTr_setIdentity(&vBdy.A_BI);
      vBdy.nShapes = 1;
      RcsShape sh;
      RcsShape_init(&sh);
      sh.computeType = RCSSHAPE_COMPUTE_DISTANCE;
      sh.type = RCSSHAPE_SSL;
      Vec3d_set(sh.extents, 0.01, 0.01, Vec3d_getLength(dir));
      vBdy.shapes = &sh;
      Vec3d_copy(vBdy.A_BI.org, eyePos);
      Mat3d_fromVec(vBdy.A_BI.rot, dir, 2);

      for (const auto& checkNtt : scene->entities)
      {
        const RcsBody* checkBdy = RcsGraph_getBodyByName(graph, checkNtt.bdyName.c_str());

        // Ignore self intersections
        if (checkBdy==goalBdy)
        {
          continue;
        }

        if (!checkBdy)
        {
          RLOG(1, "No RcsBody found for %s", checkNtt.bdyName.c_str());
          continue;
        }

        double cp0[3], cp1[3];
        const double d = RcsBody_distance(&vBdy, checkBdy, cp0, cp1, NULL);
        if (d<=0.0)
        {
          NLOG(0, "*** OCCLUSION %s (%s)- %s (%s): d=%f   cp0=%.4f %.4f %.4f   cp1=%.4f %.4f %.4f",
               checkNtt.bdyName.c_str(), checkNtt.name.c_str(), goalNtt->bdyName.c_str(), goalNtt->name.c_str(),
               d, cp0[0], cp0[1], cp0[2], cp1[0], cp1[1], cp1[2]);

          nlohmann::json occluderJson;
          occluderJson["name"] = checkNtt.name;
          occluderJson["instance_id"] = checkNtt.instanceId;
          queue.push(std::make_pair(Vec3d_distance(checkBdy->A_BI.org, eyePos), occluderJson));
        }
        else
        {
          NLOG(1, "--- NO OCCLUSION %s - %s", checkNtt.name.c_str(), goalNtt->name.c_str());
        }
      }

    }   // for (const auto& goalNtt : ntts)

  }   // for (const auto& m : heads)

  // Add the occluders to the json, ordered according to their distance to the eye (closest first)
  while (!queue.empty())
  {
    NLOG_CPP(0, queue.top().first << " " << queue.top().second);
    json["occluded_by"].push_back(queue.top().second);
    queue.pop();
  }

  return json;
}




}   // namespace aff
