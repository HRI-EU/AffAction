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
#include <Rcs_timer.h>

#include <mutex>




namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
void getSceneState(nlohmann::json& stateJson,
                   const ActionScene* scene,
                   const RcsGraph* graph)
{
  std::string fb;
  std::string logStr;

  // Step 1: Collect manipulator data
  for (const auto& m : scene->manipulators)
  {
    logStr += m.name + "; ";

    std::vector<std::string> collectedChildren = m.getChildrenOfManipulator(scene, graph, false);

    stateJson[m.name]["type"] = m.type;
    stateJson[m.name]["holdsObject"] = collectedChildren;
    stateJson[m.name]["isHeldByObject"] = std::vector<std::string>();
  }

  RLOG_CPP(1, "Manipulators: " << logStr);
  logStr.clear();

  // Step 2: Collect affordance entity data
  for (const auto& e : scene->entities)
  {
    logStr += e.name + "; ";


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

    stateJson[e.name]["type"] = e.type;
    stateJson[e.name]["color"] = bodyColor;


    // Add shapes with position and Euler angles
    /*

    "salt_bottle": {
        "color": "WHITE",
        "euler_xyzr": [ 0.0, 0.0, 0.0 ],
        "fillLevel": 0.5,
        "holdsLiquid": [ "salt" ],
        "holdsObject": [],
        "isHeldByObject": [ "shelf" ],
        "position": [ 0.03, -0.1, 0.971 ],
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
    stateJson[e.name]["collision_shapes"] = shapeJson;



    // std::vector<std::string> collectedChildren = getChildrenOfAffordanceEntity(&e);
    // stateJson[e.name]["holdsObject"] = collectedChildren;

    // Only direct children, not the whole sub-tree
    stateJson[e.name]["holdsObject"] = std::vector<std::string>();
    auto collectedChildren = scene->getDirectChildren(graph, &e);
    for (auto child : collectedChildren)
    {
      // We use the name, not the bdyName (instance name) here
      stateJson[e.name]["holdsObject"].push_back(child->name);
    }




    const Manipulator* holdingHand = scene->getGraspingHand(graph, &e);
    const AffordanceEntity* holdingObject = scene->getParentAffordanceEntity(graph, &e);
    stateJson[e.name]["isHeldByObject"] = std::vector<std::string>();
    std::string parentName = holdingHand ? holdingHand->name : holdingObject ? holdingObject->name : std::string();
    if (!parentName.empty())
    {
      stateJson[e.name]["isHeldByObject"].push_back(parentName);
    }


    // Populate the transformation field
    {
      stateJson[e.name]["position"] = std::vector<double>(bdy->A_BI.org, bdy->A_BI.org+3);
      double ea[3];
      Mat3d_toEulerAngles(ea, bdy->A_BI.rot);
      stateJson[e.name]["euler_xyzr"] = std::vector<double>(ea, ea+3);
    }


    /* Here we populate the "holdsLiquid" and volume values
    "tomato_sauce_bottle": {
            "type": "bottle",
            "volume": 0.5,
            "fillLevel": 0.5,
            "holdsLiquid": [ "tomato_sauce" ],
            "isHoldByObject": [
                "table" <-- Should be held by the table
            ]
        },
     */

    stateJson[e.name]["holdsLiquid"] = std::vector<std::string>();
    std::vector<Containable*> containables = getAffordances<Containable>(&e);
    double volume = 0.0;
    double fillLevel = 0.0;

    for (auto c : containables)
    {
      volume += c->maxVolume;
      for (const auto& l : c->liquidIngredients)
      {
        stateJson[e.name]["holdsLiquid"].push_back(l.first);
        fillLevel += l.second;
      }
    }

    stateJson[e.name]["fillLevel"] = fillLevel;
    stateJson[e.name]["volume"] = volume;

    /* Here we populate the "closure" state. For this, we go throug the Affordances and take the closure state
       from the first one we find. We give a warning if we see more than one Containables.
     "fridge": {
       "type": "furniture",
       "closure": "closed",
       "power": "on",
       "holdsObject": [ "milk_bottle", "cranberry_juice_bottle", "pineapple_juice_bottle", "tomato_juice_bottle" ],
       "isHeldByObject": ["table"]
     },
      */
    std::vector<Openable*> openables = getAffordances<Openable>(&e);
    if (openables.size()>=1)
    {
      stateJson[e.name]["closure"].push_back(!openables[0]->isOpen);
      if (openables.size()>1)
      {
        RLOG_CPP(1, "Object " << e.name << " has " << openables.size() <<
                 " Openables - using the closure state from the first one");
      }
    }






  }   // for (const auto& e : scene->entities)


  // Step 3: Collect gaze data

  /* Here we populate the gaze values. The fovea object is the one that has
     been looked at with the ActionGaze. If no gaze action was sent, this
     field is empty. The perif part contains all other obejcts for now.

  "gaze": {
    "fovea": ["table"],
    "perif": ["pizza_dough_big_plate","mozzarella_cheese_bowl","garlic_bowl",...],
    "isHeldByObject": ["robot"]
  },
      */
  {
    stateJson["gaze"]["fovea"] = std::vector<std::string>();
    if (!scene->foveatedEntity.empty())
    {
      stateJson["gaze"]["fovea"].push_back(scene->foveatedEntity);
    }

    stateJson["gaze"]["perif"] = std::vector<std::string>();
    for (const auto& e : scene->entities)
    {
      if (scene->foveatedEntity!=e.name)
      {
        stateJson["gaze"]["perif"].push_back(e.name);
      }
    }

    stateJson["gaze"]["isHeldByObject"].push_back("robot");
  }

  // TODO (CP) - currently hardcoded
  // Step 4: Collect lab data

  stateJson["table"]["isHeldByObject"].push_back("lab");

  // "lab": {
  //   "type": "room",
  //   "holdsObject": ["table"],
  //   "isHeldByObject" : []
  // }
  {
    stateJson["lab"]["type"] = "room";
    stateJson["lab"]["holdsObject"] = std::vector<std::string>({"table", "shelf"});
    stateJson["lab"]["isHeldByObject"] = std::vector<std::string>();
  }
  // TODO (CP) - currently hardcoded
  // Step 4: Collect robot data

  // "robot": {
  //   "type": "robot",
  //   "holdsObject": ["gaze","hand_left","hand_right"],
  //   "isHeldByObject": ["lab"]
  // }
  {
    stateJson["robot"]["type"] = "robot";
    stateJson["robot"]["holdsObject"] = std::vector<std::string>({"gaze", "hand_left", "hand_right"});
    stateJson["robot"]["isHeldByObject"] = std::vector<std::string>({"lab"});
  }

  RLOG_CPP(1, "affordance entities: " << logStr);
  RLOG_CPP(1, "JSON: " << stateJson.dump(4));
}

}   // namespace aff
