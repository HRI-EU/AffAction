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

#include "Manipulator.h"
#include "AffordanceEntity.h"
#include "ActionScene.h"
#include "Capability.h"

#include <Rcs_typedef.h>
#include <Rcs_shape.h>
#include <Rcs_parser.h>
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

The Manipulator class is composed of Capabilities, which model how
a manipulator can physically interact with an affordance model.

*/

namespace aff
{
/*******************************************************************************
 *
 ******************************************************************************/

Manipulator::Manipulator()
{
}

Manipulator::Manipulator(const xmlNodePtr node)
{
  name = Rcs::getXMLNodePropertySTLString(node, "name");
  id = Rcs::getXMLNodePropertySTLString(node, "id");
  type = Rcs::getXMLNodePropertySTLString(node, "type");

  if (name.empty()) 
  {
    RFATAL("Manipulator name not provided.");
  }

  if (type.empty())
  {
    RFATAL("Manipulator type not provided: `%s`", name.c_str());
  } 

  RLOG(5, "Adding Manipulator with name=%s id=%s type=%s",
       name.c_str(), id.c_str(), type.c_str());

  xmlNodePtr child = node->children;

  while (child)
  {
    if (isXMLNodeNameNoCase(child, "Joints"))
    {
      fingerJoints = Rcs::getXMLNodePropertyVecSTLString(child, "names");
    }
    else if (isXMLNodeNameNoCase(child, "PowergraspCapability"))
    {
      capabilities.push_back(new PowergraspCapability(child));
    }
    else if (isXMLNodeNameNoCase(child, "PincergraspCapability"))
    {
      capabilities.push_back(new PincergraspCapability(child));
    }
    else if (isXMLNodeNameNoCase(child, "TwistgraspCapability"))
    {
      capabilities.push_back(new TwistgraspCapability(child));
    }
    else if (isXMLNodeNameNoCase(child, "CirculargraspCapability"))
    {
      capabilities.push_back(new CirculargraspCapability(child));
    }
    else if (isXMLNodeNameNoCase(child, "PalmgraspCapability"))
    {
      capabilities.push_back(new PalmgraspCapability(child));
    }
    else if (isXMLNodeNameNoCase(child, "FingerpushCapability"))
    {
      capabilities.push_back(new FingerpushCapability(child));
    }
    else if (isXMLNodeNameNoCase(child, "GazeCapability"))
    {
      capabilities.push_back(new GazeCapability(child));
    }

    child = child->next;
  }

}

Manipulator::Manipulator(const Manipulator& other) :
  name(other.name), id(other.id), type(other.type), fingerJoints(other.fingerJoints)
{
  for (size_t i=0; i<other.capabilities.size(); ++i)
  {
    Capability* ci = other.capabilities[i]->clone();
    NLOG(0, "ci is of type %s", ci->type.c_str());
    capabilities.push_back(ci);
  }

}

Manipulator::~Manipulator()
{
  for (auto c : capabilities)
  {
    delete c;
  }
}

void Manipulator::print() const
{
  std::cout << "Manipulator " << name << " has " << capabilities.size()
            << " capabilities:" << std::endl;
  for (auto c : capabilities)
  {
    c->print();
    std::cout << std::endl;
  }
}

bool Manipulator::check(const RcsGraph* graph) const
{
  bool success = true;
  const RcsBody* manipulatorBody = RcsGraph_getBodyByName(graph, name.c_str());
  if (!manipulatorBody)
  {
    RLOG(1, "No graph body with name '%s' was found", name.c_str());
    success = false;
  }

  for (auto c : capabilities)
  {
    success = c->check(graph) && success;
  }

  for (auto& f : fingerJoints)
  {
    const RcsJoint* jnt = RcsGraph_getJointByName(graph, f.c_str());
    if (!jnt)
    {
      RLOG_CPP(1, "Failed to find joint for joint '" << f << "'");
      success = false;
    }
  }

  // Check that all Capability frames are children of the Manipulator.
  for (auto c : capabilities)
  {
    if (!c)
    {
      RLOG(1, "Found NULL capability in ActionScene");
      return false;
    }

    // cBdy does exist, this has been checked above.
    const RcsBody* cBdy = RcsGraph_getBodyByName(graph, c->frame.c_str());
    if (manipulatorBody && (cBdy->parentId != manipulatorBody->id))
      //RLOG(0, "Checking if %s is a child of %s", cBdy->name, manipulatorBody->name);
      //if (!RcsBody_isChild(graph, cBdy, manipulatorBody))
    {
      if (dynamic_cast<FingerpushCapability*>(c) || dynamic_cast<CirculargraspCapability*>(c))
      {
        RLOG_CPP(0, "Ignoring parent-child requirement for " << c->className);
      }
      else
      {
        RLOG_CPP(1, "Capability frame '" << c->frame << "' is not a child of manipulator '"
                 << name << "'");
        success = false;
      }
    }
  }

  return success;
}

bool Manipulator::isEmpty(const RcsGraph* graph) const
{
  // mHand is guaranteed to be valid after checking the domain.
  RcsBody* mHand = RcsGraph_getBodyByName(graph, name.c_str());

  // Traverse through all bodies that are children of the manipulator
  // (including the manipulator itself).
  RCSBODY_TRAVERSE_CHILD_BODIES(graph, mHand)
  {
    // We ignore bodies that have only a frame shape, since these typically
    // define capabilities and not objects to interact.
    if ((BODY->nShapes==1) && (BODY->shapes[0].type==RCSSHAPE_REFFRAME))
    {
      continue;
    }

    bool isOwnChild = false;

    // Traversing all grasps of the manipulator. If the currently traversed
    // body corresponds to one of the grasp frames, the manipulator is not
    // considered as occupied.
    for (auto c : capabilities)
    {
      if (!dynamic_cast<GraspCapability*>(c))
      {
        continue;
      }

      if (STREQ(BODY->name, c->frame.c_str()))
      {
        isOwnChild = true;
        break;
      }
    }

    if (!isOwnChild)
    {
      //RLOG(0, "\"%s\" is held in hand", BODY->name);
      return false;
    }
  }

  return true;
}

size_t Manipulator::getNumFingers() const
{
  return fingerJoints.size();
}

std::vector<double> Manipulator::fingerAnglesFromFingerTipDistance(double fingerTipDistanceInMeters) const
{
  return std::vector<double>(3, 1.0-fingerTipDistanceInMeters/0.175);
}

std::tuple<Capability*, Affordance*, double> Manipulator::getGrasp(const RcsGraph* graph,
                                                                   const AffordanceEntity* entity) const
{
  const RcsBody* object = RcsGraph_getBodyByName(graph, entity->bdyName.c_str());

  if (!object)
  {
    return std::tuple<Capability*, Affordance*,double>(NULL, NULL, 0.0);
  }

  double d_min = DBL_MAX;
  Capability* c_min = NULL;
  Affordance* a_min = NULL;

  for (auto c : capabilities)
  {
    if (dynamic_cast<GraspCapability*>(c))
    {
      RcsBody* graspFrm = RcsGraph_getBodyByName(graph, c->frame.c_str());
      RCHECK(graspFrm);   // Never happens after initial graph check
      if (object->parentId == graspFrm->id)
      {
        for (auto a : entity->affordances)
        {
          RcsBody* affordanceFrm = RcsGraph_getBodyByName(graph, a->frame.c_str());
          RCHECK(affordanceFrm);   // Never happens after initial graph check

          //double dist = Mat3d_diffAngle(graspFrm->A_BI.rot, affordanceFrm->A_BI.rot);
          double dist = Vec3d_diffAngle(&graspFrm->A_BI.org[2], &affordanceFrm->A_BI.org[2]);
          dist += Vec3d_distance(graspFrm->A_BI.org, affordanceFrm->A_BI.org);

          if (dist < d_min)
          {
            d_min = dist;
            c_min = c;
            a_min = a;
          }

          RLOG(5, "Checking %s - %s: %f", c->frame.c_str(), a->frame.c_str(), dist);
        }

      }
    }

  }

  RLOG(0, "Returning %s - %s: %f", c_min ? c_min->frame.c_str() : "NULL", a_min ? a_min->frame.c_str() : "NULL", d_min);
  return std::tuple<Capability*, Affordance*,double>(c_min, a_min, d_min);
}

std::vector<const Affordance*> Manipulator::getGraspAffordances(const RcsGraph* graph,
                                                                const AffordanceEntity* entity,
                                                                double dist) const
{
  std::vector<const Affordance*> res;
  std::string graspFrmName = getGraspingFrame(graph, entity);
  const RcsBody* graspFrm = RcsGraph_getBodyByName(graph, graspFrmName.c_str());

  if (!graspFrm)
  {
    return res;
  }

  for (auto a : entity->affordances)
  {
    if (dynamic_cast<Graspable*>(a))
    {
      const RcsBody* affordanceFrm = RcsGraph_getBodyByName(graph, a->frame.c_str());
      RCHECK(affordanceFrm);

      if (Vec3d_distance(graspFrm->A_BI.org, affordanceFrm->A_BI.org)<dist)
      {
        res.push_back(a);
      }

    }

  }

  return res;
}

const Capability* Manipulator::getGraspingCapability(const RcsGraph* graph,
                                                     const AffordanceEntity* entity) const
{
  const RcsBody* object = RcsGraph_getBodyByName(graph, entity->bdyName.c_str());
  RCHECK(object);

  for (auto c : capabilities)
  {
    if (dynamic_cast<GraspCapability*>(c))
    {
      const RcsBody* graspFrm = RcsGraph_getBodyByName(graph, c->frame.c_str());
      RCHECK(graspFrm);
      if (object->parentId==graspFrm->id)
      {
        return c;
      }
    }

  }

  return NULL;
}

std::string Manipulator::getGraspingFrame(const RcsGraph* graph,
                                          const AffordanceEntity* entity) const
{
  const Capability* c = getGraspingCapability(graph, entity);
  return c ? c->frame : std::string();
}

std::string Manipulator::getGazingFrame(const RcsGraph* graph) const
{
  for (auto c : capabilities)
  {
    c->print();
    if (dynamic_cast<GazeCapability*>(c))
    {
      return c->frame;
    }
  }

  return std::string();
}

std::vector<const AffordanceEntity*> Manipulator::getGraspedEntities(const ActionScene& scene,
                                                                     const RcsGraph* graph) const
{
  std::vector<const AffordanceEntity*> foundOnes;

  const RcsBody* hand = RcsGraph_getBodyByName(graph, name.c_str());
  RCHECK(hand);   // Should never happen \todo(MG): Remove if tested

  for (const auto& childCandidate : scene.entities)
  {
    const RcsBody* childCandidateBdy = RcsGraph_getBodyByName(graph, childCandidate.bdyName.c_str());
    RCHECK(childCandidateBdy);

    if (RcsBody_isChild(graph, childCandidateBdy, hand))
    {
      foundOnes.push_back(&childCandidate);
    }

  }

  return foundOnes;
}

// If instanceName is false, the name of
// the child entities is returned. Otherwise, it is their RcsBody name. This
// is usually the same, but different if there are several entities with the
// same name. We can have identical entity names, but require unique body
// names.
// \todo: This is pretty much the same as getGraspingFrame()
std::vector<std::string> Manipulator::getChildrenOfManipulator(const ActionScene* scene,
                                                               const RcsGraph* graph,
                                                               bool useInstanceName) const
{
  std::vector<std::string>  collectedChildren;

  RcsBody* hand = RcsGraph_getBodyByName(graph, name.c_str());
  RCHECK(hand);

  RCSBODY_TRAVERSE_BODIES(graph, hand)
  {
    for (const auto& e : scene->entities)
    {
      // Compare current graph body against all AffordanceEntities and find match
      if (std::string(BODY->name) == e.bdyName)
      {
        std::string res = useInstanceName ? e.bdyName : e.name;
        collectedChildren.push_back(res);
      }
    }
  }

  return collectedChildren;
}

} // namespace aff