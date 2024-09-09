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
#include <Rcs_joint.h>
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
#include <cfloat>

/*

The Manipulator class is composed of Capabilities, which model how
a manipulator can physically interact with an affordance model.

*/

namespace aff
{
/*******************************************************************************
 *
 ******************************************************************************/

Manipulator::Manipulator() : reach(0.0)
{
}

Manipulator::Manipulator(const xmlNodePtr node, const std::string& groupSuffix) : SceneEntity(node, groupSuffix), reach(0.0)
{
  xmlNodePtr child = node->children;

  while (child)
  {
    Capability* c = nullptr;

    if (isXMLNodeNameNoCase(child, "Joints"))
    {
      fingerJoints = Rcs::getXMLNodePropertyVecSTLString(child, "names");
    }
    else if (isXMLNodeNameNoCase(child, "PowergraspCapability"))
    {
      c = new PowergraspCapability(child);
    }
    else if (isXMLNodeNameNoCase(child, "PincergraspCapability"))
    {
      c = new PincergraspCapability(child);
    }
    else if (isXMLNodeNameNoCase(child, "TwistgraspCapability"))
    {
      c = new TwistgraspCapability(child);
    }
    else if (isXMLNodeNameNoCase(child, "CirculargraspCapability"))
    {
      c = new CirculargraspCapability(child);
    }
    else if (isXMLNodeNameNoCase(child, "PalmgraspCapability"))
    {
      c = new PalmgraspCapability(child);
    }
    else if (isXMLNodeNameNoCase(child, "FingerpushCapability"))
    {
      c = new FingerpushCapability(child);
    }
    else if (isXMLNodeNameNoCase(child, "GazeCapability"))
    {
      c = new GazeCapability(child);
    }

    if (c)
    {
      c->frame += groupSuffix;
      capabilities.push_back(c);
    }

    child = child->next;
  }

}

Manipulator::Manipulator(const Manipulator& other) : SceneEntity(other),
  baseJointName(other.baseJointName), fingerJoints(other.fingerJoints), reach(other.reach)
{
  for (size_t i=0; i<other.capabilities.size(); ++i)
  {
    Capability* ci = other.capabilities[i]->clone();
    capabilities.push_back(ci);
  }

}

Manipulator& Manipulator::operator= (const Manipulator& copyFromMe)
{
  if (this == &copyFromMe)
  {
    return *this;
  }

  SceneEntity::operator=(copyFromMe);
  baseJointName = copyFromMe.baseJointName;
  fingerJoints = copyFromMe.fingerJoints;
  reach = copyFromMe.reach;

  for (size_t i=0; i<capabilities.size(); ++i)
  {
    delete capabilities[i];
  }
  capabilities.clear();

  for (size_t i=0; i<copyFromMe.capabilities.size(); ++i)
  {
    capabilities.push_back(copyFromMe.capabilities[i]->clone());
  }

  return *this;
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
  std::cout << "Manipulator '" << name << "' (body: '" << bdyName << "') has "
            << capabilities.size() << " capabilities:" << std::endl;
  std::cout << "Reach is " << reach << " m, base joint (shoulder) is '"
            << baseJointName << "'" << std::endl;
  for (auto c : capabilities)
  {
    c->print();
    std::cout << std::endl;
  }
}

bool Manipulator::check(const RcsGraph* graph) const
{
  bool success = true;

  const RcsBody* manipulatorBody = RcsGraph_getBodyByName(graph, bdyName.c_str());
  if (!manipulatorBody)
  {
    RLOG(1, "No graph body with name '%s' was found", bdyName.c_str());
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
  std::vector<std::string> capabilityFrames;
  for (auto c : capabilities)
  {
    capabilityFrames.push_back(c->frame);
  }

  // Traverse through all bodies that are children of the manipulator
  RCSBODY_TRAVERSE_CHILD_BODIES(graph, body(graph))
  {
    // We ignore bodies that have no shape or only a frame shape, since
    // these typically define capabilities and not objects to interact.
    if (((BODY->nShapes==1) && (BODY->shapes[0].type==RCSSHAPE_REFFRAME)) ||
        (BODY->nShapes==0))
    {
      continue;
    }

    // If a body in the hand's children is not a capability frame, it is something
    // else that we consider to occupy the manipulator. We can then return early.
    if (std::find(capabilityFrames.begin(), capabilityFrames.end(), BODY->name) == capabilityFrames.end())
    {
      NLOG(0, "Manipulator %s is occupied with body %s", name.c_str(), BODY->name);
      return false;
    }

  }   // RCSBODY_TRAVERSE_CHILD_BODIES

  return true;
}

size_t Manipulator::getNumFingers() const
{
  return fingerJoints.size();
}

HTr Manipulator::getBaseJointTransform(const RcsGraph* graph) const
{
  return getBaseJoint(graph)->A_JI;
}

const RcsJoint* Manipulator::getBaseJoint(const RcsGraph* graph) const
{
  const RcsJoint* baseJnt = RcsGraph_getJointByName(graph, baseJointName.c_str());
  RCHECK_MSG(baseJnt, "Manipulator '%s' ('%s'): Base joint '%s' not found in graph",
             name.c_str(), bdyName.c_str(), baseJointName.c_str());
  return baseJnt;
}

const RcsBody* Manipulator::getBaseJointBody(const RcsGraph* graph) const
{
  const RcsJoint* baseJnt = getBaseJoint(graph);

  int bdyId = RcsJoint_getConnectedBodyId(graph, baseJnt);
  const RcsBody* bdy = RCSBODY_BY_ID(graph, bdyId);
  RCHECK_MSG(bdy, "Joint \"%s\" not connected to body", baseJnt->name);

  const RcsBody* baseBdy = RCSBODY_BY_ID(graph, bdy->parentId);
  RCHECK_MSG(baseBdy, "Body \"%s\" has no parent", bdy->name);

  return baseBdy;
}

std::vector<double> Manipulator::fingerAnglesFromFingerTipDistance(double fingerTipDistanceInMeters) const
{
  if (isOfType("ROBOTIQ-2F140"))
  {
    // 0: fully open: 128.6mm (140)
    // 40 deg: fully closed

    // 140 -> 0
    // 0 -> RCS_DEG2RAD(40)
    return std::vector<double>(1, RCS_DEG2RAD(40.0)*(1.0-fingerTipDistanceInMeters/0.14));
  }

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

  NLOG(0, "Returning %s - %s: %f", c_min ? c_min->frame.c_str() : "NULL", a_min ? a_min->frame.c_str() : "NULL", d_min);
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

std::string Manipulator::getGazingFrame() const
{
  for (auto c : capabilities)
  {
    //c->print();
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

  const RcsBody* hand = RcsGraph_getBodyByName(graph, bdyName.c_str());
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

  RcsBody* hand = RcsGraph_getBodyByName(graph, bdyName.c_str());
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

void Manipulator::computeBaseJointName(const ActionScene* scene,
                                       const RcsGraph* graph)
{
  auto graspCapabilities = getCapabilities<GraspCapability>(this);
  auto gazeCapabilities = getCapabilities<GazeCapability>(this);

  if (graspCapabilities.empty() && gazeCapabilities.empty())
  {
    return;
  }

  // Now we have a grasping hand
  const RcsBody* ee = body(graph);

  // Find "driving" joint of the body
  RcsJoint* jnt = RcsBody_lastJointBeforeBody(graph, ee);
  RCHECK_MSG(jnt, "Body '%s' has no joint", bdyName.c_str());

  this->reach = Vec3d_distance(ee->A_BI.org, jnt->A_JI.org);
  for (const auto& capability : graspCapabilities)
  {
    ee = RcsGraph_getBodyByName(graph, capability->frame.c_str());
    RCHECK(ee);
    this->reach = std::max(this->reach, Vec3d_distance(ee->A_BI.org, jnt->A_JI.org));
    RLOG_CPP(5, "Reach by " << ee->name << " is " << reach);
  }

  // Traverse backwards from end effector and collect all unconstrained joints
  std::vector<const RcsJoint*> jnts;
  while (jnt)
  {
    if (!jnt->constrained)
    {
      jnts.push_back(jnt);
    }
    jnt = RCSJOINT_BY_ID(graph, jnt->prevId);
  }

  // Calculate reach as sum of segments
  const RcsJoint* baseJnt = nullptr;
  if (!jnts.empty())
  {
    baseJnt = jnts.back();
    this->baseJointName = baseJnt->name;
    for (size_t i = 1; i < jnts.size(); ++i)
    {
      this->reach += Vec3d_distance(jnts[i-1]->A_JI.org, jnts[i]->A_JI.org);
    }
  }

  // Here we have a reach for the end effector
  RLOG(5, "[%s]: basejointname is %s, Reach is %f",
       name.c_str(), baseJointName.c_str(), reach);
}

bool Manipulator::canReachTo(const ActionScene* scene,
                             const RcsGraph* graph,
                             const double position[3]) const
{
  // No grasp capabilities - no reaching
  if (getCapabilities<GraspCapability>(this).empty())
  {
    RLOG(1, "Manipulator '%s' ('%s') has no GraspCapability", name.c_str(), bdyName.c_str());
    return false;
  }

  bool reachable = false;

  const RcsJoint* baseJnt = getBaseJoint(graph);
  const double objDistance = Vec3d_distance(position, baseJnt->A_JI.org);

  if (objDistance < reach)
  {
    reachable = true;
  }

  RLOG(1, "objDistance=%f   reach=%f", objDistance, reach);

  return reachable;
}

} // namespace aff
