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

#include "PickAndPlaceConstraint.h"
#include "ConstraintFactory.h"

#include <Rcs_body.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utils.h>
#include <Rcs_VecNd.h>

#include <algorithm>
#include <unordered_set>



namespace tropic
{
REGISTER_CONSTRAINT(PickAndPlaceConstraint);

PickAndPlaceConstraint::PickAndPlaceConstraint(double t, const std::string& parent_) :
  GraphConstraint(), gripperName(parent_), attachTime(t), active(true)
{
  setClassName("PickAndPlaceConstraint");
  HTr_setZero(&this->attachToTrf);
}

PickAndPlaceConstraint::PickAndPlaceConstraint(xmlNode* node) :
  GraphConstraint(), attachTime(0.0), active(true)
{
  setClassName("PickAndPlaceConstraint");
  HTr_setZero(&this->attachToTrf);
  fromXML(node);
}

PickAndPlaceConstraint::PickAndPlaceConstraint(const PickAndPlaceConstraint& other) :
  GraphConstraint(other), gripperName(other.gripperName),
  attachTime(other.attachTime), active(other.active), attachToTrf(other.attachToTrf)
{
}

PickAndPlaceConstraint* PickAndPlaceConstraint::clone() const
{
  PickAndPlaceConstraint* tSet = new PickAndPlaceConstraint(attachTime, gripperName);
  tSet->constraint = constraint;
  tSet->className = className;
  tSet->attachToTrf = attachToTrf;

  for (size_t i = 0; i < children.size(); ++i)
  {
    tSet->add(std::shared_ptr<ConstraintSet>(children[i]->clone()));
  }

  return tSet;
}

PickAndPlaceConstraint::~PickAndPlaceConstraint()
{
}

// parent - has child: disconnect to children's closest non-tree body
// parent - has no child: connect closest non-tree body to parent
void PickAndPlaceConstraint::findParentChild(int& parentId, int& childId) const
{
  const RcsBody* parent = RcsGraph_getBodyByName(graph, gripperName.c_str());



  // Collect all body ids of the graph
  std::unordered_set<int> bdyIdSet;
  for (unsigned int i = 0; i < graph->nBodies; ++i)
  {
    const RcsBody* bdy = &graph->bodies[i];

    if (bdy->rigid_body_joints
        //&& (bdy->m > 0.0)
        && (bdy->id != -1))
    {
      bdyIdSet.insert(bdy->id);
    }
  }



  // Case 1: "Put". A child with rigid body joints and a mass is attached to parent:
  //         We detach it and attach it to the closest object. This means that
  //         the child id is found as the rigid object non-zero mass child of the
  //         parent, and the parent id as the closest non-tree rigid body object
  childId = -1;
  RCSBODY_TRAVERSE_CHILD_BODIES(graph, parent)
  {
    if (BODY->rigid_body_joints && BODY->m > 0.0)
    {
      childId = BODY->id;
      break;
    }
  }

  if (childId != -1)
  {
    // Erase child from parents to be considered
    auto iter = bdyIdSet.find(childId);
    if (iter != bdyIdSet.end())
    {
      bdyIdSet.erase(iter);
    }

    double d_min = DBL_MAX;
    int id_min = -1;
    for (const auto& bid : bdyIdSet)
    {
      const double d = Vec3d_distance(graph->bodies[childId].A_BI.org,
                                      graph->bodies[bid].A_BI.org);
      if (d < d_min)
      {
        d_min = d;
        id_min = bid;
      }
    }

    RCHECK(id_min != -1);
    parentId = id_min;
    RLOG(5, "Closest body to %s: %s with d = %f",
         RCSBODY_NAME_BY_ID(graph, childId), RCSBODY_NAME_BY_ID(graph, parentId), d_min);
    return;
  }


  // Case 2: "Get". Nothing attached to parent - we connect the closest dynamic
  // rigid body to the parent. We erase all bodies with zero mass, since we assume
  // that only objects with a mass can be gotten.
  for (auto it = bdyIdSet.begin(); it != bdyIdSet.end();)
  {
    if (graph->bodies[*it].m == 0.0)
    {
      it = bdyIdSet.erase(it);
    }
    else
    {
      ++it;
    }
  }

  // Find closest body. We use the origin here for computational efficiency
  double d_min = DBL_MAX;
  int id_min = -1;
  for (const auto& bid : bdyIdSet)
  {
    const double d = Vec3d_distance(parent->A_BI.org, graph->bodies[bid].A_BI.org);
    if (d < d_min)
    {
      d_min = d;
      id_min = bid;
    }
  }

  RCHECK(id_min != -1);
  RcsBody* child = &graph->bodies[id_min];
  RLOG(5, "Closest body to %s: %s with d = %f", parent->name, child->name, d_min);
  parentId = parent ? parent->id : -1;
  childId = child ? child->id : -1;
}

double PickAndPlaceConstraint::compute(double dt)
{
  attachTime -= dt;

  if ((attachTime<0.0) && (attachTime>=-dt))
  {
    int parentId = -1, childId = -1;
    findParentChild(parentId, childId);
    RcsBody* child = RCSBODY_BY_ID(graph, childId);

    if (!child)
    {
      RLOG(1, "Can't find child to connect to parent %s",
           gripperName.c_str());
    }
    else
    {
      RLOG(5, "Appending \"%s\" to \"%s\"", child->name, gripperName.c_str());

      // In case there is no parent, we connect the body to -1
      RcsBody* parent = RCSBODY_BY_ID(graph, parentId);

      HTr tmp;
      HTr_copy(&tmp, &child->A_BI);

      // In case an attachment transform has been set, we assume it to be
      // represented in the child's frame of reference. We therefore first
      // transform it into world coordinates and then call the attach body
      // function.
      if (!VecNd_isZero((double*)&attachToTrf, 12))
      {
        //HTr_copy(&child->A_BI, &attachToTrf);
        const HTr* A_PI = parent ? &parent->A_BI : HTr_identity();
        HTr_transform(&child->A_BI, A_PI, &attachToTrf);
      }

      if (childId != -1)
      {
        bool success = RcsBody_attachToBodyId(graph, childId, parentId);
        RCHECK(success);   // False for child->id == -1
      }

      this->active = false;
      HTr_copy(&child->A_BI, &tmp);
    }

  }

  return GraphConstraint::compute(dt);
}

bool PickAndPlaceConstraint::inUse() const
{
  return this->active;
}

double PickAndPlaceConstraint::getStartTimeRecurse() const
{
  double startTime = ConstraintSet::getStartTimeRecurse();
  startTime = std::min(attachTime, startTime);
  return startTime < 0.0 ? 0.0 : startTime;
}

double PickAndPlaceConstraint::getEndTime() const
{
  return std::max(attachTime, ConstraintSet::getEndTime());
}

void PickAndPlaceConstraint::fromXML(xmlNode* node)
{
  if (isXMLNodeName(node, "ConstraintSet") == false)
  {
    throw ("XML node is not a \"ConstraintSet\" - giving up");
  }

  bool success = Rcs::getXMLNodePropertySTLString(node, "gripper", gripperName);
  success = getXMLNodePropertyDouble(node, "t", &attachTime) && success;

  active = (attachTime>0.0) ? true : false;

  node = node->children;

  while (node)
  {
    add(ConstraintFactory::create(node));
    node = node->next;
  }

}

void PickAndPlaceConstraint::toXML(std::ostream& outStream, size_t indent) const
{
  // Prepare indentation string so that hierarchy levels are indented nicely
  std::string indStr(indent, ' ');

  // Open set's xml description. The class name is polymorphic
  outStream << indStr << "<ConstraintSet type=\"" << getClassName() << "\" ";

  // Write out information to top-level tag
  outStream << "t=\"" << attachTime << "\" ";
  outStream << "gripper=\"" << gripperName << "\" ";

  // Add constraint ids
  outStream << getIdsForXML();

  // If there are no children, we close the tag in the first line
  if (children.empty())
  {
    outStream << " />" << std::endl;
  }
  // Go recursively through all child sets if any
  else
  {
    outStream << " >" << std::endl << std::endl;

    for (size_t i=0; i< children.size(); ++i)
    {
      children[i]->toXML(outStream, indent+2);
    }
    outStream << indStr << "</ConstraintSet>" << std::endl << std::endl;
  }

}

void PickAndPlaceConstraint::setConnectTransform(const HTr* A_BI)
{
  HTr_copy(&attachToTrf, A_BI);
}


}   // namespace tropic
