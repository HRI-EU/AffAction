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

#include "AffordanceEntity.h"
#include "SceneJsonHelpers.h"
#include "json.hpp"

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

Entity component model for affordance. An affordance model (object) is
composed of a set of affordances, each describing the physical effect
other objects or effectors can do with it. The object is the entity,
the components are the affordances. Similarly, the Manipulator class
is composed of Capabilities, which model how a manipulator can
physically interact with an affordance model.

*/

namespace aff
{

auto space_fold = [](std::string a, std::string b)
{
  return std::move(a) + ' ' + b;
};

std::string join_strings(const std::vector<std::string>& strings)
{
  return std::accumulate(std::next(strings.begin()), strings.end(), strings[0], space_fold);
}

/*******************************************************************************
 *
 ******************************************************************************/
SceneEntity::SceneEntity()
{
}

SceneEntity::SceneEntity(const xmlNodePtr node)
{
  bdyName = Rcs::getXMLNodePropertySTLString(node, "body");
  RCHECK_MSG(!bdyName.empty(), "Found SceneEntity without body name: %s", node->name);
  name = bdyName;
  instanceId = bdyName;
  Rcs::getXMLNodePropertySTLString(node, "name", name);
  Rcs::getXMLNodePropertySTLString(node, "instance_id", instanceId);
  types = Rcs::getXMLNodePropertyVecSTLString(node, "types");

  // Add name and instanceId as additional types.
  if (std::find(types.begin(), types.end(), name) == types.end())
  {
    types.push_back(name);
  }

  if (std::find(types.begin(), types.end(), instanceId) == types.end())
  {
    types.push_back(instanceId);
  }

  RLOG(5, "Adding SceneEntity with name=%s bdyName=%s instanceId=%s types=%s",
       name.c_str(), bdyName.c_str(), instanceId.c_str(), join_strings(types).c_str());
}

SceneEntity::~SceneEntity()
{
}

const RcsBody* SceneEntity::body(const RcsGraph* graph) const
{
  RcsBody* b = RcsGraph_getBodyByName(graph, bdyName.c_str());
  RCHECK_MSG(b, "SceneEntity %s (body %s) has no body attached",
             name.c_str(), bdyName.c_str());
  return b;
}

RcsBody* SceneEntity::body(RcsGraph* graph)
{
  RcsBody* b = RcsGraph_getBodyByName(graph, bdyName.c_str());
  RCHECK_MSG(b, "SceneEntity %s (body %s) has no body attached",
             name.c_str(), bdyName.c_str());
  return b;
}

bool SceneEntity::isCollideable(const RcsGraph* graph) const
{
  const RcsBody* body = RcsGraph_getBodyByName(graph, bdyName.c_str());
  return RcsBody_numDistanceShapes(body) > 0 ? true : false;
}

bool SceneEntity::isOfType(const std::string& type) const
{
  if (std::find(types.begin(), types.end(), type) == types.end())
  {
    return false;
  }

  return true;
}





/*******************************************************************************
 *
 ******************************************************************************/
AffordanceEntity::AffordanceEntity()
{
}

AffordanceEntity& AffordanceEntity::operator= (const AffordanceEntity& copyFromMe)
{
  if (this == &copyFromMe)
  {
    return *this;
  }

  SceneEntity::operator=(copyFromMe);

  for (size_t i=0; i<affordances.size(); ++i)
  {
    delete affordances[i];
  }
  affordances.clear();

  for (size_t i=0; i<copyFromMe.affordances.size(); ++i)
  {
    affordances.push_back(copyFromMe.affordances[i]->clone());
  }

  return *this;
}

AffordanceEntity::AffordanceEntity(const xmlNodePtr node) : SceneEntity(node)
{
  xmlNodePtr child = node->children;

  while (child)
  {
    if (isXMLNodeNameNoCase(child, "PowerGraspable"))
    {
      affordances.push_back(new PowerGraspable(child));
    }
    else if (isXMLNodeNameNoCase(child, "PincerGraspable"))
    {
      affordances.push_back(new PincerGraspable(child));
    }
    else if (isXMLNodeNameNoCase(child, "PalmGraspable"))
    {
      affordances.push_back(new PalmGraspable(child));
    }
    else if (isXMLNodeNameNoCase(child, "BallGraspable"))
    {
      affordances.push_back(new BallGraspable(child));
    }
    else if (isXMLNodeNameNoCase(child, "CircularGraspable"))
    {
      affordances.push_back(new CircularGraspable(child));
    }
    else if (isXMLNodeNameNoCase(child, "TwistGraspable"))
    {
      affordances.push_back(new TwistGraspable(child));
    }
    else if (isXMLNodeNameNoCase(child, "Twistable"))
    {
      affordances.push_back(new Twistable(child));
    }
    else if (isXMLNodeNameNoCase(child, "PushSwitchable"))
    {
      affordances.push_back(new PushSwitchable(child));
    }
    else if (isXMLNodeNameNoCase(child, "Supportable"))
    {
      Supportable* s = new Supportable(child);
      affordances.push_back(s);
    }
    else if (isXMLNodeNameNoCase(child, "Stackable"))
    {
      affordances.push_back(new Stackable(child));
    }
    else if (isXMLNodeNameNoCase(child, "Containable"))
    {
      affordances.push_back(new Containable(child));
    }
    else if (isXMLNodeNameNoCase(child, "Pourable"))
    {
      affordances.push_back(new Pourable(child));
    }
    else if (isXMLNodeNameNoCase(child, "PointPushable"))
    {
      affordances.push_back(new PointPushable(child));
    }
    else if (isXMLNodeNameNoCase(child, "PointPokable"))
    {
      affordances.push_back(new PointPokable(child));
    }
    else if (isXMLNodeNameNoCase(child, "Hingeable"))
    {
      affordances.push_back(new Hingeable(child));
    }
    else if (isXMLNodeNameNoCase(child, "Dispensible"))
    {
      affordances.push_back(new Dispensible(child));
    }
    else if (isXMLNodeNameNoCase(child, "Wettable"))
    {
      affordances.push_back(new Wettable(child));
    }
    else if (isXMLNodeNameNoCase(child, "Openable"))
    {
      Openable* affordance = new Openable(child);
      RLOG_CPP(1, "Assigning '" << bdyName << "' to Openable - fix me");
      affordance->frame = bdyName;   // \todo(MG): fix it. We need a frame to pass the check
      affordances.push_back(affordance);

    }

    child = child->next;
  }

}

AffordanceEntity::AffordanceEntity(const AffordanceEntity& other) : SceneEntity(other)
{
  for (size_t i=0; i<other.affordances.size(); ++i)
  {
    Affordance* ai = other.affordances[i]->clone();
    NLOG(0, "ai is of type %s", ai->classname().c_str());
    affordances.push_back(ai);
  }
}

AffordanceEntity::~AffordanceEntity()
{
  for (size_t i=0; i<affordances.size(); ++i)
  {
    delete affordances[i];
  }

}

void AffordanceEntity::print() const
{
  std::cout << "Affordance model " << name << " has " << affordances.size()
            << " affordances:" << std::endl;

  for (auto a : affordances)
  {
    a->print();
    std::cout << std::endl;
  }
}

bool AffordanceEntity::check(const RcsGraph* graph) const
{
  bool success = true;

  const RcsBody* ntt = RcsGraph_getBodyByName(graph, bdyName.c_str());

  if (!ntt)
  {
    RLOG(1, "No graph body with name '%s' was found", bdyName.c_str());
    success = false;
  }

  for (auto a : affordances)
  {
    success = a->check(graph) && success;

    // We enforce that the affordance frames are children of the entity or the
    // entity itself
    const RcsBody* affordanceFrm = RcsGraph_getBodyByName(graph, a->frame.c_str());

    if (!affordanceFrm)
    {
      success = false;
    }
    else
    {
      if (//(affordanceFrm->parentId!=ntt->id) &&
        (affordanceFrm->id!=ntt->id) &&
        (!RcsBody_isChild(graph, affordanceFrm, ntt)))
      {
        RLOG(1, "Affordance frame '%s' is not a child of or '%s'",
             affordanceFrm->name, ntt->name);
        success=false;
      }
    }
  }

  return success;
}

/*******************************************************************************
 *
 ******************************************************************************/
class CapabilityCompare
{
public:
  CapabilityCompare(const RcsGraph* g, double wLin_ = 1.0, double wAng_ = 0.0) :
    graph(g), wLin(wLin_), wAng(wAng_)
  {
  }

  // The lesser function
  bool operator()(std::tuple<Affordance*, Capability*> a,
                  std::tuple<Affordance*, Capability*> b) const
  {
    const double evalA = eval(graph, a, wLin, wAng);
    const double evalB = eval(graph, b, wLin, wAng);
    return evalA < evalB;
  }

  bool operator()(std::tuple<Affordance*, Capability*, double> a,
                  std::tuple<Affordance*, Capability*, double> b) const
  {
    const double evalA = std::get<2>(a);
    const double evalB = std::get<2>(b);
    return evalA < evalB;
  }

  static double eval(const RcsGraph* graph,
                     std::tuple<Affordance*, Capability*> grasp,
                     double wLin, double wAng)
  {
    RLOG(1, "wAng=%f", wAng);
    RcsBody* bdy0 = RcsGraph_getBodyByName(graph, std::get<0>(grasp)->frame.c_str());
    RcsBody* bdy1 = RcsGraph_getBodyByName(graph, std::get<1>(grasp)->frame.c_str());

    double dist = Vec3d_distance(bdy0->A_BI.org, bdy1->A_BI.org);
    double ang = Mat3d_diffAngle(bdy0->A_BI.rot, bdy1->A_BI.rot);

    return wLin * dist + wAng * ang;
  }

private:
  const RcsGraph* graph;
  double wLin, wAng;
};

/*******************************************************************************
 *
 ******************************************************************************/
class AffordanceCompare
{
public:
  AffordanceCompare(const RcsGraph* g, double wLin_ = 1.0, double wAng_ = 0.0) :
    graph(g), wLin(wLin_), wAng(wAng_)
  {
  }

  // The lesser function
  bool operator()(std::tuple<Affordance*, Affordance*> a,
                  std::tuple<Affordance*, Affordance*> b) const
  {
    const double evalA = eval(graph, a, wLin, wAng);
    const double evalB = eval(graph, b, wLin, wAng);
    return evalA < evalB;
  }

  bool operator()(std::tuple<Affordance*, Affordance*, double> a,
                  std::tuple<Affordance*, Affordance*, double> b) const
  {
    const double evalA = std::get<2>(a);
    const double evalB = std::get<2>(b);
    return evalA < evalB;
  }

  static double eval(const RcsGraph* graph,
                     std::tuple<Affordance*, Affordance*> grasp,
                     double wLin, double wAng)
  {
    RLOG(1, "wAng=%f", wAng);
    RcsBody* bdy0 = RcsGraph_getBodyByName(graph, std::get<0>(grasp)->frame.c_str());
    RcsBody* bdy1 = RcsGraph_getBodyByName(graph, std::get<1>(grasp)->frame.c_str());

    double dist = Vec3d_distance(bdy0->A_BI.org, bdy1->A_BI.org);
    double ang = Mat3d_diffAngle(bdy0->A_BI.rot, bdy1->A_BI.rot);

    return wLin * dist + wAng * ang;
  }

private:
  const RcsGraph* graph;
  double wLin, wAng;
};

/*******************************************************************************
 *
 ******************************************************************************/
void sort(const RcsGraph* graph,
          std::vector<std::tuple<Affordance*, Capability*>>& pairs,
          double wLin, double wAng)
{
  std::vector< std::tuple<Affordance*, Capability*, double>> qPairs;

  for (auto& pair : pairs)
  {
    Affordance* a = std::get<0>(pair);
    Capability* c = std::get<1>(pair);
    double quality = CapabilityCompare::eval(graph, pair, wLin, wAng);
    qPairs.emplace_back(std::make_tuple(a, c, quality));
  }

  std::sort(qPairs.begin(), qPairs.end(), CapabilityCompare(graph));

  pairs.clear();

  for (auto& pair : qPairs)
  {
    Affordance* a = std::get<0>(pair);
    Capability* c = std::get<1>(pair);
    pairs.emplace_back(std::make_tuple(a, c));
  }

  for (size_t i=0; i<qPairs.size(); ++i)
  {
    RLOG(0, "Solution %zu: %s - %s = %f", i,
         std::get<0>(qPairs[i])->frame.c_str(),
         std::get<1>(qPairs[i])->frame.c_str(),
         std::get<2>(qPairs[i]));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void sort(const RcsGraph* graph,
          std::vector<std::tuple<Affordance*, Affordance*>>& pairs,
          double wLin, double wAng)
{
  std::vector< std::tuple<Affordance*, Affordance*, double>> qPairs;

  for (auto& pair : pairs)
  {
    Affordance* a = std::get<0>(pair);
    Affordance* c = std::get<1>(pair);
    double quality = AffordanceCompare::eval(graph, pair, wLin, wAng);
    qPairs.emplace_back(std::make_tuple(a, c, quality));
  }

  std::sort(qPairs.begin(), qPairs.end(), AffordanceCompare(graph));

  pairs.clear();

  for (auto& pair : qPairs)
  {
    Affordance* a = std::get<0>(pair);
    Affordance* c = std::get<1>(pair);
    pairs.emplace_back(std::make_tuple(a, c));
  }

  for (size_t i=0; i<qPairs.size(); ++i)
  {
    RLOG(0, "Solution %zu: %s - %s = %f", i,
         std::get<0>(qPairs[i])->frame.c_str(),
         std::get<1>(qPairs[i])->frame.c_str(),
         std::get<2>(qPairs[i]));
  }
}

} // namespace aff
