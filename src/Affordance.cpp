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

#include "Affordance.h"

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
the components are the affordances.

*/

namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/

Affordance::Type Affordance::typeFromString(const std::string& className)
{
  auto it = typeMap.find(className);
  RCHECK_MSG(it!=typeMap.end(), "No such affordance: %s", className.c_str());
  return it->second;
}

std::string Affordance::stringFromType(Affordance::Type aType)
{
  std::map<std::string, Affordance::Type>::iterator it;

  for (it = typeMap.begin(); it != typeMap.end(); ++it)
  {
    if (aType==it->second)
    {
      return it->first;
    }

  }

  RFATAL("No such affordance type : %d", static_cast<int>(aType));
  return std::string();
}

Affordance::Affordance(const xmlNodePtr node)
{
  className = (const char*)node->name;
  classType = typeFromString((const char*)node->name);
  frame = Rcs::getXMLNodePropertySTLString(node, "frame");
}

Affordance::~Affordance()
{
}

Affordance* Affordance::clone() const
{
  return new Affordance(*this);
}

std::string Affordance::classname() const
{
  return className;
}

void Affordance::print() const
{
  std::cout << "[Affordance: type=" << classname() << " frame="
            << frame << "]";
}

bool Affordance::check(const RcsGraph* graph) const
{
  if (!RcsGraph_getBodyByName(graph, frame.c_str()))
  {
    RLOG(1, "[%s]: No frame with name '%s' was found",
         classname().c_str(), frame.c_str());
    return false;
  }

  return true;
}



std::map<std::string,Affordance::Type> Affordance::typeMap =
{
  {"Affordance", Type::Affordance},
  {"Graspable", Type::Graspable},
  {"PowerGraspable", Type::PowerGraspable},
  {"PincerGraspable", Type::PincerGraspable},
  {"PalmGraspable", Type::PalmGraspable},
  {"BallGraspable", Type::BallGraspable},
  {"CircularGraspable", Type::CircularGraspable},
  {"TwistGraspable", Type::TwistGraspable},
  {"Twistable", Type::Twistable},
  {"PushSwitchable", Type::PushSwitchable},
  {"Supportable", Type::Supportable},
  {"Stackable", Type::Stackable},
  {"Containable", Type::Containable},
  {"Pourable", Type::Pourable},
  {"PointPushable", Type::PointPushable},
  {"PointPokable", Type::PointPokable},
  {"Hingeable", Type::Hingeable},
  {"Dispensible", Type::Dispensible},
  {"Wettable", Type::Wettable},
  {"Openable", Type::Openable}
};




Graspable::Graspable(const xmlNodePtr node) : Affordance(node)
{
}

Affordance* Graspable::clone() const
{
  return new Graspable(*this);
}

Graspable::~Graspable()
{
}



// Describes a frame that allows a power grasp of the object. There
// is no constraint around the power grasp axis.
PowerGraspable::PowerGraspable(const xmlNodePtr node) : Graspable(node), radius(0.0)
{
}

Affordance* PowerGraspable::clone() const
{
  return new PowerGraspable(*this);
}

PowerGraspable::~PowerGraspable()
{
}

// Describes a frame that allows a grasp with thumb and index finger, and
// possibly any third finger. This affordance assumes that the effector
// is aligned with the object in 6 dimensions.
PincerGraspable::PincerGraspable(const xmlNodePtr node) : Graspable(node), thickness(0.0)
{
}

Affordance* PincerGraspable::clone() const
{
  return new PincerGraspable(*this);
}

PincerGraspable::~PincerGraspable()
{
}

// Describes a frame that allows a grasp like a PowerGrasp, but with the object touching
// the palm, and with a 3d orientation. This affordance assumes that the effector
// is aligned with the object in 6 dimensions.
PalmGraspable::PalmGraspable(const xmlNodePtr node) : Graspable(node), thickness(0.0)
{
}

Affordance* PalmGraspable::clone() const
{
  return new PalmGraspable(*this);
}

PalmGraspable::~PalmGraspable()
{
}

// Affordance that allows to grasp an object like a ball. An orientation
// is actually not really needed. We nevertheless add an inclination
// constraint to make the object being grasped preferably from the top.
BallGraspable::BallGraspable(const xmlNodePtr node) : Graspable(node), radius(0.0)
{
  getXMLNodePropertyDouble(node, "radius", &radius);
}

Affordance* BallGraspable::clone() const
{
  return new BallGraspable(*this);
}

BallGraspable::~BallGraspable()
{
}

bool BallGraspable::check(const RcsGraph* graph) const
{
  if (radius==0.0)
  {
    RLOG(1, "[%s %s]: Radius is %f, please make sure it has a posiive value",
         classname().c_str(), frame.c_str(), radius);
    return !false;
  }

  return Affordance::check(graph);
}

// Grasping around a circular curve, like a rim of a cup
CircularGraspable::CircularGraspable(const xmlNodePtr node) : Graspable(node), radius(0.0)
{
  getXMLNodePropertyDouble(node, "radius", &radius);
}

Affordance* CircularGraspable::clone() const
{
  return new CircularGraspable(*this);
}

CircularGraspable::~CircularGraspable()
{
}

// Affordance that allows to grasp an object like a cylinder from the top.
// The cylinder axis is not constrained.
TwistGraspable::TwistGraspable(const xmlNodePtr node) : Graspable(node), radius(0.0)
{
  getXMLNodePropertyDouble(node, "radius", &radius);
}

Affordance* TwistGraspable::clone() const
{
  return new TwistGraspable(*this);
}

TwistGraspable::~TwistGraspable()
{
}

// Same as TwistGraspable, but lets rotate around the cylinder axis
Twistable::Twistable(const xmlNodePtr node) : TwistGraspable(node)
{
}

Affordance* Twistable::clone() const
{
  return new Twistable(*this);
}

Twistable::~Twistable()
{
}

// A switch that can be pushed into the negative axis direction.
PushSwitchable::PushSwitchable(const xmlNodePtr node) :
  Affordance(node), axisDir(2), pushDepth(0.0)
{
}

Affordance* PushSwitchable::clone() const
{
  return new PushSwitchable(*this);
}

PushSwitchable::~PushSwitchable()
{
}


// A surface on which the object can be put down on. Supportables can be
// put on Stackables.
Supportable::Supportable(const xmlNodePtr node) :
  Affordance(node), extentsX(0.0), extentsY(0.0)
{
  // We just have a simple rectangle representation of the support area.
  getXMLNodePropertyDouble(node, "extentsX", &extentsX);
  getXMLNodePropertyDouble(node, "extentsY", &extentsY);
}

Affordance* Supportable::clone() const
{
  return new Supportable(*this);
}

Supportable::~Supportable()
{
}

// A surface on which other objects can be stacked
Stackable::Stackable(const xmlNodePtr node) :
  Affordance(node), extentsX(0.0), extentsY(0.0), normalDir(2)
{
  // We just have a simple rectangle representation of the support area.
  getXMLNodePropertyDouble(node, "extentsX", &extentsX);
  getXMLNodePropertyDouble(node, "extentsY", &extentsY);

  std::string text = "Z";
  Rcs::getXMLNodePropertySTLString(node, "normalDir", text);

  if (text=="X")
  {
    this->normalDir = 0;
  }
  else if (text=="Y")
  {
    this->normalDir = 1;
  }
  else if (text=="Z")
  {
    this->normalDir = 2;
  }

  // A Stackable needs a Supportable to be put on
  requiredAffordances.push_back(Type::Supportable);
}

Affordance* Stackable::clone() const
{
  return new Stackable(*this);
}

Stackable::~Stackable()
{
}

// A containment where stuff can be put into.
Containable::Containable(const xmlNodePtr node) : Affordance(node), maxVolume(1.0)
{
  getXMLNodePropertyDouble(node, "max_volume", &maxVolume);

  xmlNodePtr child = node->children;

  while (child)
  {
    if (isXMLNodeNameNoCase(child, "Ingredient"))
    {
      std::string ingredient = Rcs::getXMLNodePropertySTLString(child, "name");
      double volume = 0.0;
      getXMLNodePropertyDouble(child, "volume", &volume);
      liquidIngredients.push_back(std::make_pair(ingredient,volume));
      RLOG(1, "Affordance \"%s\" contains %f liters of %s", frame.c_str(), volume, ingredient.c_str());
    }

    child = child->next;
  }

}

Affordance* Containable::clone() const
{
  return new Containable(*this);
}

Containable::~Containable()
{
}

void Containable::print() const
{
  Affordance::print();

  for (size_t i=0; i<liquidIngredients.size(); ++i)
  {
    std::cout << "   " << liquidIngredients[i].first << ": " << liquidIngredients[i].second
              << " liters" << std::endl;
  }

}

double Containable::getVolume() const
{
  double vol = 0.0;
  for (const auto& ingredient : liquidIngredients)
  {
    vol += ingredient.second;
  }

  return vol;
}

// An opening where stuff can be poured out of.
Pourable::Pourable(const xmlNodePtr node) : Affordance(node)
{
  requiredAffordances.push_back(Type::Containable);
}

Affordance* Pourable::clone() const
{
  return new Pourable(*this);
}

Pourable::~Pourable()
{
}

// Something that can be pushed like a button.
PointPushable::PointPushable(const xmlNodePtr node) : Affordance(node)
{
}

Affordance* PointPushable::clone() const
{
  return new PointPushable(*this);
}

PointPushable::~PointPushable()
{
}

// Something that can push on a button.
PointPokable::PointPokable(const xmlNodePtr node) : Affordance(node)
{
  requiredAffordances.push_back(Type::PointPushable);
}

Affordance* PointPokable::clone() const
{
  return new PointPokable(*this);
}

PointPokable::~PointPokable()
{
}

// Something that can be rotated about a hinge joint (e.g. a door)
Hingeable::Hingeable(const xmlNodePtr node) : Affordance(node)
{
}

Affordance* Hingeable::clone() const
{
  return new Hingeable(*this);
}

Hingeable::~Hingeable()
{
}

// Something solid that lets itself dispense from a container (like an olive)
Dispensible::Dispensible(const xmlNodePtr node) : Affordance(node)
{
  requiredAffordances.push_back(Type::Containable);
  requiredAffordances.push_back(Type::Supportable);
}

Affordance* Dispensible::clone() const
{
  return new Dispensible(*this);
}

Dispensible::~Dispensible()
{
}

// Something liquid that lets itself dispense from a container (like tea)
Wettable::Wettable(const xmlNodePtr node) : Affordance(node)
{
  requiredAffordances.push_back(Type::Containable);
}

Affordance* Wettable::clone() const
{
  return new Wettable(*this);
}

Wettable::~Wettable()
{
}

// Something that can be opened and closed (fridge, drawer ...). It doesn't
// necessarily need to contain something.
Openable::Openable(const xmlNodePtr node) : Affordance(node), isOpen(true)
{
  getXMLNodePropertyBoolString(node, "open", &isOpen);
}

Affordance* Openable::clone() const
{
  return new Openable(*this);
}

Openable::~Openable()
{
}

} // namespace aff
