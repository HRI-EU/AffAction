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

#include "Capability.h"

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

namespace aff
{

Capability::Capability(const xmlNodePtr node)
{
  className = (const char*)node->name;
  classType = typeFromString((const char*)node->name);
  frame = Rcs::getXMLNodePropertySTLString(node, "frame");
}

Capability::~Capability()
{
}

Capability* Capability::clone() const
{
  return new Capability(*this);
}

void Capability::print() const
{
  std::cout << "['" << stringFromType(classType) << "': frame = '" << frame << "' ";
  if (affordanceTypes.empty())
  {
    std::cout << "no affordances";
  }
  else
  {
    std::cout << " affordances: '";
    for (const auto& n : affordanceTypes)
    {
      std::cout << Affordance::stringFromType(n) << " ";
    }
    std::cout << "'";
  }

  std::cout << "]";
}

bool Capability::check(const RcsGraph* graph) const
{
  if (!RcsGraph_getBodyByName(graph, frame.c_str()))
  {
    RLOG(1, "No frame with name '%s' was found", frame.c_str());
    return false;
  }

  return true;
}

std::map<std::string, Capability::Type> Capability::typeMap =
{
  {"Capability", Type::Capability},
  {"GraspCapability", Type::GraspCapability},
  {"PowergraspCapability", Type::PowergraspCapability},
  {"PincergraspCapability", Type::PincergraspCapability},
  {"TwistgraspCapability", Type::TwistgraspCapability},
  {"CirculargraspCapability", Type::CirculargraspCapability},
  {"PalmgraspCapability", Type::PalmgraspCapability},
  {"FingerpushCapability", Type::FingerpushCapability},
  {"GazeCapability", Type::GazeCapability}
};

Capability::Type Capability::typeFromString(const std::string& className)
{
  auto it = typeMap.find(className);
  RCHECK_MSG(it != typeMap.end(), "No such capability: %s", className.c_str());
  return it->second;
}

std::string Capability::stringFromType(Capability::Type aType)
{
  std::map<std::string, Capability::Type>::iterator it;

  for (it = typeMap.begin(); it != typeMap.end(); ++it)
  {
    if (aType == it->second)
    {
      return it->first;
    }

  }

  RFATAL("No such capability type : %d", static_cast<int>(aType));
  return std::string();
}

GraspCapability::GraspCapability(const xmlNodePtr node) : Capability(node)
{
}

GraspCapability::~GraspCapability()
{
}

Capability* GraspCapability::clone() const
{
  return new GraspCapability(*this);
}

PowergraspCapability::PowergraspCapability(const xmlNodePtr node) : GraspCapability(node)
{
  affordanceTypes.push_back(Affordance::Type::PowerGraspable);
  affordanceTypes.push_back(Affordance::Type::Hingeable);
}

PowergraspCapability::~PowergraspCapability()
{
}

Capability* PowergraspCapability::clone() const
{
  return new PowergraspCapability(*this);
}

PincergraspCapability::PincergraspCapability(const xmlNodePtr node) : GraspCapability(node)
{
  affordanceTypes.push_back(Affordance::Type::PincerGraspable);
  affordanceTypes.push_back(Affordance::Type::BallGraspable);
}

PincergraspCapability::~PincergraspCapability()
{
}

Capability* PincergraspCapability::clone() const
{
  return new PincergraspCapability(*this);
}

TwistgraspCapability::TwistgraspCapability(const xmlNodePtr node) : GraspCapability(node)
{
  affordanceTypes.push_back(Affordance::Type::TwistGraspable);
}

TwistgraspCapability::~TwistgraspCapability()
{
}

Capability* TwistgraspCapability::clone() const
{
  return new TwistgraspCapability(*this);
}

CirculargraspCapability::CirculargraspCapability(const xmlNodePtr node) : GraspCapability(node)
{
  std::vector<std::string> frames = Rcs::String_split(frame, " ");
  RCHECK_MSG(frames.size()==2, "CirculargraspCapability requires two frames (space-separated)");
  frame = frames[0];
  frame2 = frames[1];
  affordanceTypes.push_back(Affordance::Type::CircularGraspable);
}

CirculargraspCapability::~CirculargraspCapability()
{
}

Capability* CirculargraspCapability::clone() const
{
  return new CirculargraspCapability(*this);
}

bool CirculargraspCapability::check(const RcsGraph* graph) const
{
  if (!RcsGraph_getBodyByName(graph, frame2.c_str()))
  {
    RLOG(1, "No frame with name '%s' was found", frame2.c_str());
    return false;
  }

  return Capability::check(graph);
}

PalmgraspCapability::PalmgraspCapability(const xmlNodePtr node) : GraspCapability(node)
{
  affordanceTypes.push_back(Affordance::Type::PalmGraspable);
  affordanceTypes.push_back(Affordance::Type::Hingeable);
}

PalmgraspCapability::~PalmgraspCapability()
{
}

Capability* PalmgraspCapability::clone() const
{
  return new PalmgraspCapability(*this);
}

FingerpushCapability::FingerpushCapability(const xmlNodePtr node) : Capability(node)
{
  affordanceTypes.push_back(Affordance::Type::PushSwitchable);
}

FingerpushCapability::~FingerpushCapability()
{
}

Capability* FingerpushCapability::clone() const
{
  return new FingerpushCapability(*this);
}

GazeCapability::GazeCapability(const xmlNodePtr node) : Capability(node)
{
}

GazeCapability::~GazeCapability()
{
}

Capability* GazeCapability::clone() const
{
  return new GazeCapability(*this);
}

} // namespace aff