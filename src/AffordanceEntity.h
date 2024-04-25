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

#ifndef AFF_AFFORDANCEENTITY_H
#define AFF_AFFORDANCEENTITY_H

#include "Affordance.h"

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
std::string join_strings(const std::vector<std::string>& strings);

class SceneEntity
{
public:
  std::string name;
  std::string bdyName;
  std::string instanceId;
  std::vector<std::string> types;

  SceneEntity();
  SceneEntity(const xmlNodePtr node, const std::string& groupSuffix);
  virtual ~SceneEntity();

  /*! \brief Returns true if one of the SceneEntitie's types equals the
   *         type passed in the argument, false otherwise.
   */
  bool isOfType(const std::string& type) const;

  /*! \brief Checks if the RcsBody matching bdyName has a collideable shape.
   */
  bool isCollideable(const RcsGraph* graph) const;

  /*! \brief Returns the RcsBody matching bdyName.
   */
  const RcsBody* body(const RcsGraph* graph) const;
  RcsBody* body(RcsGraph* graph);

  /*! \brief Returns the RcsBody's transformation (matching bdyName).
   */
  HTr getBodyTransform(const RcsGraph* graph) const;
};

class AffordanceEntity : public SceneEntity
{
public:

  std::vector<Affordance*> affordances;

  AffordanceEntity();
  AffordanceEntity& operator = (const AffordanceEntity&);
  AffordanceEntity(const xmlNodePtr node, const std::string& groupSuffix);
  AffordanceEntity(const AffordanceEntity& other);
  virtual ~AffordanceEntity();
  void print() const;
  virtual bool check(const RcsGraph* graph) const;
};

}   // namespace aff

#endif // AFF_AFFORDANCEENTITY_H
