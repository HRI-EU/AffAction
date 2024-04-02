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

#ifndef AFF_ACTIONSCENE_H
#define AFF_ACTIONSCENE_H

#include "Agent.h"

/*

The ActionScene class comprises all manipulators and affordance models.
These classes are independent of any action.

*/

namespace aff
{

class ActionScene
{
public:
  std::vector<AffordanceEntity> entities;
  std::vector<Manipulator> manipulators;
  std::string foveatedEntity;
  std::vector<Agent*> agents;   // Pointer needed for polymorphism

  ActionScene();
  ActionScene(const std::string& xmlFile);
  ActionScene(const ActionScene& other);
  virtual ~ActionScene();
  ActionScene& operator = (const ActionScene&);
  void initializeKinematics(const RcsGraph* graph);
  void print() const;
  bool check(const RcsGraph* graph) const;
  bool reload(const std::string& xmlFile);
  std::string printAffordancesToString() const;
  std::vector<const Manipulator*> getFreeManipulators(const RcsGraph* graph) const;
  std::vector<const Manipulator*> getOccupiedManipulators(const RcsGraph* graph) const;
  const AffordanceEntity* getAffordanceEntity(const std::string& name) const;
  std::vector<const AffordanceEntity*> getAffordanceEntities(const std::string& name) const;

  std::vector<const SceneEntity*> getSceneEntities(const std::string& name) const;

  // Returns a vector of the child entities that have a parent link to the
  // passed entitie's affordance frames.
  std::vector<const AffordanceEntity*> getAllChildren(const RcsGraph* graph,
                                                      const AffordanceEntity* entity) const;

  // Returns a vector of the child entities that have a parent link to the
  // passed entitie's affordance frames.
  std::vector<const AffordanceEntity*> getDirectChildren(const RcsGraph* graph,
                                                         const AffordanceEntity* entity) const;

  const AffordanceEntity* getParentAffordanceEntity(const RcsGraph* graph,
                                                    const AffordanceEntity* child) const;
  const Manipulator* getManipulator(const std::string& manipulatorName) const;
  const Manipulator* getManipulator(const Capability* capability) const;
  std::vector<const Manipulator*> getManipulatorsOfType(const std::string& type) const;
  const Manipulator* getGraspingHand(const RcsGraph* graph,
                                     const AffordanceEntity* entity) const;

  // Returns the first combination of manipulator and entity that has been
  // found to be grasped. \todo(MG): This might have issues if several hands
  // grasp entities with the same name. In this case, the first one found will
  // be returned.
  std::tuple<const Manipulator*,const AffordanceEntity*> getGraspingHand(const RcsGraph* graph,
      std::vector<const AffordanceEntity*> entities) const;

  Agent* getAgent(const std::string& agentName);
  const Agent* getAgent(const std::string& agentName) const;
};

void sort(const RcsGraph* graph,
          std::vector<std::tuple<Affordance*, Capability*>>& pairs,
          double wLin, double wAng);

void sort(const RcsGraph* graph,
          std::vector<std::tuple<Affordance*, Affordance*>>& pairs,
          double wLin, double wAng);

// For a given affordance object and a given manipulator, return all
// capabilities that the hand can apply to the given affordance type
// that is represented in the template parameter. For example:
//
// auto res = match<Graspable>(bottle, rightHand);
//
// might return:
//   <PowerGraspable* pg, PowergraspCapability* pc, 0.3)
//   <Twistable* tg, TwistgraspCapability* tc, 0.1)
//
template<typename T>
std::vector<std::tuple<Affordance*, Capability*> >
match(const AffordanceEntity* object, const Manipulator* hand)
{
  std::vector<std::tuple<Affordance*, Capability*>> res;

  // Outer loop goes through all the object's affordances
  for (Affordance* objAffordance : object->affordances)
  {
    // If the affordance type does not match the template argument, we continue.
    // The dynamic_cast is true if objAffordance is of type T or derieved from T.
    if (!dynamic_cast<T*>(objAffordance))
    {
      continue;
    }

    // From here on, the objAffordance is of type T or its children. For
    // instance if T is Graspable, then objAffordance can be
    // Graspable or PowerGraspable.

    // Middle loop goes through all the manipulator's capabilities
    for (Capability* capability : hand->capabilities)
    {
      // Inner loop: Here we enforce a direct match between the queried
      // affordance of type T, and the capabilitie's affordance.
      for (const auto& capAffordance : capability->affordanceTypes)
      {
        // \todo(MG): if (dynamic_cast<T*>(capAffordance)) ???
        if (capAffordance==objAffordance->classType)
        {
          //std::tuple<Affordance*, Capability*> tpl(objAffordance, capability);
          //res.insert(res.end(), tpl);
          res.emplace_back(std::make_tuple(objAffordance, capability));
        }
      }

    }   // for (const Capability* ...

  }   // for (const Affordance* ...

  return res;
}

// For two given affordance objects, return all affordances that are compatible
// and are instances or descendents the template parameter. The first argument
// is the one that is required from the second one. For example:
//
// auto res = match<Supportable,Stackable>(table,bottle);
//
// means "Return all support surfaces of table that can be used with the
// bottle", and might return:
//
//   <Supportable* tableSurf1, Stackable* bottleBottom, 0.1)
//   <Supportable* tableSurf2, Stackable* bottleBottom, 0.3)
//
//
// auto res = match<Containable,Pourable>(glas,bottle);
//
// means "Return all containers of glas that can be poured into with
// the bottle", and might return:
//
//   <Containable* glasOpening, Pourable* bottleOpening, 0.1)
//
template<typename T1, typename T2>
std::vector<std::tuple<Affordance*, Affordance*> >
match(const AffordanceEntity* object1, const AffordanceEntity* object2)
{
  std::vector<std::tuple<Affordance*, Affordance*>> res;

  // Outer loop goes through all the object2's affordances
  for (Affordance* aff2 : object2->affordances)
  {
    // If the affordance aff2 is not instance or descendent of T2, we continue.
    if (!dynamic_cast<T2*>(aff2))
    {
      continue;
    }

    // From here on, the aff2 is of type T2 or its children. For instance if
    // T2 is Graspable, then aff2 can be Graspable or PowerGraspable.

    // Middle loop goes through all the object1's affordances
    for (Affordance* aff1 : object1->affordances)
    {
      // If the affordance aff1 is not instance or descendent of T1,
      // we continue.
      if (!dynamic_cast<T1*>(aff1))
      {
        continue;
      }

      // From here on, the aff1 is of type T1 or its children.
      // Inner loop: Here we enforce a direct match between the queried
      // affordances of type T, and the capabilitie's affordance.
      for (const auto& reqAffType : aff2->requiredAffordances)
      {
        if (reqAffType == aff1->classType)
        {
          // std::tuple<Affordance*, Affordance*> tpl(aff1, aff2);
          // res.insert(res.end(), tpl);
          res.emplace_back(std::make_tuple(aff1, aff2));
        }
      }

    }   // for (const Affordance* aff1...


  }   // for (const Affordance* aff2...

  return res;
}

template<typename T>
std::vector<T*> getAffordances(const AffordanceEntity* object)
{
  std::vector<T*> res;

  if (!object)
  {
    return res;
  }

  for (Affordance* affordance : object->affordances)
  {
    // If the affordance aff is instance or descendent of T,
    // we add it to the result.
    T* dst = dynamic_cast<T*>(affordance);
    if (dst)
    {
      res.push_back(dst);
    }
  }

  return res;
}

template<typename T>
std::vector<T*> getCapabilities(const Manipulator* effector)
{
  std::vector<T*> res;

  if (!effector)
  {
    return res;
  }

  for (Capability* capability : effector->capabilities)
  {
    // If the affordance aff is instance or descendent of T,
    // we add it to the result.
    T* dst = dynamic_cast<T*>(capability);
    if (dst)
    {
      res.push_back(dst);
    }
  }

  return res;
}

} // namespace aff

#endif // AFF_ACTIONSCENE_H
