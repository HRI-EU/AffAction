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

#ifndef AFF_MANIPULATOR_H
#define AFF_MANIPULATOR_H

#include "AffordanceEntity.h"
#include "Capability.h"



/*

The Manipulator class is composed of Capabilities, which model how
a manipulator can physically interact with an affordance model.

*/

namespace aff
{

class ActionScene;

class Manipulator
{
public:

  std::string name;
  std::string id;
  std::string agent;
  std::string type;
  std::vector<std::string> fingerJoints;
  std::vector<Capability*> capabilities;

  Manipulator();
  Manipulator(const xmlNodePtr node);
  Manipulator(const Manipulator& other);
  Manipulator& operator = (const Manipulator&);
  virtual ~Manipulator();
  void print() const;
  virtual bool check(const RcsGraph* graph) const;
  bool isEmpty(const RcsGraph* graph) const;
  size_t getNumFingers() const;

  /*! \brief For now this is specific for the Jaco Gen2 3-finger hand. By
   *         measuring, we obtained: y = 175*(1-x) where y is the finger
   *         tip distance in [mm] and x is the joint angle in [rad]. The
   *         function thus returns: x = 1-y/175, or x = 1-y/0.175 since the
   *         fingerTipDistance is given in meters
   */
  std::vector<double> fingerAnglesFromFingerTipDistance(double fingerTipDistanceInMeters) const;

  /*! \brief For a given AffordanceEntity, the function returns the
   *         manipulator's capability and the entitie's affordance
   *         that constitute to the grasp. If the object is not
   *         grasped, a pair of NULL pointers it returned. This function
   *         internally estimates the capability by looping through all
   *         capabilities and checking if any of these is the entitie's
   *         parent. The affordance frame is then estimated by looping
   *         through all affordance frame and returning the first one that
   *         is aligned with the capabilitie's frame. This part might
   *         return an incorrect frame, since several frames can theoretically
   *         be aligned. For most geometric checks, this doesn't matter.
   */
  std::tuple<Capability*,Affordance*,double> getGrasp(const RcsGraph* graph,
                                                      const AffordanceEntity* entity) const;

  /*! \brief Returns the Capability which is the parent of the entity. This
   *         means for instance which hand is holding the object.
   */
  const Capability* getGraspingCapability(const RcsGraph* graph,
                                          const AffordanceEntity* entity) const;

  /*! \brief Convenience function for the above getGraspingCapability() method.
   *         It returns the name of the Capabilitie's frame. If no Capability
   *         is found, the function returns an empty string.
   */
  std::string getGraspingFrame(const RcsGraph* graph,
                               const AffordanceEntity* entity) const;

  // If the manipulator has grasped the entity, the function returns the grasp
  // affordances that are closer to the grasp frame than dist. Otherwise, it
  // will return an empty vector.
  std::vector<const Affordance*> getGraspAffordances(const RcsGraph* graph,
                                                     const AffordanceEntity* entity,
                                                     double dist) const;

  // Returns a vector of AffordanceEntities that are children of the
  // manipulator.
  std::vector<const AffordanceEntity*> getGraspedEntities(const ActionScene& scene,
                                                          const RcsGraph* graph) const;

  std::vector<std::string> getChildrenOfManipulator(const ActionScene* scene,
                                                    const RcsGraph* graph,
                                                    bool useInstanceName) const;

  std::string getGazingFrame() const;
};

} // namespace aff

#endif // AFF_MANIPULATOR_H
