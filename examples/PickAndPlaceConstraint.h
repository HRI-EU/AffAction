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

#ifndef TROPIC_PICKANDPLACECONSTRAINT_H
#define TROPIC_PICKANDPLACECONSTRAINT_H

#include "GraphConstraint.h"


/*!
 *  At time t, the following happens:
 *  - If a rigid body is a child of the given gripper, it will be attached to
 *    the closest rigid body in the graph.
 *  - If the given gripper has no child, the closest rigid body with non-zero
 *    weight will be attached to the gripper.
 *
 *  It is currently not distinguished if in the second case, the rigid body
 *  is part of the gripper's kinematic chain.
 */

namespace tropic
{

class PickAndPlaceConstraint : public GraphConstraint
{
public:

  PickAndPlaceConstraint(double t, const std::string& parent);

  PickAndPlaceConstraint(xmlNode* node);

  PickAndPlaceConstraint(const PickAndPlaceConstraint& other);

  virtual PickAndPlaceConstraint* clone() const;

  virtual ~PickAndPlaceConstraint();

  virtual double compute(double dt);

  virtual bool inUse() const;

  virtual double getEndTime() const;

  virtual void fromXML(xmlNode* node);

  virtual void toXML(std::ostream& out, size_t indent = 0) const;

  virtual void findParentChild(int& parentId, int& childId) const;

  void setConnectTransform(const HTr* A_BI);

protected:

  virtual double getStartTimeRecurse() const;

  std::string gripperName;
  double attachTime;
  bool active;
  HTr attachToTrf;
};


}   // namespace tropic





#endif   // TROPIC_PICKANDPLACECONSTRAINT_H
