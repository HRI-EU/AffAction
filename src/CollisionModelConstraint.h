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
#if 0
#ifndef TROPIC_COLLISIONMODELCONSTRAINT_H
#define TROPIC_COLLISIONMODELCONSTRAINT_H

#include <GraphConstraint.h>

#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>

#include <algorithm>


namespace tropic
{

class CollisionModelConstraint : public tropic::GraphConstraint
{
public:

  CollisionModelConstraint(double t, const std::string& bdyName,
                           bool switchOn) :
    GraphConstraint(),
    toggleTime(t),
    switchesOn(switchOn),
    active(true)
  {
    setClassName("CollisionModelConstraint");
    bdyNames.push_back(bdyName);
  }

  CollisionModelConstraint(double t, std::vector<std::string> names,
                           bool switchOn) :
    GraphConstraint(),
    bdyNames(names),
    toggleTime(t),
    switchesOn(switchOn),
    active(true)
  {
    setClassName("CollisionModelConstraint");
  }

  CollisionModelConstraint(const CollisionModelConstraint& other) :
    GraphConstraint(other),
    bdyNames(other.bdyNames),
    toggleTime(other.toggleTime),
    switchesOn(other.switchesOn),
    active(other.active)
  {
  }

  virtual CollisionModelConstraint* clone() const
  {
    CollisionModelConstraint* tSet = new CollisionModelConstraint(toggleTime, bdyNames, switchesOn);
    tSet->constraint = constraint;

    for (size_t i = 0; i < children.size(); ++i)
    {
      tSet->add(std::shared_ptr<ConstraintSet>(children[i]->clone()));
    }

    return tSet;
  }

  virtual ~CollisionModelConstraint()
  {
  }

  virtual double compute(double dt)
  {
    toggleTime -= dt;

    if ((toggleTime<0.0) && (toggleTime>=-dt))
    {

      RCSGRAPH_FOREACH_BODY(this->graph)
      {
        for (size_t i=0; i<bdyNames.size(); ++i)
        {
          if (STREQ(BODY->name, bdyNames[i].c_str()))
          {
            bool wfToggle = false;  // Will be set true if the wireframe flag is toggled

            for (unsigned int j=0; j<BODY->nShapes; ++j)
            {
              RcsShape* sh = &BODY->shapes[j];

              // We ignore meshes and frames. We only switch on those shapes that
              // have been distance shapes from the time point of initialization.
              // They have been assigned the contact flag (as a HACK).
              if ((sh->type == RCSSHAPE_MESH) || (sh->type == RCSSHAPE_REFFRAME) ||
                  (!RcsShape_isOfComputeType(sh, RCSSHAPE_COMPUTE_CONTACT)))
              {
                continue;
              }

              RcsShape_setComputeType(sh, RCSSHAPE_COMPUTE_DISTANCE, switchesOn);
              wfToggle = true;
            }

            // If the wireframe flag has been toggled, the wireframe mode will be
            // changed for all shapes with a contact flag. Meshes are excluded.
            if (wfToggle)
            {
              for (unsigned int j=0; j<BODY->nShapes; ++j)
              {
                RcsShape_setComputeType(&BODY->shapes[j], RCSSHAPE_COMPUTE_WIREFRAME,
                                        !switchesOn);
              }
            }

          }
        }
      }

      this->active = false;
    }

    return GraphConstraint::compute(dt);
  }

  bool inUse() const
  {
    return this->active;
  }

  double getStartTimeRecurse() const
  {
    double startTime = ConstraintSet::getStartTimeRecurse();
    startTime = std::min(toggleTime, startTime);
    return startTime < 0.0 ? 0.0 : startTime;
  }

  double getEndTime() const
  {
    return std::max(toggleTime, ConstraintSet::getEndTime());
  }

protected:

  std::vector<std::string> bdyNames;
  double toggleTime;
  bool switchesOn;
  bool active;
};


}   // namespace tropic





#endif   // TROPIC_COLLISIONMODELCONSTRAINT_H
#endif//0
