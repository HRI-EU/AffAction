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
            RLOG(5, "1 Body \"%s\" does %s",
                 BODY->name, switchesOn ? "collide" : "not collide");
            for (unsigned int j=0; j<BODY->nShapes; ++j)
            {
              RLOG(5, "Going through shape %s", RcsShape_name(BODY->shapes[j].type));

              //snprintf(BODY->shapes[j].color, RCS_MAX_NAMELEN, switchesOn ? "RED" : "GREEN");

              RcsShape_setComputeType(&BODY->shapes[j], RCSSHAPE_COMPUTE_WIREFRAME, !switchesOn);

              // We only switch on those shapes that have been distance shapes
              // from the time point of initialization.

              if (BODY->shapes[j].type==RCSSHAPE_MESH || BODY->shapes[j].type==RCSSHAPE_REFFRAME)
                //if (/* switchesOn && */ (!distCalcIndices[i][j]))
              {
                RLOG(5, "IGNORING shape %s", RcsShape_name(BODY->shapes[j].type));
                continue;
              }

              RLOG(5, "2 Body \"%s\" does %s",
                   BODY->name, switchesOn ? "collide" : "not collide");
              RcsShape_setComputeType(&BODY->shapes[j], RCSSHAPE_COMPUTE_DISTANCE, switchesOn);
              //snprintf(BODY->shapes[j].color, RCS_MAX_NAMELEN, switchesOn ? "RED" : "GREEN");
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

  void setGraph(RcsGraph* newGraph)
  {
    GraphConstraint::setGraph(newGraph);

    // In addition, we memorize the original distance calculation of each
    // shape so that we can reset them to their original state.
    distCalcIndices.resize(bdyNames.size());

    RCSGRAPH_FOREACH_BODY(this->graph)
    {
      for (size_t i=0; i<bdyNames.size(); ++i)
      {
        if (STREQ(BODY->name, bdyNames[i].c_str()))
        {
          distCalcIndices[i].resize(BODY->nShapes);

          for (unsigned int j=0; j<BODY->nShapes; ++j)
          {
            distCalcIndices[i][j] = RcsShape_isOfComputeType(&BODY->shapes[j], RCSSHAPE_COMPUTE_DISTANCE) ? true : false;
          }

        }   // if (STREQ(BODY->name, bdyNames[i].c_str()))

      }   // for (size_t i=0; i<bdyNames.size(); ++i)

    }   // RCSGRAPH_FOREACH_BODY(this->graph)

  }



protected:

  std::vector<std::string> bdyNames;
  std::vector<std::vector<bool>> distCalcIndices;
  double toggleTime;
  bool switchesOn;
  bool active;
};


}   // namespace tropic





#endif   // TROPIC_COLLISIONMODELCONSTRAINT_H
