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

#ifndef AFF_AFFORDANCE_H
#define AFF_AFFORDANCE_H

#include <Rcs_graph.h>
#include <Rcs_parser.h>

#include <vector>
#include <string>
#include <map>

/*

Entity component model for affordance. An affordance model (object) is
composed of a set of affordances, each describing the physical effect
other objects or effectors can do with it. The object is the entity,
the components are the affordances.

*/

namespace aff
{

class Affordance
{
public:

  enum class Type { Affordance,
                    Graspable,
                    PowerGraspable,
                    PincerGraspable,
                    PalmGraspable,
                    BallGraspable,
                    CircularGraspable,
                    TwistGraspable,
                    Twistable,
                    PushSwitchable,
                    Supportable,
                    Stackable,
                    Containable,
                    Pourable,
                    PointPushable,
                    PointPokable,
                    Hingeable,
                    Dispensible,
                    Wettable,
                    Openable
                  };

  static std::map<std::string,Type> typeMap;

  std::string frame;
  std::string className;
  Type classType;
  std::vector<Type> requiredAffordances;

  static Type typeFromString(const std::string& className);
  static std::string stringFromType(Type aType);

  Affordance(const xmlNodePtr node);
  virtual ~Affordance();
  virtual Affordance* clone() const;
  virtual std::string classname() const;
  virtual void print() const;
  virtual bool check(const RcsGraph* graph) const;
};

class Graspable : public Affordance
{
public:

  Graspable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Graspable();
};

// Describes a frame that allows a power grasp of the object. There
// is no constraint around the power grasp axis.
class PowerGraspable : public Graspable
{
public:

  PowerGraspable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~PowerGraspable();

  double radius;
};

// Describes a frame that allows a grasp with thumb and index finger, and
// possibly any third finger. This affordance assumes that the effector
// is aligned with the object in 6 dimensions.
class PincerGraspable : public Graspable
{
public:

  PincerGraspable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~PincerGraspable();

  double thickness;
};

// Describes a frame that allows a grasp like a PowerGrasp, but with the object
// touching the palm, and with a 3d orientation. This affordance assumes that
// the effector is aligned with the object in 6 dimensions.
class PalmGraspable : public Graspable
{
public:

  PalmGraspable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~PalmGraspable();

  double thickness;
};

// Affordance that allows to grasp an object like a ball. An orientation
// is actually not really needed. We nevertheless add an inclination
// constraint to make the object being grasped preferably from the top.
class BallGraspable : public Graspable
{
public:

  BallGraspable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~BallGraspable();
  virtual bool check(const RcsGraph* graph) const;

  double radius;
};

// Grasping around a circular curve, like a rim of a cup
class CircularGraspable : public Graspable
{
public:

  CircularGraspable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~CircularGraspable();

  double radius;
};

// Affordance that allows to grasp an object like a cylinder from the top.
// The cylinder axis is not constrained.
class TwistGraspable : public Graspable
{
public:

  TwistGraspable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~TwistGraspable();

  double radius;
};

// Describes a frame that allows fingers radially to grasp
class Twistable : public TwistGraspable
{
public:

  Twistable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Twistable();
};

// A switch that can be pushed into the negative axis direction.
class PushSwitchable : public Affordance
{
public:

  PushSwitchable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~PushSwitchable();

protected:
  int axisDir;
  double pushDepth;
};

// A surface on which the object can be put down on (Support urface).
// Stackables can be put on Supportables.
class Supportable : public Affordance
{
public:

  Supportable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Supportable();

  //protected:
  double extentsX, extentsY;
};

// A surface on which other objects can be stacked (Bottom)
class Stackable : public Affordance
{
public:

  Stackable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Stackable();

  double extentsX, extentsY;
  unsigned int normalDir;   // Up-axis, default is 2
};

// A containment where stuff can be put into.
class Containable : public Affordance
{
public:

  Containable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Containable();
  virtual void print() const;
  double getVolume() const;

  double maxVolume;
  std::vector<std::pair<std::string,double>> liquidIngredients;
};

// An opening where stuff can be poured out of.
class Pourable : public Affordance
{
public:

  Pourable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Pourable();
};

// Something that can be pushed like a button.
class PointPushable : public Affordance
{
public:

  PointPushable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~PointPushable();
};

// Something that can push on a button.
class PointPokable : public Affordance
{
public:

  PointPokable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~PointPokable();
};

// Something that can push on a button.
class Hingeable : public Affordance
{
public:

  Hingeable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Hingeable();
};

// Something solid that lets itself dispense from a container (like an olive)
class Dispensible : public Affordance
{
public:

  Dispensible(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Dispensible();
};

// Something liquid that lets itself dispense from a container (like tea)
class Wettable : public Affordance
{
public:

  Wettable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Wettable();
};

// Something that can be opened and closed (fridge, drawer ...)
class Openable : public Affordance
{
public:

  Openable(const xmlNodePtr node);
  virtual Affordance* clone() const;
  virtual ~Openable();

  bool isOpen;
};

} // namespace aff

#endif // AFF_AFFORDANCE_H
