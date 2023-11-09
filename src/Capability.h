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

#ifndef AFF_CAPABILITY_H
#define AFF_CAPABILITY_H

#include "Affordance.h"


namespace aff
{

class Capability
{
public:

  enum class Type
  {
    Capability,
    GraspCapability,
    PowergraspCapability,
    PincergraspCapability,
    TwistgraspCapability,
    CirculargraspCapability,
    PalmgraspCapability,
    FingerpushCapability,
    GazeCapability
  };

  static std::map<std::string, Type> typeMap;

  std::string frame;
  Type classType;
  std::string className;

  std::vector<Affordance::Type> affordanceTypes;

  static Type typeFromString(const std::string& className);
  static std::string stringFromType(Type aType);

  Capability(const xmlNodePtr node);
  virtual ~Capability();
  virtual Capability* clone() const;
  virtual void print() const;
  virtual bool check(const RcsGraph* graph) const;
};

class GraspCapability : public Capability
{
public:

  GraspCapability(const xmlNodePtr node);
  virtual ~GraspCapability();
  virtual Capability* clone() const;
};

class PowergraspCapability : public GraspCapability
{
public:

  PowergraspCapability(const xmlNodePtr node);
  virtual ~PowergraspCapability();
  virtual Capability* clone() const;
};

class PincergraspCapability : public GraspCapability
{
public:

  PincergraspCapability(const xmlNodePtr node);
  virtual ~PincergraspCapability();
  virtual Capability* clone() const;
};

class TwistgraspCapability : public GraspCapability
{
public:

  TwistgraspCapability(const xmlNodePtr node);
  virtual ~TwistgraspCapability();
  virtual Capability* clone() const;
};

class CirculargraspCapability : public GraspCapability
{
public:

  CirculargraspCapability(const xmlNodePtr node);
  virtual ~CirculargraspCapability();
  virtual Capability* clone() const;
  virtual bool check(const RcsGraph* graph) const;

  std::string frame2;
};

class PalmgraspCapability : public GraspCapability
{
public:

  PalmgraspCapability(const xmlNodePtr node);
  virtual ~PalmgraspCapability();
  virtual Capability* clone() const;
};

class FingerpushCapability : public Capability
{
public:

  FingerpushCapability(const xmlNodePtr node);
  virtual ~FingerpushCapability();
  virtual Capability* clone() const;
};

class GazeCapability : public Capability
{
public:

  GazeCapability(const xmlNodePtr node);
  virtual ~GazeCapability();
  virtual Capability* clone() const;
};


} // namespace aff

#endif // AFF_CAPABILITY_H
