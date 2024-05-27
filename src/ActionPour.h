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

#ifndef AFF_ACTIONPOUR_H
#define AFF_ACTIONPOUR_H


#include "ActionBase.h"



namespace aff
{

class ActionPour : public ActionBase
{
public:

  ActionPour(const ActionScene& domain,
             const RcsGraph* graph,
             std::vector<std::string> params);

  virtual ~ActionPour();
  std::unique_ptr<ActionBase> clone() const override;

  tropic::TCS_sptr createTrajectory(double t_start, double t_end) const;
  std::vector<std::string> getManipulators() const;
  std::string getActionCommand() const;
  double getDefaultDuration() const;

protected:

  void init(const ActionScene& domain,
            const RcsGraph* graph,
            const std::string& objectToPourFrom,
            const std::string& objectToPourTo,
            double amountToPour);

  std::vector<std::string> createTasksXML() const;
  void performLiquidTransition(const AffordanceEntity* pourFromAff,
                               const AffordanceEntity* pourToAff);

  std::string bottle;
  std::string glas;     // Name of Containable frame of receiving entity
  std::string roboBaseFrame;

  std::string taskRelPos;
  std::string taskBottleOri;
  std::string taskGlasOri;
  std::string taskGlasPosX;
  std::string taskGlasPosZ;

  std::vector<std::string> usedManipulators;
  std::vector<std::string> particlesToPour;

  bool glasInHand;
  bool bottleInRightHand;
  double pouringVolume;
};

}   // namespace aff



#endif // AFF_ACTIONPOUR_H
