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

#ifndef AFF_LEROBOTCOMPONENT_H
#define AFF_LEROBOTCOMPONENT_H


#include "ComponentBase.h"
#include "ActionScene.h"
#include "IKTeleOp.h"
#include "json.hpp"

#include <ControllerBase.h>


namespace aff
{

/*! \brief Data collection class
 */
class LeRobotComponent : public ComponentBase
{
public:

  /*! \brief Constructs LeRobot data collection with the task vector and graph
   *         from the argument controller.
   *
   * \param[in] parent     Entity class responsible for event subscriptions
   * \param[in] controller Controller with all task variables and (possibly)
   *                       a collision model. Will be cloned and not stored.
   */
  LeRobotComponent(EntityBase* parent,
                   const std::string& dataCollectionXmlFileName);

  /*! \brief Destroys the IkSolverRMR and all other allocated memory.
   */
  virtual ~LeRobotComponent() = default;

  std::map<std::string, double> computeLeRobotData(bool current);
  nlohmann::json getIncomingCommand() const;
  void updateData(RcsGraph* desired, RcsGraph* current);

private:




  /*
    LeRobotCollectedData data;

    data["joint_left_shoulder_joint"] -> double

    data["pos_left_hand_0"] -> double
    ...
    data["pos_left_hand_2"] -> double

    data["rm_left_hand_00"] -> double
    ...
    data["rm_left_hand_22"] -> double



    data["fts_base_x"] -> double
    ...
    data["fts_base_thz"] -> double
   */
  mutable std::mutex leRobotStateMtx;
  std::vector<double> lerobot_state_curr, lerobot_state_des;
  std::unique_ptr<Rcs::ControllerBase> lerobot_ctrl_curr, lerobot_ctrl_des;



  // Commands. We don't need a mutex here, since all is
  // synced through the event system.
  mutable std::mutex incomingCommandMtx;
  nlohmann::json incomingCommand;

  // Asynchronously called (publish) from python
  void onIncomingCommand(nlohmann::json command);
  void onInitFromState(const RcsGraph* target);

  /*! \brief We disallow copying and assigning this class.
   */
  LeRobotComponent(const LeRobotComponent&) = delete;
  LeRobotComponent& operator=(const LeRobotComponent&) = delete;
};

}

#endif   // AFF_LEROBOTCOMPONENT_H
