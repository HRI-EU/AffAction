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

#ifndef AFF_SCENEJSONHELPERS_H
#define AFF_SCENEJSONHELPERS_H

#include "ActionScene.h"
#include "json.hpp"



namespace aff
{

void getSceneState(nlohmann::json& stateJson,
                   const ActionScene* scene,
                   const RcsGraph* graph);

nlohmann::json getOccludedObjectsForAgent(const std::string& agentName,
                                          const ActionScene* scene,
                                          const RcsGraph* graph);

nlohmann::json getObjectOccludersForAgent(const std::string& agentName,
                                          const std::string& objectName,
                                          const ActionScene* scene,
                                          const RcsGraph* graph);

/*! \brief Returns empty json if entityName does not refer to one AffordanceEntity or
 *         HumanAgent, or an array of positions and 8 AABB-vertices of the entity or agent,
 *         projected into the frame of the given camera. The AABB is computed as
 *
 *         - the envelope around all distance shapes of an AffordanceEntitie's body
 *         - the envelope around all markers of a HumanAgent
 *
 *         If no AABB can be found, the 'vertex' part of the json will be skipped. This
 *         can be the case if there are no shapes with distance calculation flag in the
 *         RcsBody that is associated with an AffordanceEntity, or if there are no
 *         markers assigned to a HumanAgent referred to by entityName.
 *
 *         The returned json looks for example like this:
 *
 *         {'vertex': [[0.2, 0.4, 0.9], [0.2, 0.4, 0.9], [0.2, 0.4, 0.9], [0.2, 0.4, 0.9],
 *                    [0.2, 0.4, 0.9], [0.2, 0.4, 0.9], [0.2, 0.4, 0.9], [0.2, 0.4, 0.9]],
 *          'x': 0.2, 'y': 0.4, 'z': 0.9}
 */
nlohmann::json getObjectInCamera(const std::string& objectName,
                                 const std::string& cameraName,
                                 const ActionScene* scene,
                                 const RcsGraph* graph);

bool isAgentBusy(const std::string& agentName,
                 const ActionScene* scene,
                 const RcsGraph* graph,
                 double distanceThreshold);


}   // namespace aff

#endif // AFF_SCENEJSONHELPERS_H
