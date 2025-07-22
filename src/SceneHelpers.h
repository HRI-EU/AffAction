/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/


#ifndef AFF_SCENEHELPERS_H
#define AFF_SCENEHELPERS_H

#include "EntityBase.h"
#include "ActionScene.h"

namespace aff
{

double getWallclockTime();

std::string recognize_faces(EntityBase& entity,
                            const std::string& boundingBox,
                            int n_iterations,
                            double timeout_in_seconds);
std::string recognize_agent_face(EntityBase& entity,
                                 const ActionScene* scene,
                                 const std::string& agentName,
                                 int n_iterations,
                                 double timeout_in_seconds);

bool track_facemesh(EntityBase& entity, const std::string& boundingBox,
                    int n_iterations, double timeout_in_seconds);
bool track_agent_facemesh(EntityBase& entity, const ActionScene* scene,
                          const std::string& agentName, int n_iterations, double timeout_in_seconds);

void add_agent_welcome_subscriber(EntityBase& entity, const ActionScene* scene);

}   // namespace aff

#endif   // AFF_FACETRACKER_H
