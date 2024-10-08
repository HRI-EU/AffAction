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

#ifndef AFF_GAZECOMPONENT_H
#define AFF_GAZECOMPONENT_H

#include "ComponentBase.h"
#include "ActionScene.h"

#include <Rcs_graph.h>

#include <tuple>


namespace aff
{

class GazeComponent : public ComponentBase
{
public:

  /*! \brief Constructs GazeComponent.
   *
   * \param[in] parent    Entity class responsible for event subscriptions
   */
  GazeComponent(EntityBase* parent, const std::string& gazingBody,
                int dirIdx = 1);

  /*! \brief Unsubscribes and deletes all previously allocated memory.
   *         There is no thread that needs to be stopped.
   */
  virtual ~GazeComponent() = default;

  void addSceneToAttend(const ActionScene& scene, const RcsGraph* graph);

private:

  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* curent);
  const RcsBody* getBody(const RcsGraph* graph, const std::string& bdyName, int& bdyId);

  std::string gazingBody;
  int id_gazeBody;
  int gazeDirectionIdx;   // 0: x, 1: y, 2: z

  struct BodyIntersection
  {
    BodyIntersection()
    {
      bdyId = -1;
      gazeAngle = -1.0;
    }

    std::string name;
    std::string bdyName;
    int bdyId;
    double gazeAngle;
  };

  std::vector<BodyIntersection> objectsToAttend;
};

}

#endif   // AFF_GAZECOMPONENT_H
