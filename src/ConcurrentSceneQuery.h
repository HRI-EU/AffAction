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

#ifndef AFF_CONCURRENTSCENEQUERY_H
#define AFF_CONCURRENTSCENEQUERY_H



#include "ActionScene.h"
#include "PredictionTree.h"
#include "json.hpp"

#include <Rcs_broadphase.h>

#include <mutex>
#include <memory>



namespace aff
{

class ExampleActionsECS;





class ConcurrentSceneQuery
{
public:

  ConcurrentSceneQuery(const ExampleActionsECS* sim);
  virtual ~ConcurrentSceneQuery();

  /*! \brief Returns empty json if there are no objects or a json in the form:
   *         {"objects": ['iphone', 'red_glass', 'fanta_bottle'] }
   */
  nlohmann::json getObjects();

  /*! \brief Returns empty json if there are no objects or a json in the form:
   *         {"agents": ['Daniel', 'Felix', 'Robot'] }
   */
  nlohmann::json getAgents();
  nlohmann::json getSceneState();
  nlohmann::json getOccludedObjectsForAgent(const std::string& agentName);
  nlohmann::json getObjectOccludersForAgent(const std::string& agentName,
                                            const std::string& objectName);

  /*! \brief Plans a detailed action sequence given a vector of (possibly
   *         abstract) actions.
   *
   *  \param[in] actions          Vector of (possibly abstract) actions.
   *  \param[in] maxThreads       Upper limit on number of threads used. If it
   *                              is 0, then the function will automatically
   *                              detect the best possible value for the fastest
   *                              computation result.
   *  \return Vector of detailed atomic actions constituting to the sequence,
   *          or empty vector if no valid solution could be found.
   */
  std::vector<std::string> planActionSequence(const std::vector<std::string>& actions,
                                              size_t maxThreads = 0);
  std::unique_ptr<PredictionTree> planActionTree(const std::vector<std::string>& actions,
                                                 size_t maxThreads = 0);
  std::unique_ptr<PredictionTree> planActionTreeDFT(const std::vector<std::string>& actions,
                                                    size_t maxThreads=0,
                                                    bool earlyExitSearch=true,
                                                    bool earlyExitAction=true);

  bool isAgentBusy(const std::string& agentName, double distanceThreshold);

  std::vector<double> getPanTilt(const std::string& agentName,
                                 const std::string& objectName);


private:

  void update(bool withBroadphase=false);

  const ExampleActionsECS* sim;
  RcsGraph* graph;
  RcsBroadPhase* broadphase;
  ActionScene scene;
  std::mutex reentrancyLock;
};



class SceneQueryPool
{
public:
  SceneQueryPool(const ExampleActionsECS* sim, size_t nInstances);

  std::shared_ptr<ConcurrentSceneQuery> instance();

  static void test(const ExampleActionsECS* sim);

private:
  std::vector<std::shared_ptr<ConcurrentSceneQuery>> queries;
  std::mutex mtx;
};


}   // namespace aff

#endif // AFF_CONCURRENTSCENEQUERY_H
