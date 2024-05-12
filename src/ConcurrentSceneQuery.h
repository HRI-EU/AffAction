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

  /*! \brief Legacy function that returns a really large json with the overal
   *         scene information. See SceneJsonHelpers.cpp for details.
   */
  nlohmann::json getSceneState();

  /*! \brief Returns empty json if no occluded objects exist, or an array of occluded objects
   *         for the agent, for example:
   *         {"occluded": [{"name": "entity name 1", "instance_id": "entity id 1"},
   *                       {"name": "entity name 2", "instance_id": "entity id 2"}]}
   */
  nlohmann::json getOccludedObjectsForAgent(const std::string& agentName);

  /*! \brief Returns a json of objects that occlude the object with objectName for the
   *         agent given with agentName. The occluders are sorted by increasing distance
   *         to the agent's eye, for example:
   *         {"occluded_by": ["entity name 1", "entity name 2"] }
   *         The jeson is empty if the agent with agentName does not exist, or the entity
   *         with objectName does not exist. If several objects with the given name exist,
   *         the function will return something, but it might not be consistent.
   */
  nlohmann::json getObjectOccludersForAgent(const std::string& agentName,
                                            const std::string& objectName);

  /*! \brief Returns the name of the kinematic parent of the object, or an empty
   *         string if:
   *         - objectName is not the name / type of an AffordanceEntity
   *         - the object has no parent.
   *         If objectName refers to more than one entities, the first one found is used.
   */
  std::string getParent(const std::string& objectName);

  /*! \brief Returns the name of the holding hand of the object, or an empty
   *         string if:
   *         - objectName is not the name / type of an AffordanceEntity
   *         - the object is not held in a hand.
   *         If objectName refers to more than one entities, the first one found is used.
   */
  std::string getHoldingHand(const std::string& objectName);

  /*! \brief Plans a detailed action sequence given a vector of (possibly
   *         abstract) actions.
   *
   *  \param[in]  searchType       See enum PredictionTree::SearchType. DFSMT is the fastest.
   *  \param[in]  actions          Vector of (possibly abstract) actions.
   *  \param[out] errMsg           Error message for tracking down reason for no solution.
   *  \param[in]  maxThreads       Upper limit on number of threads used. If it
   *                               is 0, then the function will automatically
   *                               detect the best possible value for the fastest
   *                               computation result.
   *  \param[in] earlyExitSearch   Stop search after first successful solution.
   *  \param[in] earlyExitAction   Stop action prediction after first encountered error.
   *  \return Prediction tree.
   */
  std::unique_ptr<PredictionTree> planActionTree(PredictionTree::SearchType searchType,
                                                 const std::vector<std::string>& actions,
                                                 double dt,
                                                 std::string& errMsg,
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
