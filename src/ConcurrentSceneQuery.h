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

/*!
 * \brief Class to access scene and graph state concurrently from other threads as
 *        the one running the process loop.
 *
 * For each function call, the class creates a copy of the scene, the graph, and
 * (optionally) of the broadphase model. This is done exclusive to the step()
 * function of the sim object, so that no concurrency issues arise. Copying the
 * state is not a very expensive function, but it might block the step method for
 * some time. It is advisable to not call the ConcurrentSceneQuery methods in a loop
 * or many iterations to not block the run() loop of the sim object. This class mainly
 * was written for the python module.
 */

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
  nlohmann::json getObjectInCamera(const std::string& entityName,
                                   const std::string& cameraName);

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
                                                 size_t maxThreads=0,
                                                 bool earlyExitSearch=true,
                                                 bool earlyExitAction=true);

  /*! \brief Returns true if a HumanAgent is busy, false otherwise. The state
   *         of being busy is true if one of the agent's hands is closer to
   *         an entity as the given distanceThreshold. Objects that are not
   *         collideable (have no shape with distance calculation flag) as well
   *         as very big objects like a table (one of the AABB extents > 1m)
   *         will be ignored.
   */
  bool isAgentBusy(const std::string& agentName, double distanceThreshold);

  /*! \brief Returns the pan and tilt angles for the robot agent with the agentName
   *         for the head pose looking at the entity with the type sceneEntity. If
   *         several entities with the type exist, the first one found is taken. A
   *         sceneEntity can also be an agent. Then, the pan / tilt angles are calculated
   *         to look to the agent's head.
   *         If the function fails, the returned vector will be empty. This is the case if:
   *         - agentName does not refer to a RobotAgent instance
   *         - objectName is not the name / type of an AffordanceEntity
   *         - the pan / tile joints cannot be found
   *         - The function does not converge (uses IK internally)
   */
  std::vector<double> getPanTilt(const std::string& agentName,
                                 const std::string& sceneEntity);


private:

  void update(bool withBroadphase = false);
  void updateNoMutex(bool withBroadphase = false);

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
