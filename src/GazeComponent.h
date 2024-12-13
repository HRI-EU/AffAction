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
#define DEFAULT_MAX_GAZE_ANGLE_DIFF 120.0
#define DEFAULT_MAX_DURATION_GAZE_DATA 20.0

namespace aff
{
  /*! \brief  Struct representing a single gaze data point.
 *  Each GazeDataPoint includes the following information:
 *  - Time of the gaze event in seconds.
 *  - Names of objects within the gaze field.
 *  - Angular differences (in degrees) between the head direction and objects.
 *  - Distances from the agent head to each object.
 *  - Angular differences between the head direction and the objects in the XY plane.
 *  - Angular differences between the head direction and the objects in the XZ plane.
 *  - Head velocity in degrees per second.
 */

struct GazeDataPoint {
    double time;  // Time of the gaze event (seconds)
    std::vector<std::string> objectNames;  // Names of the objects in the gaze field
    std::vector<double> angleDiffs;  // Angular differences to objects (deg)
    std::vector<double> distances;   // Distance to each object (m)
    std::vector<double> angleDiffsXY;  // Angular differences in XY plane (deg)
    std::vector<double> angleDiffsXZ;  // Angular differencts in YZ plane (deg)
    double gazeVel; // Head velocity (deg/s)

    /*! \brief Constructs a GazeDataPoint with specified values.
     *
     * \param[in] t           Time of the gaze event.
     * \param[in] names       Names of objects in the gaze field.
     * \param[in] angleDiffs_ Angular differences between the head direction and objects.
     * \param[in] distances_  Distances from the head to objects.
     * \param[in] vel         Head velocity.
     * \param[in] angleDiffsXY_ Angular differences between the head direction and the objects in the XY plane.
     * \param[in] angleDiffsXZ_ Angular differences between the head direction and the objects in the XZ plane.
     */
    GazeDataPoint(double t, const std::vector<std::string>& names, const std::vector<double>& angleDiffs_,
     const std::vector<double>& distances_, double vel, const std::vector<double>& angleDiffsXY_, const std::vector<double>& angleDiffsXZ_)
        : time(t), objectNames(names), angleDiffs(angleDiffs_),distances(distances_), gazeVel(vel),
          angleDiffsXY(angleDiffsXY_), angleDiffsXZ(angleDiffsXZ_) {}
};


class GazeComponent : public ComponentBase
{
public:

  /*! \brief Constructs GazeComponent
   *
   * \param[in] parent    Entity class responsible for event subscriptions
   * \param[in] agentName Name of the agent whose gaze is being tracked
   * \param[in] gazingBody Name of the body that the agent is gazing with (e.g. head)
   * \param[in] dirIdx    Gaze direction index (default: 1 for y-axis)
   */
   GazeComponent(EntityBase* parent, const std::string& agentName, const std::string& gazingBody,
                int dirIdx = 1);

  /*! \brief Unsubscribes and deletes all previously allocated memory.
   *         There is no thread that needs to be stopped.
   */
  virtual ~GazeComponent();

  /*! \brief Adds a scene to attend to the GazeComponent.
   *
   * \param[in] scene ActionScene object containing the scene to attend.
   * \param[in] graph RcsGraph object representing the scene.
   */
  void addSceneToAttend(const ActionScene& scene, const RcsGraph* graph);


  /*! \brief Returns read-only access to the gaze data deque.
   *
   * The gazeData deque contains GazeDataPoint objects representing gaze data points from the current instant
   * to a maximum duration specified by maxDurationGazeData into the past.
   * 
   * \return Pointer to a deque containing gaze data points.
   */
  const std::deque<GazeDataPoint>* getGazeData() {
      return &gazeData;  
  }


  /*! \brief Returns the name of the agent whose gaze is being tracked.
   *
   * \return Name of the agent.
   */
  std::string getAgentName() const {
      return agentName;
  }

private:

  /*! \brief Computes gaze-related data after the graph is updated.
  *  This method is called every time the graph is updated. It performs the following:
  *  - Computes the angular differences between the head direction and all objects in the scene.
  *  - Filters objects based on gaze angle and other criteria.
  *  - Adds a new gaze data point to the internal deque, including details about the current
  *    gaze velocity, angular differences, and distances to objects.
  *
  * \param[in] desired The desired state of the graph.
  * \param[in] current The current state of the graph.
  */
  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* curent);

  /*! \brief Retrieves a body from the graph by name and ID.
   *
   * \param[in] graph The graph containing the body.
   * \param[in] bdyName The name of the body to retrieve.
   * \param[in] bdyId The ID of the body to retrieve.
   */
  const RcsBody* getBody(const RcsGraph* graph, const std::string& bdyName, int& bdyId);


  /*! \brief Adds a new gaze data point to the internal deque.
   *
   * \param[in] time Time of the gaze event.
   * \param[in] objectNames Names of objects in the gaze field.
   * \param[in] diffAngles Angular differences between the head direction and objects.
   * \param[in] distances Distances from the head to objects.
   * \param[in] gazeVel Head velocity.
   * \param[in] angleDiffsXY Angular differences between the head direction and the objects in the XY plane.
   * \param[in] angleDiffsXZ Angular differences between the head direction and the objects in the XZ plane.
   */
  void addGazeDataPoint(double time, const std::vector<std::string>& objectNames, const std::vector<double>& diffAngles, 
                        const std::vector<double>& distances, double gazeVel, const std::vector<double>& angleDiffsXY, 
                        const std::vector<double>& angleDiffsXZ);

  /*! \brief Removes the oldest gaze data point from the deque.
   */
  void removeOldestGazeDataPoint();

  /*! \brief Computes points on the surface of an AABB.
   *
   * \param[in] xyzMin Minimum coordinates of the AABB.
   * \param[in] xyzMax Maximum coordinates of the AABB.
   * \param[out] pointsObject Vector of points on the AABB surface.
   * \param[in] distance Distance between points on the AABB surface.
   */
  void getPointsAABBSurface(const double (&xyzMin)[3], const double (&xyzMax)[3], std::vector<std::array<double,3>>& pointsObject, const double & distance);

  std::string gazingBody; // Name of the body used for gaze direction
  int id_gazeBody; // ID of the body used for gaze direction
  int gazeDirectionIdx;   // Index for gaze direction 0: x, 1: y, 2: z
  double prevHeadDirection[3]; // Previous head direction
  std::string agentName; // Name of the agent

  struct BodyIntersection
  {
    BodyIntersection()
    {
      bdyId = -1;
      gazeAngle = -1.0;
    }

    std::string name; // Name of the object
    std::string bdyName; // Node name of the body
    int bdyId; // ID of the body
    double gazeAngle; // Angular difference between the head direction and the object
    double objectPointDistance; // Distance from the head to the object
    double gazeAngleXY; // Angular difference in the XY plane
    double gazeAngleXZ; // Angular difference in the XZ plane
  };

  std::vector<BodyIntersection> objectsToAttend; // List of objects to attend for the gaze tracking
  std::deque<GazeDataPoint> gazeData; // Deque of gaze data points
  double totalDurationGazeData; // Total duration of gaze data in the deque (seconds)
  

  };

}

#endif   // AFF_GAZECOMPONENT_H
