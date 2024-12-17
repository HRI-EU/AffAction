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

#include "GazeComponent.h"

#include <Rcs_timer.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>

#include <algorithm>
#include <iomanip>  // for std::setprecision and std::fixed


#define GAZEOBJECT_MAX_EXTENTS (1.0)   // Ignore everything with an extent larger than this

/*
This class computes the intersection of a gazing body (head) with a set of objects
that it should attend to. The gaze direction is the head's y-axis.
 */

namespace aff
{

GazeComponent::GazeComponent(EntityBase* parent, const std::string& agentName_, const std::string& gazingBody_, int dirIdx) :
  ComponentBase(parent), agentName(agentName_),gazingBody(gazingBody_), id_gazeBody(-1), gazeDirectionIdx(dirIdx)
{
  prevHeadDirection[0] = 0;
  prevHeadDirection[1] = 0;
  prevHeadDirection[2] = 0;

  subscribe("PostUpdateGraph", &GazeComponent::onPostUpdateGraph);

}

GazeComponent::~GazeComponent()
{

}

void GazeComponent::addSceneToAttend(const ActionScene& scene, const RcsGraph* graph)
{
  auto ntts = scene.getSceneEntities();

  for (const auto& ntt : ntts)
  {
    RLOG_CPP(1, "Adding " << ntt->name << " to GazeComponent");
    const RcsBody* bdy = RcsGraph_getBodyByName(graph, ntt->bdyName.c_str());
    RCHECK_MSG(bdy, "%s", ntt->bdyName.c_str());

    if (ntt->bdyName == gazingBody || ntt->bdyName == agentName)
    {
      continue;
    }

    if (ntt->bdyName.find("robot") != std::string::npos)
    {
      continue;
    }


    double xyzMin[3], xyzMax[3];
    bool aabbValid = RcsGraph_computeBodyAABB(graph, bdy->id, -1, xyzMin, xyzMax, NULL);
    if (aabbValid)
    {
      double extents[3];
      Vec3d_sub(extents, xyzMax, xyzMin);
      if (VecNd_maxAbsEle(extents, 3) > GAZEOBJECT_MAX_EXTENTS)
      {
        RLOG(0, "Ignoring object %s with extents %f %f %f",
             bdy->name, extents[0], extents[1], extents[2]);
        continue;
      }
    }

    BodyIntersection bi;
    bi.name = ntt->name;
    bi.bdyName = ntt->bdyName;
    bi.bdyId = bdy->id;
    bi.gazeAngle = -1.0;   // Negative: uninitialized


    objectsToAttend.push_back(bi);

  }


}

const RcsBody* GazeComponent::getBody(const RcsGraph* graph, const std::string& bdyName, int& bdyId)
{
  const RcsBody* bdy = nullptr;

  if ((bdyId == -1) || (std::string(graph->bodies[bdyId].name) != bdyName))
  {
    bdy = RcsGraph_getBodyByName(graph, bdyName.c_str());
    RCHECK_MSG(bdy, "%s", bdyName.c_str());
    bdyId = bdy->id;
  }
  else
  {
    bdy = &graph->bodies[bdyId];
  }

  RCHECK_MSG(bdy, "Body %s with id %d", bdyName.c_str(), bdyId);

  return bdy;
}


void GazeComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  double t_calc = Timer_getSystemTime();

  const RcsGraph* graph = desired;

  const RcsBody* head = getBody(graph, gazingBody, id_gazeBody);
  const double* eyePos = head->A_BI.org;
  const double* gazeDir = head->A_BI.rot[gazeDirectionIdx];   // y-axis

  // Lets try the angle difference in the x-y plane

  double gazeDirXY[3];
  Vec3d_set(gazeDirXY, gazeDir[0], gazeDir[1], 0);
  double gazeDirXZ[3];
  Vec3d_set(gazeDirXZ, gazeDir[0], 0, gazeDir[2]);


  // This is kind of a saccade suppression. If the gaze point moves quickly, we
  // suppress the attention mechanism so that objects we just pass by gazing
  // are ignored. Otherwise, there will be spurious detections of objects to
  // attend that are just in the way
  double gazeVel = Vec3d_getLength(head->omega);

  bool useClosestPointAABB = true;

  if (Vec3d_getLength(prevHeadDirection)!=0)
  {
    double gazeOmega[3];
    Vec3d_sub(gazeOmega, gazeDir, prevHeadDirection);

    gazeVel = Vec3d_getLength(gazeOmega);

  }

  Vec3d_set(prevHeadDirection, gazeDir[0], gazeDir[1], gazeDir[2]);

  // Now we go through all objects
  for (auto& o : objectsToAttend)
  {
    o.gazeAngle = std::numeric_limits<double>::max();
    o.objectPointDistance = std::numeric_limits<double>::max();


    const RcsBody* obj = getBody(graph, o.bdyName, o.bdyId);
    double eye_obj[3];
    const double* objPos = obj->A_BI.org;

    double xyzMin[3], xyzMax[3], centroid[3];
    bool aabbValid = RcsGraph_computeBodyAABB(graph, obj->id, -1, xyzMin, xyzMax, NULL);


    if (aabbValid)
    {

      if (useClosestPointAABB &&  agentName!= "Johnnie")
      {
        // RLOG(0, "Using AABB points for object %s", o.bdyName.c_str());
        std::vector<std::array<double,3>> pointsObject;
        // Get  points for each aabb
        getPointsAABBSurface(xyzMin, xyzMax, pointsObject, 0.015);

        for (const auto& point: pointsObject)
        {
          double p[3];
          Vec3d_set(p, point[0], point[1], point[2]);
          // RLOG(1, "Point: (%f, %f, %f)", p[0], p[1],p[2]);
          Vec3d_sub(eye_obj, p, eyePos);

          double eye_objXY[3];
          Vec3d_set(eye_objXY, eye_obj[0], eye_obj[1], 0);
          double eye_objXZ[3];
          Vec3d_set(eye_objXZ, eye_obj[0], 0, eye_obj[2]);

          double angle = Vec3d_diffAngle(eye_obj, gazeDir);
          double angleXY = Vec3d_diffAngle(eye_objXY, gazeDirXY);
          double angleXZ = Vec3d_diffAngle(eye_objXZ, gazeDirXZ);
          // RLOG(1, "Angle %f", angle*180.0/M_PI);
          if (angle<o.gazeAngle)
          {
            o.gazeAngle = angle;
            o.objectPointDistance = Vec3d_distance(p, eyePos);
            o.gazeAngleXY = angleXY;
            o.gazeAngleXZ = angleXZ;
          }
        }

      }
      else
      {
        // Use only centroid
        Vec3d_set(centroid, 0.5*(xyzMin[0]+xyzMax[0]), 0.5*(xyzMin[1]+xyzMax[1]), 0.5*(xyzMin[2]+xyzMax[2]));
        objPos = centroid;
        Vec3d_sub(eye_obj, objPos, eyePos);
        double eye_objXY[3];
        Vec3d_set(eye_objXY, eye_obj[0], eye_obj[1], 0);
        double eye_objXZ[3];
        Vec3d_set(eye_objXZ, eye_obj[0], 0, eye_obj[2]);
        o.objectPointDistance = Vec3d_distance(objPos, eyePos);
        o.gazeAngle = Vec3d_diffAngle(eye_obj, gazeDir);
        o.gazeAngleXY = Vec3d_diffAngle(eye_objXY, gazeDirXY);
        o.gazeAngleXZ = Vec3d_diffAngle(eye_objXZ, gazeDirXZ);
      }

    }


  }

  // Sorting based on gazeAngle, smaller values first
  //std::partial_sort(objectsToAttend.begin(), objectsToAttend.begin()+3, objectsToAttend.end(), [](const BodyIntersection& a, const BodyIntersection& b)
  //{
  //  return a.gazeAngle < b.gazeAngle;
  //});
  std::sort(objectsToAttend.begin(), objectsToAttend.end(), [](const BodyIntersection& a, const BodyIntersection& b)
  {
    return a.gazeAngle < b.gazeAngle;
  });

  if (!objectsToAttend.empty())
  {
    REXEC(2)
    {
      RLOG_CPP(1, objectsToAttend.size() << " gaze objects:");
      for (auto& o : objectsToAttend)
      {
        std::cout << o.name << ": " << (180.0/M_PI)*o.gazeAngle << std::endl;
      }
    }


    // Prepare data to store
    std::vector<std::string> objectNames;
    std::vector<double> gazeAngles;
    std::vector<double> distances;
    std::vector<double> gazeAnglesXY;
    std::vector<double> gazeAnglesXZ;
    for (const auto& o : objectsToAttend)
    {
      if (o.gazeAngle*(180.0/M_PI) > DEFAULT_MAX_GAZE_ANGLE_DIFF)
      {
        break;
      }
      objectNames.push_back(o.name);
      gazeAngles.push_back((180.0 / M_PI) * o.gazeAngle);  // Convert to degrees
      distances.push_back(o.objectPointDistance);
      gazeAnglesXY.push_back((180.0 / M_PI) * o.gazeAngleXY);
      gazeAnglesXZ.push_back((180.0 / M_PI) * o.gazeAngleXZ);
    }

    // Add data to deque
    double currentTime = Timer_getSystemTime();
    if (!gazeData.empty())
    {
      gazeVel = gazeVel*(180.0/M_PI)*100.0; // 100 Hz
    }
    else
    {
      gazeVel = 0;
    }
    addGazeDataPoint(currentTime, objectNames, gazeAngles, distances, gazeVel, gazeAnglesXY, gazeAnglesXZ);


    t_calc = Timer_getSystemTime() - t_calc;


    RLOG(1, "Took %.3f usec, gazeVel is %.3f", 1000.0 * t_calc, gazeVel);
    RLOG(1, "Omega vector: (%.3f, %.3f, %.3f)", head->omega[0], head->omega[1], head->omega[2]);
    RLOG_CPP(1, "Number of gazeData elements stored: " << gazeData.size() << " with a total duration: " << totalDurationGazeData);
    RLOG_CPP(1, "Oldest object in deque: " << gazeData.front().objectNames[0] << " Newest object in deque: " << gazeData.back().objectNames[0]);
  }
}



void GazeComponent::addGazeDataPoint(double time, const std::vector<std::string>& objectNames, const std::vector<double>& angleDiffs,
                                     const std::vector<double>& distances, double gazeVel, const std::vector<double>& angleDiffsXY,
                                     const std::vector<double>& angleDiffsXZ)
{

  if (!gazeData.empty())
  {
    totalDurationGazeData = time - gazeData.front().time;
  }

  // Add the new gaze data
  gazeData.emplace_back(time, objectNames, angleDiffs, distances, gazeVel, angleDiffsXY, angleDiffsXZ);

  // Remove oldest data points if total duration exceeds the maxDurationGazeData
  while (totalDurationGazeData > DEFAULT_MAX_DURATION_GAZE_DATA && !gazeData.empty())
  {
    removeOldestGazeDataPoint();
  }
}

// Remove the oldest gaze data from the deque
void GazeComponent::removeOldestGazeDataPoint()
{

  gazeData.pop_front();  // Remove the oldest element
  if (gazeData.size() > 1)
  {
    totalDurationGazeData = gazeData.back().time-gazeData.front().time;
  }

}

void GazeComponent::getPointsAABBSurface(const double (&xyzMin)[3], const double (&xyzMax)[3], std::vector<std::array<double,3>>& pointsObject, const double& distance)
{

  pointsObject.clear();

  double lengthX = xyzMax[0] - xyzMin[0];
  double lengthY = xyzMax[1] - xyzMin[1];
  double lengthZ = xyzMax[2] - xyzMin[2];

  // RLOG(0, "LENGTHS: (%f, %f, %f)", lengthX, lengthY, lengthZ);
  int stepsX = static_cast<int>(lengthX / distance) + 1;
  int stepsY = static_cast<int>(lengthY / distance) + 1;
  int stepsZ = static_cast<int>(lengthZ / distance) + 1;

  double stepX = lengthX / (stepsX);
  double stepY = lengthY / (stepsY);
  double stepZ = lengthZ / (stepsZ);


  // Reserve memory for points
  pointsObject.reserve(2 * (stepsX * stepsY + stepsY * stepsZ + stepsX * stepsZ));


  for (int i = 0; i < stepsX; ++i)
  {
    double x = xyzMin[0] + i * stepX;
    for (int j = 0; j < stepsY; ++j)
    {
      std::array<double, 3> point = {x, xyzMin[1] + j * stepY, xyzMax[2] };
      pointsObject.push_back(point);
    }
  }

  for (int i = 0; i < stepsY; ++i)
  {
    double y = xyzMin[1] + i * stepY;
    for (int j = 0; j < stepsZ; ++j)
    {
      std::array<double, 3> point = { xyzMin[0], y, xyzMin[2] + j * stepZ };
      pointsObject.push_back(point);
    }
  }

  for (int i = 0; i < stepsY; ++i)
  {
    for (int j = 0; j < stepsZ; ++j)
    {
      std::array<double, 3> point = { xyzMax[0], xyzMin[1] + i * stepY, xyzMin[2] + j * stepZ };
      pointsObject.push_back(point);
    }
  }

  for (int i = 0; i < stepsX; ++i)
  {
    double x = xyzMin[0] + i * stepX;
    for (int j = 0; j < stepsZ; ++j)
    {
      std::array<double, 3> point = { x, xyzMin[1], xyzMin[2] + j * stepZ };
      pointsObject.push_back(point);
    }
  }

  for (int i = 0; i < stepsX; ++i)
  {
    double x = xyzMin[0] + i * stepX;
    for (int j = 0; j < stepsZ; ++j)
    {
      std::array<double, 3> point = { x, xyzMax[1], xyzMin[2] + j * stepZ };
      pointsObject.push_back(point);
    }
  }
  // RLOG(0, "Number of points: %ld", pointsObject.size());
}


}   // namespace aff
