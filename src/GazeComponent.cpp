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



#define GAZEOBJECT_MAX_EXTENTS (1.0)   // Ignore everything with an extent larger than this

/*
This class computes the intersection of a gazing body (head) with a set of objects
that it should attend to. The gaze direction is the head's y-axis.
 */

namespace aff
{

GazeComponent::GazeComponent(EntityBase* parent, const std::string& gazingBody_, int dirIdx) :
  ComponentBase(parent), gazingBody(gazingBody_), id_gazeBody(-1), gazeDirectionIdx(dirIdx)
{
  subscribe("PostUpdateGraph", &GazeComponent::onPostUpdateGraph);
}

void GazeComponent::addSceneToAttend(const ActionScene& scene, const RcsGraph* graph)
{
  auto ntts = scene.getSceneEntities();

  for (const auto& ntt : ntts)
  {
    RLOG_CPP(1, "Adding " << ntt->name << " to GazeComponent");
    const RcsBody* bdy = RcsGraph_getBodyByName(graph, ntt->bdyName.c_str());
    RCHECK_MSG(bdy, "%s", ntt->bdyName.c_str());

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


  // This is kind of a saccade suppression. If the gaze point moves quickly, we
  // suppress the attention mechanism so that objects we just pass by gazing
  // are ignored. Otherwise, there will be spurious detections of objects to
  // attend that are just in the way
  double gazeVel = Vec3d_getLength(head->omega);


  // Now we go through all objects
  for (auto& o : objectsToAttend)
  {
    const RcsBody* obj = getBody(graph, o.bdyName, o.bdyId);
    double eye_obj[3];
    const double* objPos = obj->A_BI.org;

    double xyzMin[3], xyzMax[3], centroid[3];
    bool aabbValid = RcsGraph_computeBodyAABB(graph, obj->id, -1, xyzMin, xyzMax, NULL);
    if (aabbValid)
    {
      Vec3d_set(centroid, 0.5*(xyzMin[0]+xyzMax[0]), 0.5*(xyzMin[1]+xyzMax[1]), 0.5*(xyzMin[2]+xyzMax[2]));
      objPos = centroid;
    }

    Vec3d_sub(eye_obj, objPos, eyePos);
    o.gazeAngle = Vec3d_diffAngle(eye_obj, gazeDir);
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

  REXEC(2)
  {
    RLOG_CPP(1, objectsToAttend.size() << " gaze objects:");
    for (auto& o : objectsToAttend)
    {
      std::cout << o.name << ": " << (180.0/M_PI)*o.gazeAngle << std::endl;
    }
  }

  t_calc = Timer_getSystemTime() - t_calc;

  RLOG(1, "Took %.3f usec, gazeVel is %.3f", 1000.0 * t_calc, (180.0 / M_PI)* gazeVel);
}


}   // namespace aff
