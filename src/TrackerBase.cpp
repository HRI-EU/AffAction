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

#include "TrackerBase.h"

#include <Rcs_macros.h>
#include <Rcs_Mat3d.h>
#include <Rcs_typedef.h>

#include <chrono>


namespace aff
{

TrackerBase::TrackerBase(const std::string& cameraName) :
  currentTime(0.0), frozen(false), cameraNamedId(cameraName, -1)
{
}

void TrackerBase::setCurrentTime(double time)
{
  currentTime = time;
}

double TrackerBase::getCurrentTime() const
{
  return currentTime;
}

/*static*/ double TrackerBase::getWallclockTime()
{
  // Get the current time point
  auto currentTime = std::chrono::system_clock::now();

  // Convert the time point to a duration since the epoch
  std::chrono::duration<double> durationSinceEpoch = currentTime.time_since_epoch();

  // Convert the duration to seconds as a floating-point number
  double seconds = durationSinceEpoch.count();

  return seconds;
}

void TrackerBase::setFrozen(bool freeze)
{
  this->frozen = freeze;
}

bool TrackerBase::initDebugGraphics(Rcs::Viewer* viewer, const RcsGraph* graph)
{
  return false;
}

/*static*/ RcsBody* TrackerBase::getBody(const RcsGraph* graph, std::pair<std::string, int>& bdyIdPair)
{
  if ((bdyIdPair.second == -1) || (bdyIdPair.first != graph->bodies[bdyIdPair.second].name))
  {
    RcsBody* bdy = RcsGraph_getBodyByName(graph, bdyIdPair.first.c_str());
    bdyIdPair.second = bdy ? bdy->id : -1;
    return bdy;
  }

  return &graph->bodies[bdyIdPair.second];
}

HTr TrackerBase::getCameraTransform(const RcsGraph* graph) const
{
  RcsBody* cam = getBody(graph, this->cameraNamedId);
  HTr A_CI;
  HTr_copy(&A_CI, cam ? &cam->A_BI : HTr_identity());
  return A_CI;
}

std::string TrackerBase::getCameraName() const
{
  return cameraNamedId.first;
}


}   // namespace
