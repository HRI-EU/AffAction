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

#include "HeadGesture.h"
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_body.h>

#include <unordered_set>






namespace aff
{
/******************************************************************************
 *
 *****************************************************************************/
HeadGesture::HeadGesture(const std::string& gestureName, double duration, std::vector<int> jntIds) :
  name(gestureName), t_gesture(-1.0), gestureDuration(duration), amplitude(RCS_DEG2RAD(6.0)),
  numTurns(3), panJointId(-1), tiltJointId(-1), jointIds(jntIds)
{
}

HeadGesture::~HeadGesture()
{
}

void HeadGesture::setAmplitude(double newAmplitude)
{
  amplitude = newAmplitude;
}

void HeadGesture::setNumTurns(int turns)
{
  numTurns = turns;
}

std::vector<double> HeadGesture::stepPrecise(const Rcs::ControllerBase* controller, MatNd* a_des, RcsGraph* targetGraph, double dt)
{
  static double panStart = 0.0;
  static double tiltStart = 0.0;

  if (t_gesture < 0.0)
  {
    return std::vector<double>();
  }
  else if (t_gesture >= gestureDuration)
  {
    t_gesture = -1.0;
    return std::vector<double>();
  }
  else if (t_gesture == 0.0)
  {
    controller->getTask("Pan")->computeX(&panStart);
    controller->getTask("Tilt")->computeX(&tiltStart);
    RLOG(0, "Pan Tilt start[deg]: %.2f %.3f",
         RCS_RAD2DEG(panStart), RCS_RAD2DEG(tiltStart));
  }

  std::vector<double> panTilt = computePanTilt(t_gesture);
  panTilt[0] += panStart;
  panTilt[1] += tiltStart;
  t_gesture += dt;

  return panTilt;
}

void HeadGesture::step(const RcsGraph* graph, RcsGraph* targetGraph, double dt)
{
  if (t_gesture < 0.0)
  {
    return;
  }
  else if (t_gesture >= gestureDuration)
  {
    t_gesture = -1.0;
    return;
  }

  std::vector<double> panTilt = computePanTilt(t_gesture);
  updateHeuristic(graph, targetGraph, panTilt[0], panTilt[1]);

  t_gesture += dt;
}

// Goes after IK step
void HeadGesture::updateHeuristic(const RcsGraph* graph, RcsGraph* targetGraph,
                                  double pan_gesture, double tilt_gesture)
{
  if (panJointId == -1)
  {
    const RcsJoint* pan = RcsGraph_getJointByName(graph, "ptu_pan_joint");
    RCHECK(pan);
    panJointId = pan->id;
  }

  if (tiltJointId == -1)
  {
    const RcsJoint* tilt = RcsGraph_getJointByName(graph, "ptu_tilt_joint");
    RCHECK(tilt);
    tiltJointId = tilt->id;
  }

  // Constrain gaze dof in passed graphs
  for (const auto& j : jointIds)
  {
    const unsigned int jidx = graph->joints[j].jointIndex;

    if (graph->joints[j].id == tiltJointId)
    {
      targetGraph->q->ele[jidx] += tilt_gesture;
    }
    // We assume that the tilt DOF has the same rotation direction as the EyeThz DOF.
    // We move this DOF in the opposite direction, so that the eye approximately
    // keeps the focus of the gaze target
    else if (STREQ(graph->joints[j].name, "LeftEyeThY") ||
             STREQ(graph->joints[j].name, "RightEyeThY"))
    {
      targetGraph->q->ele[graph->joints[j].jointIndex] -= tilt_gesture;
    }
    // This is more "cosmetic": We assume that the PupilThY DOF has the same rotation
    // axis as the EyeThY DOF, and remove the out-of-plane rotation that comes from the
    // above EyeThY compensation. This only affects the graphics display of the pupil,
    // and does not have any other effect.
    else if (STREQ(graph->joints[j].name, "LeftPupilThY") ||
             STREQ(graph->joints[j].name, "RightPupilThY"))
    {
      targetGraph->q->ele[graph->joints[j].jointIndex] += tilt_gesture;
    }
    else if (graph->joints[j].id == panJointId)
    {
      targetGraph->q->ele[jidx] += pan_gesture;
    }
    // See above, now pan directions
    else if (STREQ(graph->joints[j].name, "LeftEyeThZ") ||
             STREQ(graph->joints[j].name, "RightEyeThZ"))
    {
      targetGraph->q->ele[graph->joints[j].jointIndex] -= pan_gesture;
    }
    else if (STREQ(graph->joints[j].name, "LeftPupilThZ") ||
             STREQ(graph->joints[j].name, "RightPupilThZ"))
    {
      targetGraph->q->ele[graph->joints[j].jointIndex] += pan_gesture;
    }
  }

}

void HeadGesture::start()
{
  RLOG_CPP(0, "Starting gesture " << name);
  t_gesture = 0.0;
}

std::string HeadGesture::getName() const
{
  return name;
}

/******************************************************************************
 * gnuplot> plot 6*sin(2*pi*x), 12*pi*cos(2*pi*x)
 *****************************************************************************/
HeadNod::HeadNod(const std::string& gestureName, double duration, std::vector<int> jntIds) :
  HeadGesture(gestureName, duration, jntIds)
{
}

std::vector<double> HeadNod::computePanTilt(double t)
{
  const double vmax = RCS_DEG2RAD(40.0);
  const double phase = vmax/amplitude;
  gestureDuration = numTurns*2.0*M_PI/phase;

  std::vector<double> panTilt(2, 0.0);
  panTilt[1] = -amplitude * sin(phase*t);
  return panTilt;
}

/******************************************************************************
 *
 *****************************************************************************/
HeadShake::HeadShake(const std::string& gestureName, double duration, std::vector<int> jntIds) :
  HeadGesture(gestureName, duration, jntIds)
{
}

std::vector<double> HeadShake::computePanTilt(double t)
{
  const double vmax = RCS_DEG2RAD(40.0);
  const double phase = vmax/amplitude;
  gestureDuration = numTurns*2.0*M_PI/phase;

  std::vector<double> panTilt(2, 0.0);
  panTilt[0] = -amplitude * sin(phase*t);
  return panTilt;
}


}   // namespace aff
