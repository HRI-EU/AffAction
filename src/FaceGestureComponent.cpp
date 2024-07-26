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

#include "FaceGestureComponent.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>

static const size_t upperLeftLidIdx  = 3 * 386;
static const size_t lowerLeftLidIdx  = 3 * 374;
static const size_t upperRightLidIdx = 3 * 159;
static const size_t lowerRightLidIdx = 3 * 145;
static const size_t upperLipIdx      = 3 * 13;
static const size_t lowerLipIdx      = 3 * 14;
static const size_t rightMouthCorner = 3 * 61;
static const size_t leftMouthCorner  = 3 * 291;

static const size_t betweenEyes      = 3 * 6;
static const size_t foreheadTop      = 3 * 10;
static const size_t chinCenter       = 3 * 152;

static int isNodding(std::vector<double> angle, double dt,
                     double minAmplitude)
{
  if (angle.empty())
  {
    return -1;
  }

  // Initial direction
  double dirSgn = Math_dsign(angle[1] - angle[0]);
  double lastTurn = angle[0];
  int nTurns = 0;

  RLOG(5, "***********************");
  for (size_t i=1; i<angle.size(); ++i)
  {
    const double newSgn = Math_dsign(angle[i] - angle[i-1]);

    if (newSgn != dirSgn)   // Turn detected
    {
      dirSgn = newSgn;
      if (fabs(lastTurn-angle[i])>minAmplitude)
      {
        RLOG(5, "[%zu]: Detected turn at %.1f, last one: %.1f", i, RCS_RAD2DEG(angle[i]), RCS_RAD2DEG(lastTurn));
        nTurns++;
        lastTurn = angle[i];
      }
    }
  }

  return nTurns;
}

namespace aff
{

FaceGestureComponent::FaceGestureComponent(EntityBase* parent,
                                           const std::string& faceName_,
                                           const RcsMeshData* mesh) :
  ComponentBase(parent), faceName(faceName_), faceMesh(mesh), smiling(false), mouthOpen(false),
  rightEyeOpen(false), leftEyeOpen(false), nodding(false), headShaking(false)
{
  subscribe("PostUpdateGraph", &FaceGestureComponent::onPostUpdateGraph);

  // Number of vertices: Face only: 468, face + iris: 478
  RCHECK_MSG(faceMesh->nVertices==468 || faceMesh->nVertices == 478, "faceMesh->nVertices=%d", faceMesh->nVertices);
}

FaceGestureComponent::~FaceGestureComponent()
{
}


void FaceGestureComponent::onUpdateGraph(RcsGraph* graph)
{
  RcsBody* face = RcsGraph_getBodyByName(graph, faceName.c_str());
  RCHECK(face);

  const double faceHeight = Vec3d_distance(&faceMesh->vertices[foreheadTop],
                                           &faceMesh->vertices[chinCenter]);
  std::string textLine;


  // Smile detection
  estimateSmile(faceMesh, faceHeight, textLine);

  // Mouth open / close estimations
  estimateMouthOpen(faceMesh, faceHeight, textLine);

  // Get RPY angles from face rotation matrix
  double ea[3];
  Mat3d_toEulerAngles(ea, face->A_BI.rot);
  ea[2] = Math_fmodAngle(ea[2]+M_PI);

  // Head shake estimation
  const size_t windowSize = 150*3;
  estimateNodding(ea[1], windowSize, textLine);
  estimateHeadshaking(ea[2], windowSize, textLine);

  // Eyes open / close estimations
  estimateEyeOpen(faceMesh, faceHeight, true, textLine);
  estimateEyeOpen(faceMesh, faceHeight, false, textLine);

  // Head orientation RPY
  char faceOrientation[256];
  snprintf(faceOrientation, 256, "\nRPY[deg]: %.0f %.0f %.0f",
           RCS_RAD2DEG(ea[0]), RCS_RAD2DEG(ea[1]), RCS_RAD2DEG(ea[2]));
  textLine += std::string(faceOrientation);

  // Show debug text in HUD
  getEntity()->publish("SetTextLine", textLine, 3);
}

double FaceGestureComponent::getLipDistance(const RcsMeshData* faceMesh) const
{
  return Vec3d_distance(upperLipCoords(faceMesh),
                        lowerLipCoords(faceMesh));
}

double FaceGestureComponent::getLeftLidDistance(const RcsMeshData* faceMesh) const
{
  return Vec3d_distance(leftUpperLidCoords(faceMesh),
                        leftLowerLidCoords(faceMesh));
}

double FaceGestureComponent::getRightLidDistance(const RcsMeshData* faceMesh) const
{
  return Vec3d_distance(rightUpperLidCoords(faceMesh),
                        rightLowerLidCoords(faceMesh));
}


const double* FaceGestureComponent::leftUpperLidCoords(const RcsMeshData* faceMesh) const
{
  return &faceMesh->vertices[upperLeftLidIdx];
}

const double* FaceGestureComponent::leftLowerLidCoords(const RcsMeshData* faceMesh) const
{
  return &faceMesh->vertices[lowerLeftLidIdx];
}

const double* FaceGestureComponent::rightUpperLidCoords(const RcsMeshData* faceMesh) const
{
  return &faceMesh->vertices[upperRightLidIdx];
}

const double* FaceGestureComponent::rightLowerLidCoords(const RcsMeshData* faceMesh) const
{
  return &faceMesh->vertices[lowerRightLidIdx];
}

const double* FaceGestureComponent::upperLipCoords(const RcsMeshData* faceMesh) const
{
  return &faceMesh->vertices[upperLipIdx];
}

const double* FaceGestureComponent::lowerLipCoords(const RcsMeshData* faceMesh) const
{
  return &faceMesh->vertices[lowerLipIdx];
}


void FaceGestureComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* curent)
{
  onUpdateGraph(desired);
}

void FaceGestureComponent::estimateSmile(const RcsMeshData* faceMesh, double faceHeight, std::string& debugMsg)
{
  const double smileThreshold = 0.03;
  const double lipCloseThreshold = 0.01;
  const double* mcr = &faceMesh->vertices[rightMouthCorner];
  const double* mcl = &faceMesh->vertices[leftMouthCorner];
  const double* ul = upperLipCoords(faceMesh);
  const double* ll = lowerLipCoords(faceMesh);

  const double smile = (0.5*(mcr[2]+mcl[2]) - 0.5*(ul[2]+ll[2]))/faceHeight;
  RLOG(5, "Smile=%f, lip distance=%f   height: %f", smile, getLipDistance(faceMesh), faceHeight);

  if ((!smiling) && (smile>=smileThreshold) && (getLipDistance(faceMesh)<lipCloseThreshold))
  {
    smiling = true;
    RLOG(5, "Start smiling");
    getEntity()->publish("Speak", std::string("Smiling"));

  }
  else if ((smiling) && ((smile<0.6*smileThreshold) ||
                         (getLipDistance(faceMesh)>=lipCloseThreshold)))
  {
    smiling = false;
    RLOG(5, "Stop smiling");
    getEntity()->publish("Speak", std::string("Not smiling"));
  }

  debugMsg += (smiling ? "Smiling       " : "Not smiling   ");
}

void FaceGestureComponent::estimateNodding(double tiltAngle, size_t windowSize, std::string& debugMsg)
{
  int nods = -1;

  tilt.push_back(tiltAngle);
  if (tilt.size() > windowSize)
  {
    tilt.erase(tilt.begin());
  }

  if (tilt.size() == windowSize)
  {
    nods = isNodding(tilt, getEntity()->getDt(),
                     RCS_DEG2RAD(5.0));
  }

  if (!nodding && (nods > 3))
  {
    nodding = true;
    getEntity()->publish("Speak", std::string("Nodding"));
  }

  if (nodding && (nods == 0))
  {
    nodding = false;
    getEntity()->publish("Speak", std::string("Not nodding"));
  }

  if (nods>3)
  {
    debugMsg += "\n    NODDING ";
  }
  else
  {
    debugMsg += "\nNOT NODDING ";
  }
  debugMsg += "   Nods: ";
  debugMsg += std::to_string(nods);
}

void FaceGestureComponent::estimateHeadshaking(double panAngle, size_t windowSize, std::string& debugMsg)
{
  int headShakes = -1;

  pan.push_back(panAngle);
  if (pan.size() > windowSize)
  {
    pan.erase(pan.begin());
  }

  if (pan.size() == windowSize)
  {
    headShakes = isNodding(pan, getEntity()->getDt(),
                           RCS_DEG2RAD(15.0));
  }

  if (!headShaking && (headShakes > 3))
  {
    headShaking = true;
    getEntity()->publish("Speak", std::string("Headshaking"));
  }

  if (headShaking && (headShakes == 0))
  {
    headShaking = false;
    getEntity()->publish("Speak", std::string("Not headshaking"));
  }

  if (headShakes > 3)
  {
    debugMsg += "\n    HEADSHAKING ";
  }
  else
  {
    debugMsg += "\nNOT HEADSHAKING ";
  }
  debugMsg += "   Headshakes: ";
  debugMsg += std::to_string(headShakes);
}

void FaceGestureComponent::estimateEyeOpen(const RcsMeshData* faceMesh, double faceHeight, bool leftEye, std::string& debugMsg)
{
  const double eyeOpenThreshold = 0.035;

  if (leftEye)
  {
    const double eyeDistance = getLeftLidDistance(faceMesh) / faceHeight;
    RLOG(5, "left eyeDistance: %f", eyeDistance);

    if ((!leftEyeOpen) && (eyeDistance >= eyeOpenThreshold))
    {
      leftEyeOpen = true;
      RLOG(5, "Left eye opened");
    }
    else if ((leftEyeOpen) && (eyeDistance < 0.6 * eyeOpenThreshold))
    {
      leftEyeOpen = false;
      RLOG(5, "Left eye closed");
    }

    debugMsg += (leftEyeOpen ? "\nLeft eye open     " : "\nLeft eye closed   ");
  }
  else
  {
    const double eyeDistance = getRightLidDistance(faceMesh) / faceHeight;
    RLOG(5, "right eyeDistance: %f", eyeDistance);

    if ((!rightEyeOpen) && (eyeDistance >= eyeOpenThreshold))
    {
      rightEyeOpen = true;
      RLOG(5, "Right eye opened");
    }
    else if ((rightEyeOpen) && (eyeDistance < 0.6 * eyeOpenThreshold))
    {
      rightEyeOpen = false;
      RLOG(5, "Right eye closed");
    }

    debugMsg += (rightEyeOpen ? "\nRight eye open     " : "\nRight eye closed   ");
  }

}

void FaceGestureComponent::estimateMouthOpen(const RcsMeshData* faceMesh, double faceHeight, std::string& debugMsg)
{
  const double mouthOpenthreshold = 0.1;
  const double lipDistance = getLipDistance(faceMesh) / faceHeight;
  RLOG(5, "lipDistance: %f", lipDistance);

  if ((!mouthOpen) && (lipDistance >= mouthOpenthreshold))
  {
    mouthOpen = true;
    RLOG(5, "Mouth opened");
  }
  else if ((mouthOpen) && (lipDistance < 0.6 * mouthOpenthreshold))
  {
    mouthOpen = false;
    RLOG(5, "Mouth closed");
  }

  debugMsg += (mouthOpen ? "\nMouth opened   " : "\nMouth closed   ");
}

}   // namespace aff
