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

#ifndef AFF_FACEGESTURECOMPONENT_H
#define AFF_FACEGESTURECOMPONENT_H

#include <ComponentBase.h>
#include <Rcs_graph.h>


namespace aff
{

/*! \brief Interprets face mesh and emits events on certain face gestures
 */
class FaceGestureComponent: public ComponentBase
{
public:

  /*! \brief Constructor depends only on EntityBase.
   */
  FaceGestureComponent(EntityBase* parent,
                       const std::string& faceName,
                       const RcsMeshData* mesh);

  /*! \brief Virtual destructor to allow proper inheritance.
   */
  virtual ~FaceGestureComponent();


protected:

  void onUpdateGraph(RcsGraph* graph);

  double getLipDistance(const RcsMeshData* faceMesh) const;

  double getLeftLidDistance(const RcsMeshData* faceMesh) const;

  double getRightLidDistance(const RcsMeshData* faceMesh) const;


  const double* leftUpperLidCoords(const RcsMeshData* faceMesh) const;

  const double* leftLowerLidCoords(const RcsMeshData* faceMesh) const;

  const double* rightUpperLidCoords(const RcsMeshData* faceMesh) const;

  const double* rightLowerLidCoords(const RcsMeshData* faceMesh) const;

  const double* upperLipCoords(const RcsMeshData* faceMesh) const;

  const double* lowerLipCoords(const RcsMeshData* faceMesh) const;

  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* curent);

  void estimateSmile(const RcsMeshData* faceMesh, double faceHeight, std::string& debugMsg);

  void estimateNodding(double tiltAngle, size_t windowSize, std::string& debugMsg);

  void estimateHeadshaking(double panAngle, size_t windowSize, std::string& debugMsg);

  void estimateEyeOpen(const RcsMeshData* faceMesh, double faceHeight, bool leftTrueRightFalse, std::string& debugMsg);

  void estimateMouthOpen(const RcsMeshData* faceMesh, double faceHeight, std::string& debugMsg);

  std::string faceName;
  const RcsMeshData* faceMesh;
  bool smiling;
  bool mouthOpen;
  bool rightEyeOpen;
  bool leftEyeOpen;
  bool nodding;
  bool headShaking;
  std::vector<double> pan, tilt, dPan;
};

}   // namespace aff



#endif   // AFF_FACEGESTURECOMPONENT_H
