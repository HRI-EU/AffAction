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

#ifndef ARUCOMULTICAMTRACKER_H
#define ARUCOMULTICAMTRACKER_H

#include "TrackerBase.h"

#include <Rcs_HTr.h>

#include <map>
#include <mutex>
#include <memory>
#include <functional>
#include <vector>




class ArucoMultiCamCalibrator;

class ArucoMultiCamTracker : public TrackerBase
{
public:

  ArucoMultiCamTracker();

  virtual ~ArucoMultiCamTracker();

  // Process aruco frames. Called from control loop (100Hz or so)
  void updateGraph(RcsGraph* graph);

  virtual std::string getRequestKeyword() const;

  void calibrate(size_t numFrames);
  void parse(const nlohmann::json& json, double time, const std::string& cameraFrame);
  void setCameraTransform(const HTr* A_CI);
  // void addCalibrationFinishedCallback(std::function<void(const HTr*)> callback);
  // std::string getBaseMarkerName() const;
  // bool isCalibrating() const;

  // This is a vector since there might be several entries of markers with the
  // same name, coming from different cameras. It is populated in the parse()
  // function, there is no knowledge about the graph.
  struct ArucoMarkerShapeData
  {
    std::string markerShapeName;
    HTr T_MC;   // Transformation from camera to marker
    double lastUpdateTime;
  };

  // This is a map, since the body names are unique. It is populated with the
  // updateGraph() function which computes the marker body transforms using the
  // transformation information from the graph.
  struct ArucoMarkerBodyData
  {
    std::vector<ArucoMarkerShapeData> shapes;
    std::vector<HTr> A_BP;
    int qIdx;
    double lastUpdateTime;
    bool isBaseMarker;
    std::vector<double> q_rbj;
  };

private:

  bool newArucoUpdate;
  std::mutex parseMtx;

  // This is to instantiate calibrators coming through the parsing. The key is
  // the camera name.
  std::map<std::string, std::unique_ptr<ArucoMultiCamCalibrator>> calibrators;

  // Key is camera name, values are the parsed aruco shape updates
  std::map<std::string,std::vector<ArucoMarkerShapeData>> arucoShapes;

  // Key is RcsBody name, values are the transformed aruco shapes
  std::map<std::string,ArucoMarkerBodyData> arucoBodies;

  static ArucoMarkerBodyData updateMarkerBodyTransform(const RcsGraph* graph,
                                                       const RcsBody* body,
                                                       const RcsBody* cameraBdy,
                                                       const std::vector<ArucoMarkerShapeData>& arucoShapes);

  static void updateMarkerBodyTransforms(const RcsGraph* graph,
                                         const RcsBody* cameraBdy,
                                         const std::vector<ArucoMarkerShapeData>& arucoShapes,
                                         std::map<std::string,ArucoMarkerBodyData>& arucoBodies);

};

#endif // ARUCOTRACKER_H
