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

#ifndef ARUCOTRACKER_H
#define ARUCOTRACKER_H

#include "TrackerBase.h"

#include <Rcs_HTr.h>

#include <map>
#include <mutex>
#include <memory>
#include <functional>
#include <vector>



namespace aff
{

class ArucoCalibrator;

class ArucoTracker : public TrackerBase
{
public:

  ArucoTracker(const std::string& cameraBodyName, const std::string& baseMarkerBdyName);

  virtual ~ArucoTracker();

  // Process aruco frames. Called from control loop (100Hz or so)
  void updateGraph(RcsGraph* graph);

  virtual std::string getRequestKeyword() const;

  void calibrate(size_t numFrames);
  void parse(const nlohmann::json& json, double time, const std::string& cameraFrame);
  void setCameraTransform(const HTr* A_CI);
  void addCalibrationFinishedCallback(std::function<void(const HTr*)> callback);
  std::string getBaseMarkerName() const;
  std::string getCameraName() const;
  bool isCalibrating() const;

private:

  std::unique_ptr<ArucoCalibrator> calibration;
  std::map<std::string,std::vector<double>> arucoMap;
  HTr A_CI;         // Transform from world frame into camera frame
  bool newArucoUpdate;
  std::mutex arucoMapMtx;

  //  Tuple: 1. body name, 2. q-index, 3. vector with 6 joint values
  std::vector<std::tuple<std::string,std::vector<std::string>,int,std::vector<double>>> markers;

public:
  // This is better
  struct ArucoMarkerData
  {
    std::vector<std::string> markerShapes;
    int qIdx;
    std::vector<double> q_rbj;
    double lastUpdateTime;
  };
  std::map<std::string,ArucoMarkerData> arucoBodies;
};

}   // namespace

#endif // ARUCOTRACKER_H
