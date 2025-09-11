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

#ifndef AFF_IMAGETRACKER_H
#define AFF_IMAGETRACKER_H

#include "TrackerBase.h"
#include "ComponentBase.h"
#include "VirtualCamera.h"
#include "ImageHelpers.h"

#include <mutex>
#include <memory>


namespace aff
{

class ImageTracker : public ComponentBase, public TrackerBase
{
public:

  ImageTracker(EntityBase* parent, const std::string& cameraName);

  virtual ~ImageTracker() = default;

  std::string getRequestKeyword() const override;

  void parse(const nlohmann::json& header, const nlohmann::json& data, double time) override;

  void update(ActionScene* scene, RcsGraph* graph) override;

  std::pair<int, std::string> getStampedImage(int frame_count=-1) const;

  PinholeCamera getCameraModel() const;

  std::vector<int> getGazeObjectBoundingBox() const;

  // return vector: int minX, int minY, int maxX, int maxY
  static std::vector<int> getObjectBoundingBox(const ActionScene* scene, const RcsGraph* graph,
                                               const std::string objName,
                                               const std::string& cameraName,
                                               const PinholeCamera& phCam);

  void enableDebugWindow(bool enable);

protected:

  void onSetGazeTarget(std::string bdyName);
  void updateDebugWindow(const std::vector<int>& bb) const;

  std::pair<int,std::string> stamped_image;
  mutable std::mutex imgMtx;
  double t_parse;
  std::string gazeTarget;
  PinholeCamera pinhole;
  bool showDebugWindow;
  std::vector<int> gaze_bb;
};





class VirtualImageTracker : public ImageTracker
{
public:

  VirtualImageTracker(EntityBase* parent,
                      const std::string& cameraName,
                      const std::string& cameraType="AzureKinect_WFOV",
                      int width=640,
                      int height=480);
  virtual ~VirtualImageTracker() = default;
  void parse(const nlohmann::json& header, const nlohmann::json& data, double time) override;
  std::string getRequestKeyword() const override;
  void update(ActionScene* scene, RcsGraph* graph) override;

protected:

  std::string cameraType;
  int capture_count;
  std::unique_ptr<VirtualCamera> vCamPtr;
};


}   // namespace



#endif // AFF_IMAGETRACKER_H
