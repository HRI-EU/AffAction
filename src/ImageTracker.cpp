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

#include "ImageTracker.h"
#include "SceneHelpers.h"

#include <Rcs_macros.h>


namespace aff
{


ImageTracker::ImageTracker(const std::string& cameraName) : TrackerBase(cameraName), t_parse(0.0)
{
}

std::string ImageTracker::getRequestKeyword() const
{
  return "image";
}

void ImageTracker::parse(const nlohmann::json& header, const nlohmann::json& data, double time)
{
  const double t_prev = t_parse;
  t_parse = getWallclockTime();

  try
  {
    // Extract sequence number safely
    if (!header.contains("seq") || !header["seq"].is_number_integer())
    {
      RLOG_CPP(1, "Warning: 'seq' not found or not an integer in header.");
      return;
    }

    int stamp = header.at("seq").get<int>();

    // Serialize image JSON
    std::string image_str = data.dump();

    {
      std::lock_guard<std::mutex> lock(imgMtx);
      this->stamped_image = std::make_pair(stamp, image_str);
    }

    RLOG_CPP(1, "Received: count=" << stamped_image.first << " after " << 1.0e3 * (t_parse - t_prev) << " msec");// << " image=" << stamped_image.second);
  }
  catch (const nlohmann::json::exception& e)
  {
    RLOG_CPP(1, "JSON processing error: " << e.what());
  }
  catch (const std::exception& e)
  {
    RLOG_CPP(1, "General error during JSON handling: " << e.what());
  }

}

void ImageTracker::update(ActionScene* scene, RcsGraph* graph)
{
}

std::pair<int, std::string> ImageTracker::getStampedImage(int frame_count)
{
  std::lock_guard<std::mutex> lock(imgMtx);
  if ((frame_count == -1) || (stamped_image.first < frame_count))
  {
    return this->stamped_image;
  }

  return std::make_pair(stamped_image.first, std::string());
}

}   // namespace

