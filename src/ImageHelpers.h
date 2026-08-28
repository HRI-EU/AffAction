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

#ifndef AFF_IMAGETHELPERS_H
#define AFF_IMAGETHELPERS_H

#include "json.hpp"

#include <QString>
#include <QImage>

#include <string>



namespace aff
{

class PinholeCamera
{
public:
  PinholeCamera();

  nlohmann::json toJson() const;

  int width, height;      // Pixel resolution
  double fx, fy, cx, cy;  // Camera matrix
  double skew;            // Camera matrix entry (0, 1)
  double k1, k2;          // Radial distortion
  double p1, p2;          // Tangential distortion
  double k3, k4, k5, k6;  // Higher-order radial distortion
};

QImage decodeBase64JpegToQImage(const QString& base64String);
std::string rgbToJpegBase64(const uint8_t* rgb, int width, int height, int quality=90);          // 0-100

void showFrame(const QImage& img);

bool extract_intrinsics(const nlohmann::json& data,
                        PinholeCamera& cam,
                        std::string& err);

bool computeCameraGazeDirection(const PinholeCamera& cam,
                                int x, int y,
                                double gazeDir[3],
                        std::string& err);


}   // namespace



#endif // AFF_IMAGETHELPERS_H
