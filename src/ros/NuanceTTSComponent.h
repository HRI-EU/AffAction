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

#ifndef AFF_NUANCETTSCOMPONENT_H
#define AFF_NUANCETTSCOMPONENT_H

#include "ComponentBase.h"

#if defined (USE_ROS)
#include <ros/ros.h>
#endif



namespace aff
{

class NuanceTTSComponent : public ComponentBase
{
public:

  NuanceTTSComponent(EntityBase* parent);
  virtual ~NuanceTTSComponent();

private:

#if defined (USE_ROS)
  void onStart();
  void onStop();
  void speakThreadFunc(std::string text);
  void onSpeak(std::string text);

  std::unique_ptr<ros::NodeHandle> nh;
  ros::ServiceClient ttsClient;

  // Avoid copying this class
  NuanceTTSComponent(const NuanceTTSComponent&);
  NuanceTTSComponent& operator=(const NuanceTTSComponent&);
#endif
};

}

#endif   // AFF_NUANCETTSCOMPONENT_H
