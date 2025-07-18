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

#include "AzureBodyTrackingComponent.h"

#include <Rcs_macros.h>


namespace aff
{

#if defined(HAVE_AZURE_TRACKER)

AzureBodyTrackingComponent::AzureBodyTrackingComponent(EntityBase* parent) :
  ComponentBase(parent), AzureBodyTracker()
{
  subscribe("Start", &AzureBodyTrackingComponent::onStart);
  subscribe("Stop", &AzureBodyTracker::stop);
  subscribe("EstimateCameraPose", &AzureBodyTracker::estimateCameraPose);
}

AzureBodyTrackingComponent::~AzureBodyTrackingComponent()
{
}

void AzureBodyTrackingComponent::onStart()
{
  start();
  getEntity()->publish("SetTextLine", std::string("Azure Kinect not yet up ..."), 1);
}

void AzureBodyTrackingComponent::cameraCalibrationStartedCallback()
{
  getEntity()->publish("RenderCommand", std::string("BackgroundColor"), std::string("RED"));
}

void AzureBodyTrackingComponent::cameraCalibrationFinishedCallback()
{
  getEntity()->publish("RenderCommand", std::string("BackgroundColor"), std::string(""));
}

void AzureBodyTrackingComponent::cameraTransformUpdatedCallback(HTr A_CI)
{
  getEntity()->publish("SetAzureKinectTransform", A_CI);
}

void AzureBodyTrackingComponent::posesUpdatedCallback(const std::map<int, std::vector<HTr>>& poses)
{
  if (poses.empty())
  {
    return;
  }

  //getEntity()->publish("Retarget", poses[0]);

  // for (auto it = poses.begin(); it != poses.end(); it++)
  {
    getEntity()->publish("RetargetPose", poses);
  }
  //for (size_t i=0; i<poses.size(); ++i)
  //{
  //  getEntity()->publish("RetargetPose", (int)i, poses[i]);
  //}
}

void AzureBodyTrackingComponent::updateDebugTextCallback(std::string textMsg)
{
  getEntity()->publish("SetTextLine", textMsg, 1);
}

#else

AzureBodyTrackingComponent::AzureBodyTrackingComponent(EntityBase* parent)
{
  RMSG("Kinect Azure libraries are not linked (k4a, k4abt)");
}

AzureBodyTrackingComponent::~AzureBodyTrackingComponent()
{
}


#endif

}   // namespace aff
