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

#include "ExampleLLMSim.h"

#include <ExampleFactory.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>

#include <thread>



namespace aff
{

RCS_REGISTER_EXAMPLE(ExampleLLMSim, "Actions", "LLM Simulator");


ExampleLLMSim::ExampleLLMSim() : ExampleActionsECS(0, NULL)
{
}

ExampleLLMSim::ExampleLLMSim(int argc, char** argv) :
  ExampleActionsECS(argc, argv)
{
}

ExampleLLMSim::~ExampleLLMSim()
{
}

bool ExampleLLMSim::initGraphics()
{
  ExampleActionsECS::initGraphics();

  if (noViewer || valgrind)
  {
    RLOG(0, "Running without graphics");
    return true;
  }

  // Check if there is a body named 'initial_camera_view'.
  double q_cam[6];
  VecNd_set6(q_cam, -2.726404, -2.515165, 3.447799,   -0.390124, 0.543041, 0.795294);

  const RcsBody* camera_body = RcsGraph_getBodyByName(graphC->getGraph(), "default_camera_view");
  if (camera_body)
  {
    RLOG(1, "Setting initial view based on body 'initial_camera_view'.");
    HTr_to6DVector(q_cam, &camera_body->A_BI);
  }

  viewer->setCameraTransform(q_cam[0], q_cam[1], q_cam[2], q_cam[3], q_cam[4], q_cam[5]);

  if (!hwc.empty())
  {
    entity.publish("BackgroundColor", std::string("PEWTER"));
  }

  return true;
}

bool ExampleLLMSim::initAlgo()
{
  ExampleActionsECS::initAlgo();

  entity.subscribe<bool, double, std::string>("ActionResult", &ExampleLLMSim::onActionResult, this);
  entity.subscribe("TextCommand", &ExampleLLMSim::onTextCommand, this);

  return true;
}

void ExampleLLMSim::onActionResult(bool success, double quality, std::string resMsg)
{
  if (verbose)
  {
    RMSG_CPP(resMsg);
  }
  entity.publish("SetTextLine", resMsg, 1);

  lastResultMsg = resMsg;

  // This needs to be done independent of failure or success
  if (!actionStack.empty())
  {
    addToCompletedActionStack(actionStack[0], resMsg);

    if (verbose)
    {
      printCompletedActionStack();
    }

    actionStack.erase(actionStack.begin());
  }

  if (!success)
  {
    numFailedActions++;

    if (unittest)
    {
      if (!actionStack.empty())
      {
        // \todo: Check why this works
        //RPAUSE_MSG("Calling reset");
        entity.publish("TextCommand", std::string("reset"));
        actionStack.insert(actionStack.begin(), "reset");
      }
    }
    else
    {
      actionStack.clear();
    }
    RLOG_CPP(0, "Clearing action stack, number of failed actions: " << numFailedActions);
  }

  // After the last action command of a sequence has been finished (the
  // trajectory has ended), or we face a failure, we "unfreeze" the perception
  // and accept updates from the perceived scene.
  if (actionStack.empty())
  {
    RLOG(0, "ActionStack is empty, unfreezing perception");
    entity.publish("FreezePerception", false);
    processingAction = false;
    actionC->setFinalPoseRunning(false);
  }

  if ((actionStack.size()==1) && STRNEQ(actionStack[0].c_str(), "pose", 4))
  {
    actionC->setFinalPoseRunning(true);
  }

  // In valgrind mode, we only perform one action, since this might run with valgrind
  if (valgrind || (unittest && actionStack.empty()))
  {
    ExampleBase::stop();
  }
  else
  {
    entity.publish("SendWebsocket", resMsg);
  }

  // HACK \todo(MG): Unify this
  failCount = numFailedActions;
  //processingAction = false;
}

// This only handles the "get_state" keyword
void ExampleLLMSim::onTextCommand(std::string text)
{
  if (verbose)
  {
    RMSG_CPP("ExampleLLMSim::onTextCommand RECEIVED: " << text);
  }

  if (text=="get_state")
  {
    std::thread feedbackThread([this]()
    {
      std::string fb = sceneQuery->getSceneState().dump();
      entity.publish("SendWebsocket", fb);
    });
    feedbackThread.detach();
  }
}

size_t ExampleLLMSim::getNumFailedActions() const
{
  return numFailedActions;
}


}   // namespace aff
