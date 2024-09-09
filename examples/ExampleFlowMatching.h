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

#ifndef AFF_EXAMPLEFLOWMATCHING_H
#define AFF_EXAMPLEFLOWMATCHING_H

#include "EntityBase.h"
#include "GraphComponent.h"
#include "GraphicsWindow.h"
#include "PhysicsComponent.h"
#include "VirtualCameraComponent.h"

#include <ExampleBase.h>



namespace aff
{

class ExampleFlowMatching : public Rcs::ExampleBase
{
public:

  ExampleFlowMatching();
  ExampleFlowMatching(int argc, char** argv);
  virtual ~ExampleFlowMatching();

  // From ExampleBase
  virtual bool initParameters();
  virtual bool parseArgs(Rcs::CmdLineParser* parser);
  virtual bool initAlgo();
  virtual bool initGraphics();
  virtual void run();
  virtual void step();

protected:

  EntityBase entity;

  std::string xmlFileName;
  std::string configDirectory;
  std::unique_ptr<GraphicsWindow> viewer;
  double dt, dt_max;
  size_t loopCount;

  std::unique_ptr<GraphComponent> graphC;
  std::unique_ptr<PhysicsComponent> physicsC;
  std::unique_ptr<VirtualCameraComponent> vcamC;


  // Subscribed callbacks
  void onQuit();

  ES::SubscriberCollectionDecay<RcsGraph*>* updateGraph;
  ES::SubscriberCollectionDecay<RcsGraph*>* computeKinematics;
  ES::SubscriberCollectionDecay<>* setRenderCommand;
};

}   // namespace aff

#endif   // AFF_EXAMPLEFLOWMATCHING_H
