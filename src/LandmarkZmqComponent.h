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

#ifndef AFF_LANDMARKZMQCOMPONENT_H
#define AFF_LANDMARKZMQCOMPONENT_H

#include "LandmarkBase.h"
#include "ComponentBase.h"

#include <thread>


namespace aff
{

class LandmarkZmqComponent : public ComponentBase, public LandmarkBase
{
public:

  /*! \brief The argument connection can also be a filename. In that case,
   *         the contents of the file will be parsed into a json string and
   *         passed to the class methods, instead of running a zmq interface.
   */
  LandmarkZmqComponent(EntityBase* parent,
                       std::string connection = "tcp://localhost:5555");

  virtual ~LandmarkZmqComponent();

private:

  /*! \brief Same as in LandmarkBase with the addition of the time being handled
   *         differently when data comes from a file.
   */
  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);

  void onToggleJsonLogging();
  void onEstimateCameraPose(int numFrames);
  void fromFileThreadFunc(const std::string& fileName);
  void zmqThreadFunc();
  void startZmqThread();
  void stopZmqThread();

  std::string connectionStr;
  bool threadRunning;
  bool threadFunctionCompleted;
  bool readDataFromFile;
  bool logging;

  int socketTimeoutInMsec;
  double frameRate;
  std::thread zmqThread;
};

} // namespace

#endif // AFF_LANDMARKZMQCOMPONENT_H
