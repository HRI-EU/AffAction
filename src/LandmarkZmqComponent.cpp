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


#include "LandmarkZmqComponent.h"
#include "ArucoTracker.h"

#include <ComponentBase.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_utils.h>

#include <zmq.hpp>

#include <fstream>



static void appendToFile(const std::string& fileName, const std::string& str)
{
  std::ofstream of;

  // opening file using ofstream
  of.open(fileName, std::ios::app);
  if (!of)
  {
    std::cout << "No such file found";
  }
  else
  {
    of << str;
    std::cout << "Data appended successfully\n";
    of.close();
    // string word;

    // // opening file using fstream
    //std::fstream f;
    // f.open("Geeks for Geeks.txt");
    // while (f >> word)
    // {
    //   cout << word << " ";
    // }
    // f.close();
  }

}

static void sendRequest(zmq::socket_t& socket, const nlohmann::json& request)
{
  std::string json_str = request.dump();
  zmq::message_t query(json_str.length());
  memcpy(query.data(), json_str.c_str(), json_str.size());
#if ZMQ_VERSION <= ZMQ_MAKE_VERSION(4, 3, 1)
  socket.send(query);
#else
  socket.send(query, zmq::send_flags::none);
#endif
}

static std::string receiveReply(zmq::socket_t& socket)
{
  zmq::message_t reply;
#if ZMQ_VERSION <= ZMQ_MAKE_VERSION(4, 3, 1)
  socket.recv(&reply);
#else
  (void) socket.recv(reply);
#endif

  std::string reply_str;
  reply_str = std::string(static_cast<char*>(reply.data()), reply.size());
  NLOG_CPP(1, reply_str);

  return reply_str;
}


namespace aff
{


LandmarkZmqComponent::LandmarkZmqComponent(EntityBase* parent, std::string connection):
  ComponentBase(parent), LandmarkBase(),
  connectionStr(connection), threadRunning(false), threadFunctionCompleted(false),
  readDataFromFile(false), socketTimeoutInMsec(3000), frameRate(0.0), logging(false)
{
  readDataFromFile = File_exists(connection.c_str());

  subscribe("Start", &LandmarkZmqComponent::startZmqThread);
  subscribe("Stop", &LandmarkZmqComponent::stopZmqThread);
  subscribe("PostUpdateGraph", &LandmarkZmqComponent::onPostUpdateGraph);
  subscribe("FreezePerception", &LandmarkBase::onFreezePerception);
  subscribe("EstimateCameraPose", &LandmarkZmqComponent::onEstimateCameraPose);
  subscribe("ToggleJsonLogging", &LandmarkZmqComponent::onToggleJsonLogging);
  subscribe("EnableDebugGraphics", &LandmarkBase::enableDebugGraphics);
}

LandmarkZmqComponent::~LandmarkZmqComponent()
{
  if (threadRunning)
  {
    RLOG(0, "Thread still running in destructor - please stop it before.");
  }
}

std::string LandmarkZmqComponent::getName() const
{
  std::string res = ComponentBase::getName() + ": ";

  for (const auto& tracker : trackers)
  {
    res += tracker->getRequestKeyword();
    res += " ";
  }

  return res;
}

void LandmarkZmqComponent::onToggleJsonLogging()
{
  logging = !logging;
}

// Same as in LandmarkBase with the addition of the time being handled
// differently when data comes from a file.
void LandmarkZmqComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  const double wallClockTime = readDataFromFile ? 0.0 : TrackerBase::getWallclockTime();
  for (const auto& tracker : trackers)
  {
    tracker->setCurrentTime(wallClockTime);
  }

  updateGraph(desired);
}

void LandmarkZmqComponent::onEstimateCameraPose(int numFrames)
{
  for (auto& t : trackers)
  {
    ArucoTracker* at = dynamic_cast<ArucoTracker*>(t.get());

    if (at)
    {
      RLOG(0, "Calibrating Aruco camera");
      at->calibrate(numFrames);
    }
  }
}

void LandmarkZmqComponent::fromFileThreadFunc(const std::string& fileName)
{
  threadFunctionCompleted = false;
  setSyncInputWithWallclock(true);

  RLOG_CPP(0, "Reading data from " << fileName);
  std::ifstream ifs(fileName);
  RCHECK_MSG(ifs.good(), "Can't open '%s'", fileName.c_str());

  RLOG_CPP(0, "Parsing jsons");
  std::vector<nlohmann::json> jsons;

  try
  {
    while (!ifs.eof())
    {
      nlohmann::json j;
      ifs >> j;
      // (CP) disabled for other debugging
      //RLOG_CPP(1, "json: '" << j << "'");
      j["header"]["timestamp"] = 0.0;
      jsons.push_back(j);
    }
  }
  catch (nlohmann::json::parse_error& ex)
  {
    std::cerr << "parse error at byte " << ex.byte << std::endl;
  }

  RLOG_CPP(0, "json has " << jsons.size() << " entries");

  while (threadRunning)
  {
    for (auto& json : jsons)
    {
      if (!frozen)
      {
        setJsonInput(json);
      }
      Timer_waitDT(0.2);

      // Inner loop check so that quitting is a bit more responsive
      if (!threadRunning)
      {
        break;
      }
    }
  }

  threadFunctionCompleted = true;
}

void LandmarkZmqComponent::zmqThreadFunc()
{
  RLOG(5, "zmqThreadFunc()");
  threadFunctionCompleted = false;
  zmq::context_t context;
  zmq::socket_t socket(context, ZMQ_REQ);

  RLOG(5, "Connecting to tcp://localhost:5555");
  socket.connect("tcp://localhost:5555");

  // set receive timeout to 3 seconds
  int timeout_ms = this->socketTimeoutInMsec;

  RLOG(5, "Setting socket timeout");
#if ZMQ_VERSION <= ZMQ_MAKE_VERSION(4, 3, 2)
  socket.setsockopt(ZMQ_RCVTIMEO, &timeout_ms, sizeof(int));
#else
  socket.set(zmq::sockopt::rcvtimeo, timeout_ms);
#endif
  bool timedOut = false;



  RLOG(5, "Starting thread loop");
  while (this->threadRunning && !timedOut)
  {
    double t_mp = Timer_getSystemTime();

    nlohmann::json request;

    for (const auto& tracker : trackers)
    {
      request[tracker->getRequestKeyword()] = true;
    }

    RLOG_CPP(5, "Sending request: " << request.dump());
    sendRequest(socket, request);
    RLOG_CPP(5, "Waiting for reply");
    std::string reply_str = receiveReply(socket);
    RLOG_CPP(5, "Received reply");
    RLOG_CPP(6, "Received reply: " << reply_str);

    if (logging)
    {
      appendToFile("python_landmark_input.json", reply_str);
    }

    // We end up here if there is no reply within the timeout. In this case, we return.
    if (reply_str.empty())
    {
      RLOG(0, "Didn't receive message within timeout - quitting thread function");
      timedOut = true;
      socket.disconnect("tcp://localhost:5555");
      socket.close();
      threadFunctionCompleted = true;
      return;
    }

    if (!frozen)
    {
      RLOG_CPP(5, "Parsing reply json");
      nlohmann::json json = nlohmann::json::parse(reply_str);
      RLOG_CPP(5, "setJsonInput");
      setJsonInput(json);
      RLOG_CPP(5, "done setJsonInput");
    }

    // Timing statistics
    t_mp = Timer_getSystemTime() - t_mp;
    frameRate = (frameRate == 0.0) ? 1.0 / t_mp : 0.99 * frameRate + 0.01 * (1.0 / t_mp);

    RLOG(5, "Framerate: %.2f Hz", this->frameRate);
  }

  // Quit the python server
#if defined (_MSC_VER)
  //if (quitPythonServerOnExit)
  //{
  //  nlohmann::json request;
  //  request["quit"] = true;
  //  sendRequest(socket, request);
  //}
#endif

  threadFunctionCompleted = true;
  RLOG(1, "Quitting thread function");
}

void LandmarkZmqComponent::startZmqThread()
{
  if (threadRunning)
  {
    RLOG(1, "Thread already running");
    return;
  }

  RLOG(0, "startZmqThread()");

  threadRunning = true;

  if (readDataFromFile)
  {
    zmqThread = std::thread(&LandmarkZmqComponent::fromFileThreadFunc, this, connectionStr);
  }
  else
  {
    zmqThread = std::thread(&LandmarkZmqComponent::zmqThreadFunc, this);
  }

  // Ideally, we should join it in the onStop() function. For some reasons,
  // this does not work on all platforms, maybe due to some dangling zmq
  // class that inhibits joining the thread somehow. This only is the case
  // once we run into a receive timeout. We therefore detach the thread here.
  zmqThread.detach();
}

void LandmarkZmqComponent::stopZmqThread()
{
  if (!threadRunning)
  {
    RLOG(0, "Thread already stopped");
    return;
  }

  RLOG(0, "Trying to stop thread");
  threadRunning = false;

  // See startZmqThread() why we don't join the thread here as one would expect.
  // mpThread.join();
  // RLOG(0, "Thread joined");
  while (!threadFunctionCompleted)
  {
    Timer_waitDT(0.1);
  }

  RLOG(0, "onStop() completed");
}




} // namespace
