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

#include "SceneHelpers.h"

#include <Rcs_macros.h>

#include <json.hpp>

#include <condition_variable>
#include <memory>
#include <atomic>
#include <chrono>
#include <thread>



namespace aff
{


/*******************************************************************************
 *
 ******************************************************************************/
double getWallclockTime()
{
  // Get the current time point
  auto currentTime = std::chrono::system_clock::now();

  // Convert the time point to a duration since the epoch
  std::chrono::duration<double> durationSinceEpoch = currentTime.time_since_epoch();

  // Convert the duration to seconds as a floating-point number
  double seconds = durationSinceEpoch.count();

  return seconds;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string recognize_faces(EntityBase& entity,
                            const std::string& boundingBox,
                            int n_iterations,
                            double timeout_in_seconds)
{
  double t_calc = getWallclockTime();
  auto sub = std::make_shared<ES::ScopedSubscription>();
  auto active = std::make_shared<std::atomic<bool>>(true);
  std::mutex mtx;
  std::condition_variable cv;
  int counter = 0;
  std::string faceName;

  auto callback = [n=n_iterations, active=active, &mtx, &cv, &counter, &faceName]
                  (std::string id, std::string data) mutable
  {
    if ((!*active) || (id != "face_recog"))
    {
      return;
    }

    nlohmann::json j;
    try
    {
      j = nlohmann::json::parse(data);
      if (!j.contains("data") || !j["data"].contains("face_recog") || !j["data"]["face_recog"].is_array())
      {
        throw std::runtime_error("JSON missing required ['data']['face_recog'] array.");
      }
    }
    catch (const std::exception& e)
    {
      RLOG_CPP(1, "Invalid JSON: " << e.what());
      return;
    }

    const auto& faces = j["data"]["face_recog"];
    RLOG_CPP(1, "Iteration " << counter+1 << ": number of recognized faces: " << faces.size());

    bool done = false;
    {
      std::lock_guard<std::mutex> lk(mtx);
      counter++;
      done = (counter >= n);
      if (!faces.empty())
      {
        faceName = faces[0]["recognized_face"];
      }
    }

    RLOG_CPP(1, "First recognized face: " << faceName);

    if (done)
    {
      *active = false;  // prevent further callback execution
      cv.notify_one();   // wake calling context
    }
  };

  entity.withProcessLock([&]()
  {
    *sub = entity.subscribe("ZmqDealerMessage", std::move(callback));
  });

  entity.publish("TriggerPerception", std::string("face_recog"), n_iterations+25, boundingBox);

  std::unique_lock<std::mutex> lk(mtx);
  RLOG(1, "cv.wait");

  bool success = cv.wait_for(lk, std::chrono::duration<double>(timeout_in_seconds), [&]()
  {
    return counter >= n_iterations;
  });

  *active = false;  // handle cv timeouts with this
  entity.withProcessLock([&]()
  {
    sub->unsubscribe();
  });

  t_calc = getWallclockTime() - t_calc;
  RLOG(0, "%s recognize_faces after %.3f sec%s",
       (success ? "SUCCESS" : "FAIL"), t_calc,
       (success ? "" : ": Timeout reached while waiting for face recognition."));

  return faceName;
}



/*******************************************************************************
 *
 ******************************************************************************/
bool track_facemesh(EntityBase& entity,
                    const std::string& boundingBox,
                    int n_iterations,
                    double timeout_in_seconds)
{
  auto sub = std::make_shared<ES::ScopedSubscription>();
  auto active = std::make_shared<std::atomic<bool>>(true);
  std::mutex mtx;
  std::condition_variable cv;
  int counter = 0;

  auto callback = [n=n_iterations, active=active, &mtx, &cv, &counter]
                  (std::string id, std::string data) mutable
  {
    if ((!*active) || (id!="mediapipe"))
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lk(mtx);
      counter++;
    }

    RLOG_CPP(1, "Received mediapipe reply: " << counter);

    if (counter >= n)
    {
      *active = false;  // prevent further callback execution
      cv.notify_one();  // wake calling context
    }
  };

  RLOG(1, "Subscribing ZmqDealerMessage");
  entity.withProcessLock([&]()
  {
    *sub = entity.subscribe("ZmqDealerMessage", std::move(callback));
  });

  RLOG_CPP(1, "Publishing TriggerPerception with bounding box " << boundingBox);
  entity.publish("TriggerPerception", std::string("mediapipe"), n_iterations, boundingBox);


  std::unique_lock<std::mutex> lk(mtx);
  RLOG(1, "cv.wait");
  bool success = cv.wait_for(lk, std::chrono::duration<double>(timeout_in_seconds), [&]()
  {
    return counter >= n_iterations;
  });
  RLOG(1, "done cv.wait");

  *active = false;  // ensure no more callbacks after return
  entity.withProcessLock([&]()
  {
    sub->unsubscribe();
  });

  RLOG(1, "%s track_facemesh%s",
       success ? "SUCCESS" : "FAIL",
       success ? "" : ": Timeout reached while waiting for face mesh.");

  return success;
}



/*******************************************************************************
 *
 ******************************************************************************/
bool track_agent_facemesh(EntityBase& entity,
                          const ActionScene* scene,
                          const std::string& agentName,
                          int n_iterations,
                          double timeout_in_seconds)
{
  double t_calc = getWallclockTime();
  const Agent* agent = nullptr;

  if (agentName.empty())
  {
    auto humanAgents = scene->getAgents<HumanAgent>();
    if (!humanAgents.empty())
    {
      agent = humanAgents[0];
    }
  }
  else
  {
    agent = scene->getAgent(agentName);
  }

  if (!agent)
  {
    RLOG_CPP(1, "Agent '" << agentName << "' not found in scene");
    return false;
  }

  auto humanAgent = dynamic_cast<const HumanAgent*>(agent);

  if (!humanAgent)
  {
    RLOG_CPP(1, "Agent '" << agentName << "' is not a human agent");
    return false;
  }

  for (size_t i = 0; i < n_iterations; ++i)
  {
    nlohmann::json bb_json;

    bb_json["bounding_box"] =
    {
      { "left",   humanAgent->bb_head[0]},
      { "top",    humanAgent->bb_head[1]},
      { "right",  humanAgent->bb_head[2]},
      { "bottom", humanAgent->bb_head[3]}
    };

    RLOG_CPP(1, "track_agent_facemesh iteration " << i
             << " with bounding box " << bb_json.dump());
    REXEC(0)
    {
      std::cout << ".";
    }

    // In the last iteration, we set the repetitions to continusous update (-2), so
    // that the face is tracked without bounding box updates.
    int n_times = (i==n_iterations-1) ? -2 : 1;

    bool success = track_facemesh(entity, bb_json.dump(), n_times, timeout_in_seconds);
    if (!success)
    {
      RLOG(1, "Failed - returning");
      return false;
    }
    RLOG(1, "Success - continuing");
  }

  t_calc = getWallclockTime() - t_calc;

  REXEC(0)
  {
    std::cout << std::endl;
  }

  RLOG(0, "Took %.2f sec (is %.2f fps)", t_calc, n_iterations/t_calc);

  return true;
}



/*******************************************************************************
 *
 ******************************************************************************/
std::pair<std::string,std::string> recognize_agent_face(EntityBase& entity,
                                                        const ActionScene* scene,
                                                        const std::string& agentName,
                                                        int n_iterations,
                                                        double timeout_in_seconds)
{
  double t_calc = getWallclockTime();
  const Agent* agent = nullptr;

  if (agentName.empty())
  {
    auto humanAgents = scene->getAgents<HumanAgent>();
    if (!humanAgents.empty())
    {
      agent = humanAgents[0];
    }
  }
  else
  {
    agent = scene->getAgent(agentName);
  }

  if (!agent)
  {
    RLOG_CPP(1, "Agent '" << agentName << "' not found in scene");
    return std::pair<std::string,std::string>();
  }

  auto humanAgent = dynamic_cast<const HumanAgent*>(agent);

  if (!humanAgent)
  {
    RLOG_CPP(1, "Agent '" << agentName << "' is not a human agent");
    return std::pair<std::string,std::string>();
  }

  std::map<std::string,int> detections;

  for (size_t i = 0; i < n_iterations; ++i)
  {
    nlohmann::json bb_json;

    bb_json["bounding_box"] =
    {
      { "left",   humanAgent->bb_head[0]},
      { "top",    humanAgent->bb_head[1]},
      { "right",  humanAgent->bb_head[2]},
      { "bottom", humanAgent->bb_head[3]}
    };

    RLOG_CPP(1, "track_agent_facemesh iteration " << i
             << " with bounding box " << bb_json.dump());
    REXEC(0)
    {
      std::cout << ".";
    }

    std::string recognized = recognize_faces(entity, bb_json.dump(), 1, timeout_in_seconds);

    if (!recognized.empty())
    {
      detections[recognized]++;
    }

  }

  t_calc = getWallclockTime() - t_calc;

  REXEC(0)
  {
    std::cout << std::endl;
  }

  RLOG(0, "Took %.2f sec (is %.2f fps)", t_calc, n_iterations/t_calc);

  int winnerCount = 0;
  std::string winnerName;
  for (const auto& detection : detections)
  {
    std::cout << "Key: " << detection.first << ", Value: " << detection.second << "\n";

    if (detection.second>winnerCount)
    {
      winnerName = detection.first;
      winnerCount = detection.second;
    }
  }

  RLOG(0, "Winner is '%s' with %d out of %d detections",
       winnerName.c_str(), winnerCount, n_iterations);

  return std::make_pair(winnerName, agentName);
}



/*******************************************************************************
 * Must be called after init (entity and scene are captured and must exist)
 ******************************************************************************/
void add_agent_welcome_subscriber(EntityBase& entity, const ActionScene* scene)
{
  entity.subscribe("AgentChanged", [&entity, scene](std::string agentName, bool appeared) mutable
  {
    std::thread([](EntityBase& entity, const ActionScene* scene, std::string agentName, bool appeared)
    {
      RLOG_CPP(0, "Agent " << agentName << (appeared ? " appeared" : " disappeared"));

      if (appeared)
      {
        std::pair<std::string,std::string> res;
        res = recognize_agent_face(entity, scene, agentName, 3, 2.0);
        std::string recognized = res.first;
        std::string text = recognized.empty() ? "Hello, I don't think we met before." : "Hello " + recognized + " nice to see you!";
        entity.publish("Speak", text);
        entity.publish("RenameAgent", res.second, recognized);
      }
    },
    std::ref(entity), scene, std::move(agentName), appeared).detach();
  });
}

}   // namespace aff
