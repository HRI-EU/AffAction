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



namespace aff
{





std::string recognize_faces(EntityBase& entity, int n_iterations, double timeout_in_seconds)
{
  auto sub = std::make_shared<ES::ScopedSubscription>();
  auto active = std::make_shared<std::atomic<bool>>(true);  // Shared active flag
  std::mutex mtx;
  std::condition_variable cv;
  int counter = 0;
  std::string faceName;

  auto callback = [sub=sub, n=n_iterations, active=active, &mtx, &cv, &counter, &faceName]
                  (std::string id, std::string data) mutable
  {
    if (!*active)
    {
      RLOG_CPP(1, "Callback skipped because function is no longer active.");
      sub.reset();
      return;
    }

    std::lock_guard<std::mutex> lk(mtx);
    RLOG_CPP(1, "id: " << id << " data: " << data);

    if (id=="face_recog")
    {
      // Parse the JSON
      nlohmann::json j = nlohmann::json::parse(data);

      // Check existence and type of "face_recog"
      if (j.contains("data") &&
          j["data"].contains("face_recog") &&
          j["data"]["face_recog"].is_array())
      {
        counter++;
        const auto& faces = j["data"]["face_recog"];
        RLOG_CPP(1, "Iteration " << counter << ": number of recognized faces: " << faces.size());

        if (!faces.empty())
        {
          const nlohmann::json& first_face = faces[0];

          faceName = first_face["recognized_face"];
          auto& bbox = first_face["bounding_box"];

          RLOG_CPP(1, "First recognized face: " << faceName);
          RLOG_CPP(1, "Bounding box: left=" << bbox["left"]
                   << ", top=" << bbox["top"]
                   << ", right=" << bbox["right"]
                   << ", bottom=" << bbox["bottom"]);
        }
        else
        {
          RLOG_CPP(1, "No faces found!");
        }

      }
      else
      {
        RLOG_CPP(1, "\"face_recog\" array not found.");
      }

    }

    if (counter >= n)
    {
      *active = false;  // prevent further callback execution
      sub.reset();      // explicitly unsubscribe
      cv.notify_one();   // wake calling context
    }
  };

  RLOG(1, "Subscribing ZmqDealerMessage");
  *sub = entity.subscribe("ZmqDealerMessage", std::move(callback));

  RLOG(1, "Publishing PerceptionCommand");
  entity.publish("SetPerceptionCommand", std::string("face_recog"), n_iterations);

  {
    std::unique_lock<std::mutex> lk(mtx);
    RLOG(1, "cv.wait");

    bool success = cv.wait_for(lk, std::chrono::duration<double>(timeout_in_seconds), [&]()
    {
      return counter >= n_iterations;
    });

    if (!success)
    {
      RLOG_CPP(0, "Timeout reached while waiting for face recognition.");
    }

    *active = false;  // ensure no more callbacks after return
    sub.reset();      // explicitly unsubscribe
    RLOG(1, "done cv.wait");
  }

  return faceName;
}





bool track_facemesh(EntityBase& entity, int n_iterations, double timeout_in_seconds)
{
  auto sub = std::make_shared<ES::ScopedSubscription>();
  auto active = std::make_shared<std::atomic<bool>>(true);  // Shared active flag
  std::mutex mtx;
  std::condition_variable cv;
  int counter = 0;

  auto callback = [sub=sub, n=n_iterations, active=active, &mtx, &cv, &counter]
                  (std::string id, std::string data) mutable
  {
    if (!*active)
    {
      RLOG_CPP(0, "Callback skipped because function is no longer active.");
      return;
    }

    std::lock_guard<std::mutex> lk(mtx);

    if (id=="mediapipe")
    {
      counter++;
      RLOG_CPP(0, "Received mediapipe reply: " << counter);
    }

    if (counter >= n)
    {
      *active = false;  // prevent further callback execution
      sub.reset();
      cv.notify_one();   // wake calling context
    }
  };

  RLOG(0, "Subscribing ZmqDealerMessage");
  *sub = entity.subscribe("ZmqDealerMessage", std::move(callback));

  RLOG(0, "Publishing PerceptionCommand");
  entity.publish("SetPerceptionCommand", std::string("mediapipe"), n_iterations);

  {
    std::unique_lock<std::mutex> lk(mtx);
    RLOG(0, "cv.wait");

    bool success = cv.wait_for(lk, std::chrono::duration<double>(timeout_in_seconds), [&]()
    {
      return counter >= n_iterations;
    });

    if (!success)
    {
      RLOG_CPP(0, "Timeout reached while waiting for face mesh.");
    }

    *active = false;  // ensure no more callbacks after return
    sub.reset();      // explicitly unsubscribe
    RLOG(0, "done cv.wait");
  }

  RLOG(0, "done track_facemesh");
  return true;
}




}   // namespace aff
