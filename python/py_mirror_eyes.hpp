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



aff::ImageTracker* getImageTracker(aff::ExampleActionsECS& ex)
{
  auto lmrs = aff::getComponents<aff::ZmqRouterComponent>(ex.getComponentsRef());

  if (lmrs.size() != 1)
  {
    RLOG(0, "Found %zu ZmqRouterComponent instances - must be 1", lmrs.size());
    return nullptr;
  }

  auto imgTracker = lmrs[0]->getTrackers<aff::ImageTracker>();

  if (imgTracker.size() != 1)
  {
    RLOG(0, "Found %zu ImageTracker instances - must be 1", imgTracker.size());
    return nullptr;
  }

  return imgTracker[0];

}



void bind_mirror_eyes(py::class_<aff::ExampleActionsECS>& cls)
{
  //////////////////////////////////////////////////////////////////////////////
  // minX, minY, maxX, maxY
  //////////////////////////////////////////////////////////////////////////////
  cls.def("getGazeObjectBoundingBox", [](aff::ExampleActionsECS& ex) -> std::vector<int>
  {
    if (const aff::ImageTracker* tracker = getImageTracker(ex))
    {
      return tracker->getGazeObjectBoundingBox();
    }

    return std::vector<int>();
  })

  //////////////////////////////////////////////////////////////////////////////
  // minX, minY, maxX, maxY
  //////////////////////////////////////////////////////////////////////////////
  .def("getObjectBoundingBox", [](aff::ExampleActionsECS& ex, std::string objName) -> std::vector<int>
  {
    py::gil_scoped_release release;  // Unblock Python while we wait
    auto sub = std::make_shared<ES::ScopedSubscription>();
    std::atomic<bool> active{true};

    // promise/future pair for a single result
    auto prom = std::make_shared<std::promise<std::vector<int>>>();
    std::future<std::vector<int>> fut = prom->get_future();

    // Subscribe once; deliver exactly one result
    auto callback = [prom, &ex, objName, &active]
                    (RcsGraph*, RcsGraph*, aff::ActionScene*) mutable
    {
      // Allow only the first invocation to proceed
      if (!active.exchange(false))
      {
        return;
      }

      std::vector<int> bb;
      if (auto* tracker = getImageTracker(ex))
      {
        auto intr = tracker->getCameraParameters();
        bb = aff::ImageTracker::getObjectBoundingBox(
          ex.getScene(), ex.getGraph(), objName, tracker->getCameraName(),
          intr[0], intr[1], intr[2], intr[3]);
      }

      // Satisfy the promise (catch in case something races after timeout)
      try
      {
        prom->set_value(std::move(bb));
      }
      catch (const std::future_error&)
      {
        // ignore promise_already_satisfied if something slipped through
      }
    };

    RLOG(1, "Subscribing to 'UpdateScene' event");
    ex.getEntity().withProcessLock([&] {
      *sub = ex.getEntity().subscribe("UpdateScene", std::move(callback));
    });

    // Wait for result with timeout
    const auto timeout = std::chrono::duration<double>(2.0);
    const auto status = fut.wait_for(timeout);

    // We’re done listening either way
    active.store(false);
    ex.getEntity().withProcessLock([&] { sub->unsubscribe(); });

    // On timeout, bail out gracefully
    if (status != std::future_status::ready)
    {
      RLOG(1, "getObjectBoundingBox timed out after 2.0s");
      return std::vector<int>(); // or throw / return sentinel
    }

    // Got a result
    std::vector<int> bb = fut.get();
    if (bb.size() >= 4)
    {
      RLOG_CPP(1, "bb: " << bb[0] << " " << bb[1] << " " << bb[2] << " " << bb[3]);
    }
    else
    {
      RLOG(1, "getObjectBoundingBox returned incomplete box (size=%zu)", bb.size());
    }

    return bb;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns the most recent camera image if newer than the given frame count
  //////////////////////////////////////////////////////////////////////////////
  .def("getCameraImage", [](aff::ExampleActionsECS& ex, int frame_count) -> std::pair<int, std::string>
  {
    if (aff::ImageTracker* tracker = getImageTracker(ex))
    {
      return tracker->getStampedImage(frame_count);
    }

    return std::pair<int, std::string>(0, "");
  },
  "Return image if more recent than frame_count",
  py::arg("frame_count") = -1)

  //////////////////////////////////////////////////////////////////////////////
  // Head gestures: "yes", "no"
  // "StartGesture": std::string gestureName, double gestureAmplitude, int numTurns
  // "GestureThreeRepetitions": std::string gestureName, double gestureAmplitude
  //////////////////////////////////////////////////////////////////////////////
  .def("setHeadGesture", [](aff::ExampleActionsECS& ex, std::string gestureName, double gestureAmplitude, int numTurns)
  {
    ex.getEntity().publish("StartGesture", gestureName, gestureAmplitude, numTurns);
  },
  py::arg("gestureName"),
  py::arg("gestureAmplitude") = RCS_DEG2RAD(5.0),
  py::arg("numTurns") = 3)

  //////////////////////////////////////////////////////////////////////////////
  // Gaze model methods: 0: Neck only, 1: pupils only.
  //////////////////////////////////////////////////////////////////////////////
  .def("setPupilSpeedWeight", [](aff::ExampleActionsECS& ex, double value)
  {
    ex.getEntity().publish("SetPupilWeight", value);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Pupil point in screen coordinates: z points outwards, x points left, y
  // points down. Origin is screen center. TODO: Make threadsafe
  //////////////////////////////////////////////////////////////////////////////
  .def("getPupilCoordinates", [](aff::ExampleActionsECS& ex) -> std::pair<std::vector<double>, std::vector<double>>
  {
    double pr[3], pl[3];
    std::vector<double> xy_right, xy_left;

    bool success = aff::ActionEyeGaze::computePupilCoordinates(ex.getGraph(), pr, pl);

    if (success)
    {
      xy_right = std::vector<double>(pr, pr + 3);
      xy_left = std::vector<double>(pl, pl + 3);
    }

    return std::make_pair(xy_right, xy_left);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Returns the json with all relevant coordinates for the pupils etc:
  //////////////////////////////////////////////////////////////////////////////
  .def("getMirrorEyesData", [](aff::ExampleActionsECS& ex) -> nlohmann::json
  {
    nlohmann::json j;
    auto eyeComponents = aff::getComponents<aff::EyeModelIKComponent>(ex.getComponentsRef());

    if (eyeComponents.size() != 1)
    {
      RLOG(0, "Found %zu EyeModelIKComponent instances - must be 1", eyeComponents.size());
      return j;
    }

    std::string eyeStr = eyeComponents[0]->getMirrorEyesJsonString();

    try
    {
      j = nlohmann::json::parse(eyeStr);
    }
    catch (const nlohmann::json::parse_error& e)
    {
      RLOG_CPP(0, "Parse error for '" << eyeStr << "' : " << e.what());
    }

    return j;
  },
    // *INDENT-OFF*
    R"pbdoc(
Returns the json with all relevant coordinates for the pupils in this form:

  {
      "bounding_box": {
          "vertex": [
              [ 1.27, -0.29, -0.53 ],
              [ 1.18, -0.29, -0.43 ],
              [ 1.27, -0.23, -0.53 ],
              [ 1.18, -0.23, -0.43 ],
              [ 1.27, -0.29, -0.53 ],
              [ 1.18, -0.29, -0.43 ],
              [ 1.27, -0.23, -0.53 ],
              [ 1.18, -0.23, -0.43 ],
          ],
              "x": 1.297,
              "y" : -0.268,
              "z" : -0.515
      },
          "left_eye": {
          "gaze_distance": 0.96,
              "screen_coordinates" : [ -0.05, 0.01, -1.6 ]
      },
          "right_eye": {
          "gaze_distance": 0.94,
              "screen_coordinates" : [ 0.06, 0.015, -1.6 ]
      },
      "screen_distance": 2.199
  }

)pbdoc"
// *INDENT-ON*
)


;
}
