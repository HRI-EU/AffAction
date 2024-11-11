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

#ifndef ARUCOTRACKER_H
#define ARUCOTRACKER_H

#include "TrackerBase.h"

#include <Rcs_HTr.h>

#include <map>
#include <mutex>
#include <memory>
#include <functional>
#include <vector>



namespace aff
{

class ArucoCalibrator;

class ArucoTracker : public TrackerBase
{
public:

  /*!
   * \brief Constructs an ArucoTracker instance for managing the position of
   *        bodies based on detected ArUco markers.
   *
   * This constructor initializes the tracker with specified camera and base
   * marker body names. These names are used to reference the camera and base
   * marker within the RcsGraph, allowing the ArucoTracker to calculate marker
   * transformations and update the graph accordingly.
   *
   * \param[in] cameraBodyName Name of the camera body in the RcsGraph.
   * \param[in] baseMarkerBdyName Name of the base marker body in the RcsGraph.
   */
  ArucoTracker(const std::string& cameraBodyName, const std::string& baseMarkerBdyName);

  /*!
   * \brief Destructor for ArucoTracker.
   *
   * Cleans up any allocated resources and ensures that calibration-related
   * callbacks are deregistered.
   */
  virtual ~ArucoTracker();

  /*!
   * \brief Updates the RcsGraph with the current transformations derived from
   *        detected ArUco markers.
   *
   * Called from the control loop at regular intervals (approximately 100 Hz),
   * this method processes the latest detected marker transformations and
   * applies them to the corresponding bodies in the RcsGraph. It ensures
   * accurate real-time tracking by updating only the relevant transformations
   * based on marker detections, filtering, and validation checks.
   *
   * \param[in,out] graph Pointer to the RcsGraph structure representing the
   *                      tracked bodies and joints.
   */
  void updateGraph(RcsGraph* graph);

  /*!
   * \brief Returns the keyword associated with ArUco marker processing
   *        requests.
   *
   * This method provides a keyword identifier ("aruco") used for managing
   * requests related to ArUco markers, making it easier to identify and
   * separate different request types within the system.
   *
   * \return A string representing the keyword ("aruco").
   */
  virtual std::string getRequestKeyword() const;

  /*!
   * \brief Initiates the camera pose calibration process using ArUco markers.
   *
   * This method starts the calibration process by averaging the detected
   * marker transformations over a specified number of frames. The calibration
   * aligns the camera's transformation relative to the base marker, refining
   * its accuracy by applying a low-pass filter and averaging marker positions.
   *
   * \param[in] numFrames Number of frames over which to average the
   *                      calibration.
   */
  void calibrate(size_t numFrames);

  /*!
   * \brief Parses JSON data containing ArUco marker information to update
   *        marker positions.
   *
   * Processes incoming JSON data that includes the position and orientation
   * of detected ArUco markers. This method extracts relevant details from the
   * JSON, transforms them as needed, and stores them in the ArucoTracker's
   * internal map for later use in updating the RcsGraph.
   *
   * \param[in] json JSON object with position and orientation for each marker.
   * \param[in] time Timestamp of the received data, used for marker updates.
   * \param[in] cameraFrame Frame identifier for the camera's coordinate system.
   */
  void parse(const nlohmann::json& json, double time, const std::string& cameraFrame);

  /*!
   * \brief Sets the camera's transformation relative to the world frame.
   *
   * This method updates the camera's transformation (position and orientation)
   * in the world frame, enabling accurate alignment of detected markers with
   * the physical camera setup. This is essential for interpreting marker
   * positions accurately in world coordinates.
   *
   * \param[in] A_CI Transformation matrix of the camera in world coordinates.
   */
  void setCameraTransform(const HTr* A_CI);

  /*!
   * \brief Registers a callback function to be triggered upon completion of
   *        camera calibration.
   *
   * Adds a user-defined callback to the list of functions executed when the
   * calibration process finishes. This allows external components to respond
   * to calibration completion and adjust configurations or settings as needed.
   *
   * \param[in] callback Function to be called with the final calibrated
   *                     camera transform.
   */
  void addCalibrationFinishedCallback(std::function<void(const HTr*)> callback);

  /*!
   * \brief Gets the name of the base marker body used for calibration.
   *
   * The base marker name is crucial in identifying the reference marker
   * body in the RcsGraph. This method provides access to the name for use
   * in other operations involving the calibration or alignment of the
   * camera.
   *
   * \return A string representing the base marker's body name.
   */
  std::string getBaseMarkerName() const;

  /*!
   * \brief Gets the name of the camera body used for calibration.
   *
   * The camera body name is used to identify the camera body in the RcsGraph,
   * which serves as a reference point for applying transformations based on
   * marker detections.
   *
   * \return A string representing the camera's body name.
   */
  std::string getCameraName() const;

  /*!
   * \brief Checks if the ArucoTracker is currently in the calibration process.
   *
   * This method indicates whether the calibration is active by verifying if
   * the calibration frame count is within the specified limit. It allows
   * external systems to know if the tracker is adjusting for precise alignment.
   *
   * \return True if calibration is in progress; false otherwise.
   */
  bool isCalibrating() const;

  bool initDebugGraphics(Rcs::Viewer* viewer, const RcsGraph* graph);


private:

  struct MarkerBodyData
  {
    MarkerBodyData();
    bool isHeldInHand() const;
    bool hasUpdate() const;
    void print(const RcsGraph* graph=nullptr) const;
    const RcsBody* body(const RcsGraph* graph);

    int bodyId;
    int jointIndex;
    double t_latest;
    bool heldInHand;
    bool frozen;
    std::vector<double> q_rbj;
    std::vector<std::string> markerNames;
  };

  std::unique_ptr<ArucoCalibrator> calibration;

  // marker-name (aruco_05 etc.) - 12 + 1 doubles (transform + update time)
  std::map<std::string,std::vector<double>> arucoMap;

  std::map<std::string, ArucoTracker::MarkerBodyData> markerMap;

  HTr A_CI;         // Transform from world frame into camera frame
  bool newArucoUpdate;
  std::mutex arucoMapMtx;

  //  Tuple: 1. body name, 2. q-index, 3. vector with 6 joint values
  std::vector<std::tuple<std::string,std::vector<std::string>,int,std::vector<double>>> markers;

  static MarkerBodyData computeBodyDofsFromAruco(const RcsGraph* graph,
                                                 const RcsBody* body,
                                                 const HTr* T_camI,
                                                 const std::map<std::string, std::vector<double>>& arucoMap);

  static void computeDofsFromAruco(const RcsGraph* graph,
                                   const HTr* T_camI,
                                   const std::map<std::string, std::vector<double>>& arucoMap,
                                   std::map<std::string, ArucoTracker::MarkerBodyData>& markerMap);
};

}   // namespace

#endif // ARUCOTRACKER_H
