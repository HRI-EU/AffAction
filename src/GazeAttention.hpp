#ifndef AFF_GAZEATTENTION_HPP
#define AFF_GAZEATTENTION_HPP

#include "ActionScene.h"
#include "Agent.h"
#include "ComponentBase.h"

#include <Rcs_graph.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <json.hpp>

#include <chrono>
#include <cmath>
#include <string>

namespace aff
{

class GazeAttention : public ComponentBase
{
public:
  GazeAttention(EntityBase* parent, const std::string& agentName) :
    ComponentBase(parent),
    agentName(agentName),
    leftHandArmed(true),
    rightHandArmed(true),
    lastDetection(std::chrono::steady_clock::now() - std::chrono::seconds(2))
  {
    subscribe("UpdateScene", &GazeAttention::onUpdateScene);
  }

private:
  void onUpdateScene(RcsGraph* desired, RcsGraph* current, ActionScene* scene)
  {
    (void) current;

    const auto agents = scene->getAgents<HumanAgent>();
    if (agents.size() != 1)
    {
      RLOG_CPP(1, "Expected exactly one HumanAgent, found " << agents.size());
      return;
    }

    const HumanAgent* agent = agents[0];
    double headPosition[3];
    double gazeDirection[3];
    HTr leftHand;
    HTr rightHand;

    if (!agent->getHeadPositionInWorld(headPosition, desired) ||
        !agent->getGazeDirectionInWorld(gazeDirection, desired) ||
        !agent->getTrackedFrameTransform(&leftHand, desired, "HandLeft") ||
        !agent->getTrackedFrameTransform(&rightHand, desired, "HandRight"))
    {
      RLOG_CPP(1, "Failed to determine gaze or hand kinematics for agent '"
               << agentName << "'");
      return;
    }

    const auto leftManipulators = agent->getManipulatorsOfType(scene, "hand_left");
    const auto rightManipulators = agent->getManipulatorsOfType(scene, "hand_right");
    if (leftManipulators.size() != 1 || rightManipulators.size() != 1)
    {
      RLOG_CPP(1, "Expected one left and one right hand manipulator for agent '"
               << agentName << "'");
      return;
    }

    updateHand(headPosition, gazeDirection, leftHand.org, "left",
               leftManipulators[0]->name, leftHandArmed);
    updateHand(headPosition, gazeDirection, rightHand.org, "right",
               rightManipulators[0]->name, rightHandArmed);
  }

  void updateHand(const double headPosition[3], const double gazeDirection[3],
                  const double handPosition[3], const std::string& side,
                  const std::string& manipulatorName, bool& armed)
  {
    double headToHand[3];
    Vec3d_sub(headToHand, handPosition, headPosition);
    const double gazeLengthSquared = gazeDirection[0]*gazeDirection[0] +
                                     gazeDirection[1]*gazeDirection[1] +
                                     gazeDirection[2]*gazeDirection[2];
    if (gazeLengthSquared < 1.0e-12)
    {
      return;
    }

    // Find the closest point on the forward gaze ray to the hand. A negative
    // projection means the hand is behind the head and therefore not gazed at.
    const double projection =
      (headToHand[0]*gazeDirection[0] +
       headToHand[1]*gazeDirection[1] +
       headToHand[2]*gazeDirection[2]) / gazeLengthSquared;

    double distance = std::sqrt(headToHand[0]*headToHand[0] +
                                headToHand[1]*headToHand[1] +
                                headToHand[2]*headToHand[2]);
    if (projection >= 0.0)
    {
      const double dx = headToHand[0] - projection*gazeDirection[0];
      const double dy = headToHand[1] - projection*gazeDirection[1];
      const double dz = headToHand[2] - projection*gazeDirection[2];
      distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    const auto now = std::chrono::steady_clock::now();
    if (armed && projection >= 0.0 && distance < 0.15 &&
        now - lastDetection >= std::chrono::seconds(2))
    {
      nlohmann::json detail =
      {
        {"side", side},
        {"manipulator", manipulatorName},
        {"distance_m", distance}
      };
      getEntity()->publish<std::string, std::string, std::string>(
        "PublishEvent", "scene", "gaze_at_hand", detail.dump());
      armed = false;
      lastDetection = now;
    }
    else if (!armed && (projection < 0.0 || distance > 0.25))
    {
      armed = true;
    }
  }

  std::string agentName;
  bool leftHandArmed;
  bool rightHandArmed;
  std::chrono::steady_clock::time_point lastDetection;
};

}   // namespace aff

#endif   // AFF_GAZEATTENTION_HPP
