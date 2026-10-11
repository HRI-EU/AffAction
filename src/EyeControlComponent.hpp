/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the conditions in the project
  license are met.

*******************************************************************************/

#ifndef AFF_EYECONTROLCOMPONENT_HPP
#define AFF_EYECONTROLCOMPONENT_HPP

#include "ComponentBase.h"
#include "EyeModelIKComponent.h"
#include "json.hpp"

#include <Rcs_macros.h>

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>


namespace aff
{

/*! \brief Publishes the latest EyeModelIK state for a Mirror Eyes DEALER.
 *
 * The component deliberately does not own eye-coordinate or ROI computation.
 * EyeModelIKComponent remains authoritative and supplies its thread-safe JSON
 * snapshot. ZmqRouterComponent is expected to handle SetPerceptionState with
 * latest-value semantics for the target identity.
 */
class EyeControlComponent : public ComponentBase
{
public:
  EyeControlComponent(EntityBase* parent,
                      const EyeModelIKComponent* eyeModel,
                      std::string targetIdentity="mirror_eyes",
                      unsigned int publishEveryNUpdates=4) :
    ComponentBase(parent),
    eyeModel(eyeModel),
    targetIdentity(std::move(targetIdentity)),
    publishEveryNUpdates(publishEveryNUpdates),
    updateCount(0),
    sequence(0)
  {
    if (this->eyeModel == nullptr)
    {
      throw std::invalid_argument("EyeControlComponent requires EyeModelIKComponent");
    }
    if (this->publishEveryNUpdates == 0)
    {
      throw std::invalid_argument("publishEveryNUpdates must be positive");
    }
    subscribe("PostUpdateGraph", &EyeControlComponent::onPostUpdateGraph);
  }

  std::string getName() const override
  {
    return "EyeControlComponent";
  }

private:
  void onPostUpdateGraph(RcsGraph*, RcsGraph*)
  {
    if ((++updateCount % publishEveryNUpdates) != 0)
    {
      return;
    }

    const std::string stateString = eyeModel->getMirrorEyesJsonString();
    if (stateString.empty())
    {
      return;
    }

    try
    {
      nlohmann::json state = nlohmann::json::parse(stateString);
      state["kind"] = "state";
      state["version"] = 1;
      state["sequence"] = sequence++;
      state["timestamp"] = monotonicSeconds();
      getEntity()->publish("SetPerceptionState", targetIdentity, state.dump());
    }
    catch (const nlohmann::json::exception& exception)
    {
      RLOG_CPP(1, "Invalid EyeModelIK state: " << exception.what());
    }
  }

  static double monotonicSeconds()
  {
    using Seconds = std::chrono::duration<double>;
    return Seconds(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  const EyeModelIKComponent* eyeModel;
  std::string targetIdentity;
  unsigned int publishEveryNUpdates;
  unsigned int updateCount;
  std::uint64_t sequence;
};

}   // namespace aff

#endif   // AFF_EYECONTROLCOMPONENT_HPP
