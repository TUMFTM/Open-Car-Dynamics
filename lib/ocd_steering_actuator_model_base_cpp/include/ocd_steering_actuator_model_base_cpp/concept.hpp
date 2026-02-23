// Copyright 2026 Simon Sagmeister
#pragma once

#include <concepts>

#include "ocd_steering_actuator_model_base_cpp/base_class.hpp"
namespace tam::ocd::interfaces::concepts
{
/// @brief Concept for steering actuator model types
template <typename T>
concept SteeringActuatorModel = std::derived_from<
  T, tam::ocd::interfaces::SteeringActuatorModelBase<
       typename T::DriverInputType, typename T::FeedbackType, T::k_state_vector_length,
       typename T::StateNamesTrait, typename T::AuxiliaryInputType>> &&
  (!std::is_abstract_v<T>);  // Check that all pure virtual functions are implemented by the derived
                             // class // NOLINT
}  // namespace tam::ocd::interfaces::concepts
