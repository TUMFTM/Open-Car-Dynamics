// Copyright 2026 Simon Sagmeister
#pragma once

#include <concepts>

#include "ocd_vehicle_model_base_cpp/base_class.hpp"
namespace tam::ocd::interfaces::concepts
{
/// @brief Concept for vehicle model types
template <typename T>
concept VehicleModel = std::derived_from<
  T, tam::ocd::interfaces::VehicleModelBase<
       typename T::DrivetrainDriverInputType, typename T::DrivetrainFeedbackType,
       typename T::SteeringActuatorDriverInputType, typename T::SteeringActuatorFeedbackType,
       typename T::DrivetrainAuxiliaryInputType, typename T::SteeringActuatorAuxiliaryInputType>> &&
  (!std::is_abstract_v<T>);  // Check that all pure virtual functions are implemented by the derived
                             // class // NOLINT

}  // namespace tam::ocd::interfaces::concepts
