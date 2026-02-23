// Copyright 2026 Simon Sagmeister
#pragma once

#include <concepts>

#include "ocd_drivetrain_model_base_cpp/base_class.hpp"
namespace tam::ocd::interfaces::concepts
{
/// @brief Concept for drivetrain model types
template <typename T>
concept DrivetrainModel = std::derived_from<
  T, tam::ocd::interfaces::DrivetrainModelBase<
       typename T::DriverInputType, typename T::FeedbackType, T::k_state_vector_length,
       typename T::StateNamesTrait, typename T::AuxiliaryInputType>> &&
  (!std::is_abstract_v<T>);  // Check that all pure virtual functions are implemented by the derived
                             // class

}  // namespace tam::ocd::interfaces::concepts
