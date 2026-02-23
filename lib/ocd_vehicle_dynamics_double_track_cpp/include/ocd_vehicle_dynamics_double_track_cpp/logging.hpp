// Copyright 2026 Simon Sagmeister
#pragma once

#include <string>
#include <tum_types_cpp/common.hpp>

#include "tsl_logger_cpp/type_support.hpp"
// Use the type support system to efficiently log the Vector3D type

#ifndef OCD_VEHICLE_DYNAMICS_TYPES_LOGGING
#define OCD_VEHICLE_DYNAMICS_TYPES_LOGGING
namespace tam::tsl::type_support
{
template <>
inline void log<tam::types::common::Vector3D<double>>(
  TypeSupportInterface * logger, std::string const & name,
  tam::types::common::Vector3D<double> const & value)
{
  // You can use the type support system to again log any type that is supported by the interface.
  type_support::log(logger, name + "/x", value.x);
  type_support::log(logger, name + "/y", value.y);
  type_support::log(logger, name + "/z", value.z);
}
template <>
inline void log<tam::types::common::DataPerWheel<double>>(
  TypeSupportInterface * logger, std::string const & name,
  tam::types::common::DataPerWheel<double> const & value)
{
  type_support::log(logger, name + "/front_left", value.front_left);
  type_support::log(logger, name + "/front_right", value.front_right);
  type_support::log(logger, name + "/rear_left", value.rear_left);
  type_support::log(logger, name + "/rear_right", value.rear_right);
}
template <>
inline void log<Eigen::Vector2d>(
  TypeSupportInterface * logger, std::string const & name, Eigen::Vector2d const & value)
{
  type_support::log(logger, name + "/longitudinal", value[0]);
  type_support::log(logger, name + "/lateral", value[1]);
}
}  // namespace tam::tsl::type_support
#endif
