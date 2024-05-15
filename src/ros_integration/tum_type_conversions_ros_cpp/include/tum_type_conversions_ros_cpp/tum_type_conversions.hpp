// Copyright 2023 Philipp Pitschi
#pragma once

// ROS
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>

// messages
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel.hpp"
// types
#include "tum_type_conversions_ros_cpp/helpers/type_conversion.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::type_conversions
{
unsigned char diagnostic_level_from_type(const tam::types::ErrorLvl & lvl);
tam::types::ErrorLvl error_type_from_diagnostic_level(unsigned char lvl);
tam::types::common::Vector3D<double> vector_3d_type_from_msg(
  geometry_msgs::msg::Vector3 const & msg);
geometry_msgs::msg::Vector3 vector_3d_msg_from_type(
  tam::types::common::Vector3D<double> const & vector_3d);
tam::types::common::DataPerWheel<double> data_per_wheel_type_from_msg(
  tum_msgs::msg::TUMFloat64PerWheel const & msg);
tum_msgs::msg::TUMFloat64PerWheel data_per_wheel_msg_from_type(
  tam::types::common::DataPerWheel<double> const & data);
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  sensor_msgs::msg::Imu const & msg);
nav_msgs::msg::Odometry odometry_msg_from_type(const tam::types::control::Odometry & odometry);
tam::types::control::Odometry odometry_type_from_msg(nav_msgs::msg::Odometry const & odometry);
tam::types::control::Odometry odometry_type_from_imu_msg(sensor_msgs::msg::Imu const & msg);
geometry_msgs::msg::AccelWithCovarianceStamped accel_with_covariance_stamped_msg_from_type(
  const tam::types::control::AccelerationwithCovariances & acceleration);
tam::types::control::AccelerationwithCovariances accel_with_covariance_stamped_type_from_msg(
  geometry_msgs::msg::AccelWithCovarianceStamped const & msg);
// STUFF BELOW THIS LINE IS JUST HERE FOR BACKWARDS COMPATIBILITY
// DO NOT USE ANY MORE
// ==============================================================================
tam::types::control::Odometry odometry_type_from_imu_msg(
  const sensor_msgs::msg::Imu::SharedPtr msg);
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  const sensor_msgs::msg::Imu::SharedPtr msg);
tam::types::common::DataPerWheel<double> data_per_wheel_type_from_msg(
  const tum_msgs::msg::TUMFloat64PerWheel::SharedPtr msg);
}  // namespace tam::type_conversions
namespace tam::type::conversions::cpp
{

/// @brief Here for backwards compability
tam::types::control::Odometry Odometry_type_from_msg(const nav_msgs::msg::Odometry::SharedPtr msg);
}  // namespace tam::type::conversions::cpp
