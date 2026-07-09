// Copyright 2026 Simon Sagmeister
#pragma once
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "ocd_interfaces/msg/external_influences.hpp"
#include "ocd_types_cpp/types.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel.hpp"
#include "tum_type_conversions_ros_cpp/orientation.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
namespace tam::ocd::helpers::type_conversion
{
inline tam::ocd::types::ExternalInfluences external_influences_type_from_msg(
  ocd_interfaces::msg::ExternalInfluences const & msg)
{
  types::ExternalInfluences type_;
  type_.external_force_N = tam::type_conversions::vector_3d_type_from_msg(msg.external_force);
  type_.external_torque_Nm = tam::type_conversions::vector_3d_type_from_msg(msg.external_torque);
  type_.lambda_mue = tam::type_conversions::data_per_wheel_type_from_msg(msg.lambda_mue);
  type_.z_height_road_m = tam::type_conversions::data_per_wheel_type_from_msg(msg.z_height_road_m);
  return type_;
}
inline nav_msgs::msg::Odometry toMsg(const types::VehicleDynamicsModelOutput & in)
{
  nav_msgs::msg::Odometry odom_;
  odom_.pose.pose.position.x = in.position_m.x;
  odom_.pose.pose.position.y = in.position_m.y;
  odom_.pose.pose.position.z = in.position_m.z;
  odom_.pose.pose.orientation = tam::types::conversion::euler_type_to_quaternion_msg(
    tam::types::common::EulerYPR(in.orientation_rad.z, in.orientation_rad.y, in.orientation_rad.x));

  odom_.twist.twist.linear.x = in.velocity_mps.x;
  odom_.twist.twist.linear.y = in.velocity_mps.y;
  odom_.twist.twist.linear.z = in.velocity_mps.z;
  odom_.twist.twist.angular.x = in.angular_velocity_radps.x;
  odom_.twist.twist.angular.y = in.angular_velocity_radps.y;
  odom_.twist.twist.angular.z = in.angular_velocity_radps.z;
  return odom_;
}
inline tum_msgs::msg::TUMFloat64PerWheel toMsg(
  const tam::types::common::DataPerWheel<double> & input)
{
  return tam::type_conversions::data_per_wheel_msg_from_type(input);
}
inline tam::types::common::DataPerWheel<double> fromMsg(
  const tum_msgs::msg::TUMFloat64PerWheel * input)
{
  tam::types::common::DataPerWheel<double> out;
  out.front_left = input->front_left;
  out.front_right = input->front_right;
  out.rear_left = input->rear_left;
  out.rear_right = input->rear_right;
  return out;
}
inline geometry_msgs::msg::Vector3 toMsg(const tam::types::common::Vector3D<double> & input)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = input.x;
  msg.y = input.y;
  msg.z = input.z;
  return msg;
}
}  // namespace tam::ocd::helpers::type_conversion
