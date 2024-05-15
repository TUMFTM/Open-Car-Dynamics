// Copyright 2023 Philip Pitschi
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

#include "tum_type_conversions_ros_cpp/orientation.hpp"
namespace tam::type_conversions
{
unsigned char diagnostic_level_from_type(const tam::types::ErrorLvl & lvl)
{
  unsigned char lvl_out;
  switch (lvl) {
    case tam::types::ErrorLvl::OK:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::OK;
      break;
    case tam::types::ErrorLvl::STALE:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      break;
    case tam::types::ErrorLvl::ERROR:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      break;
    case tam::types::ErrorLvl::WARN:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
    default:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      std::cerr << "Unknown Error Level \n";
      break;
  }
  return lvl_out;
}
tam::types::ErrorLvl error_type_from_diagnostic_level(unsigned char lvl)
{
  tam::types::ErrorLvl lvl_out;
  switch (lvl) {
    case diagnostic_msgs::msg::DiagnosticStatus::OK:
      lvl_out = tam::types::ErrorLvl::OK;
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::WARN:
      lvl_out = tam::types::ErrorLvl::WARN;
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      lvl_out = tam::types::ErrorLvl::ERROR;
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::STALE:
      lvl_out = tam::types::ErrorLvl::STALE;
      break;
    default:
      lvl_out = tam::types::ErrorLvl::ERROR;
      std::cerr << "Unknown Error Level \n";
      break;
  }
  return lvl_out;
}
tam::types::control::Odometry odometry_type_from_imu_msg(sensor_msgs::msg::Imu const & msg)
{
  tam::types::control::Odometry odometry;
  odometry.angular_velocity_radps.x = msg.angular_velocity.x;
  odometry.angular_velocity_radps.y = msg.angular_velocity.y;
  odometry.angular_velocity_radps.z = msg.angular_velocity.z;

  return odometry;
}
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  sensor_msgs::msg::Imu const & msg)
{
  tam::types::control::AccelerationwithCovariances acceleration;
  acceleration.acceleration_mps2.x = msg.linear_acceleration.x;
  acceleration.acceleration_mps2.y = msg.linear_acceleration.y;
  acceleration.acceleration_mps2.z = msg.linear_acceleration.z;

  return acceleration;
}
tam::types::control::Odometry odometry_type_from_msg(nav_msgs::msg::Odometry const & msg)
{
  tam::types::control::Odometry odometry;
  odometry.position_m.x = msg.pose.pose.position.x;
  odometry.position_m.y = msg.pose.pose.position.y;
  odometry.position_m.z = msg.pose.pose.position.z;
  odometry.orientation_rad = tam::helpers::types::to_vector3d(
    tam::types::conversion::quaternion_msg_to_euler_type(msg.pose.pose.orientation));
  odometry.pose_covariance = msg.pose.covariance;
  odometry.angular_velocity_radps.x = msg.twist.twist.angular.x;
  odometry.angular_velocity_radps.y = msg.twist.twist.angular.y;
  odometry.angular_velocity_radps.z = msg.twist.twist.angular.z;
  odometry.velocity_mps.x = msg.twist.twist.linear.x;
  odometry.velocity_mps.y = msg.twist.twist.linear.y;
  odometry.velocity_mps.z = msg.twist.twist.linear.z;

  return odometry;
}
nav_msgs::msg::Odometry odometry_msg_from_type(const tam::types::control::Odometry & odometry)
{
  nav_msgs::msg::Odometry odometry_msg;
  odometry_msg.pose.pose.position.x = odometry.position_m.x;
  odometry_msg.pose.pose.position.y = odometry.position_m.y;
  odometry_msg.pose.pose.position.z = odometry.position_m.z;
  odometry_msg.pose.pose.orientation =
    tam::types::conversion::euler_type_to_quaternion_msg(tam::types::common::EulerYPR(
      odometry.orientation_rad.z, odometry.orientation_rad.y, odometry.orientation_rad.x));
  odometry_msg.pose.covariance = odometry.pose_covariance;
  odometry_msg.twist.twist.linear.x = odometry.velocity_mps.x;
  odometry_msg.twist.twist.linear.y = odometry.velocity_mps.y;
  odometry_msg.twist.twist.linear.z = odometry.velocity_mps.y;
  odometry_msg.twist.twist.angular.x = odometry.angular_velocity_radps.x;
  odometry_msg.twist.twist.angular.y = odometry.angular_velocity_radps.y;
  odometry_msg.twist.twist.angular.z = odometry.angular_velocity_radps.z;

  return odometry_msg;
}
geometry_msgs::msg::AccelWithCovarianceStamped accel_with_covariance_stamped_msg_from_type(
  const tam::types::control::AccelerationwithCovariances & acceleration)
{
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration_msg;
  acceleration_msg.accel.accel.linear = vector_3d_msg_from_type(acceleration.acceleration_mps2);
  acceleration_msg.accel.accel.angular =
    vector_3d_msg_from_type(acceleration.angular_acceleration_radps2);
  acceleration_msg.accel.covariance = acceleration.acceleration_covariance;
  return acceleration_msg;
}
tam::types::control::AccelerationwithCovariances accel_with_covariance_stamped_type_from_msg(
  geometry_msgs::msg::AccelWithCovarianceStamped const & msg)
{
  tam::types::control::AccelerationwithCovariances type_;
  type_.acceleration_mps2 = vector_3d_type_from_msg(msg.accel.accel.linear);
  type_.angular_acceleration_radps2 = vector_3d_type_from_msg(msg.accel.accel.angular);
  type_.acceleration_covariance = msg.accel.covariance;
  return type_;
}
geometry_msgs::msg::Vector3 vector_3d_msg_from_type(
  tam::types::common::Vector3D<double> const & vector_3d)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = vector_3d.x;
  msg.y = vector_3d.y;
  msg.z = vector_3d.z;
  return msg;
}
tam::types::common::Vector3D<double> vector_3d_type_from_msg(
  geometry_msgs::msg::Vector3 const & msg)
{
  return tam::types::common::Vector3D<double>{msg.x, msg.y, msg.z};
}
tum_msgs::msg::TUMFloat64PerWheel data_per_wheel_msg_from_type(
  tam::types::common::DataPerWheel<double> const & data)
{
  tum_msgs::msg::TUMFloat64PerWheel msg;
  msg.front_left = data.front_left;
  msg.front_right = data.front_right;
  msg.rear_left = data.rear_left;
  msg.rear_right = data.rear_right;
  return msg;
}
tam::types::common::DataPerWheel<double> data_per_wheel_type_from_msg(
  tum_msgs::msg::TUMFloat64PerWheel const & msg)
{
  return tam::types::common::DataPerWheel<double>{
    msg.front_left, msg.front_right, msg.rear_left, msg.rear_right};
}
tam::types::common::DataPerWheel<double> data_per_wheel_type_from_msg(
  const tum_msgs::msg::TUMFloat64PerWheel::SharedPtr msg)
{
  return data_per_wheel_type_from_msg(*msg);
}
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  return acceleration_with_covariances_type_from_imu_msg(*msg);
}
tam::types::control::Odometry odometry_type_from_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  return odometry_type_from_imu_msg(*msg);
}
}  // namespace tam::type_conversions
// namespace tam::type_conversions
namespace tam::type::conversions::cpp
{
tam::types::control::Odometry Odometry_type_from_msg(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  return tam::type_conversions::odometry_type_from_msg(*msg);
}
}  // namespace tam::type::conversions::cpp
