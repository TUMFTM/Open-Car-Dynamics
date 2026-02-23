// Copyright 2026 Simon Sagmeister

#include "ocd_steering_actuator_pt1_cpp/steering_actuator_pt1_model.hpp"
namespace tam::ocd::steering_actuator
{
PT1SteeringActuatorModel::PT1SteeringActuatorModel()
{
  declare_parameters();
  register_log_signals();
}
void PT1SteeringActuatorModel::evaluate()
{
  double max_angle = p_.angle_max_rad;
  double max_rate = p_.angle_rate_max_radps;
  double x_dot = (requested_steering_angle_rad_ - x_vec_[x::position_rad]) / p_.T_PT1;
  // Enforce the maximum rate
  x_dot = std::clamp(x_dot, -max_rate, max_rate);
  // Enforce the maximum angle
  if (x_vec_[x::position_rad] <= -max_angle) x_dot = std::max(0.0, x_dot);
  if (x_vec_[x::position_rad] >= max_angle) x_dot = std::min(0.0, x_dot);
  // Set the x_dot vec
  x_dot_vec_[x::position_rad] = x_dot;
  measured_steering_angle_rad_ = x_vec_[x::position_rad];
  actual_steering_angle_rad_ = measured_steering_angle_rad_ + p_.static_offset_rad;
}
void PT1SteeringActuatorModel::set_x_vec(const StateVectorType & x_vec) { x_vec_ = x_vec; }
void PT1SteeringActuatorModel::set_driver_input(const DriverInputType & input)
{
  requested_steering_angle_rad_ = input.steering_angle_rad;
}
void PT1SteeringActuatorModel::set_load(
  const DoublePerWheelType & steering_load_torque_per_wheel_Nm)
{
  steering_load_torque_per_wheel_Nm_ = steering_load_torque_per_wheel_Nm;
}
PT1SteeringActuatorModel::StateVectorType PT1SteeringActuatorModel::get_x_vec() const
{
  return x_vec_;
}
PT1SteeringActuatorModel::StateVectorType PT1SteeringActuatorModel::get_x_dot_vec() const
{
  return x_dot_vec_;
}
tam::tsl::LoggerAccessInterface::SharedPtr PT1SteeringActuatorModel::get_logger() const
{
  return logger_;
}
tam::pmg::MgmtInterface::SharedPtr PT1SteeringActuatorModel::get_param_manager() const
{
  return param_manager_;
}
PT1SteeringActuatorModel::DoublePerWheelType PT1SteeringActuatorModel::get_steering_angles() const
{
  DoublePerWheelType out;
  // Assume parallel steering for front wheels and zero for rear wheels
  out.front_left = actual_steering_angle_rad_;
  out.front_right = actual_steering_angle_rad_;
  out.rear_left = 0;
  out.rear_right = 0;
  return out;
}
PT1SteeringActuatorModel::FeedbackType PT1SteeringActuatorModel::get_feedback() const
{
  FeedbackType out;
  out.steering_angle_rad = measured_steering_angle_rad_;
  return out;
}
void PT1SteeringActuatorModel::declare_parameters()
{
  using pt = tam::pmg::ParameterType;
  param_manager_->declare_parameter("steering_actuator.T_PT1", &p_.T_PT1, 0.06, pt::DOUBLE, "");
  param_manager_->declare_parameter(
    "steering_actuator.angle_max_rad", &p_.angle_max_rad, 0.3, pt::DOUBLE, "");
  param_manager_->declare_parameter(
    "steering_actuator.angle_rate_max_radps", &p_.angle_rate_max_radps, 0.5, pt::DOUBLE, "");
  param_manager_->declare_parameter(
    "steering_actuator.static_offset_rad", &p_.static_offset_rad, 0.0, pt::DOUBLE,
    "'Oval steering'");
}
void PT1SteeringActuatorModel::register_log_signals()
{
  logger_->log("target_steering_angle_rad", &requested_steering_angle_rad_);
  logger_->log("measured_steering_angle_rad", &measured_steering_angle_rad_);
  logger_->log("actual_steering_angle_rad", &actual_steering_angle_rad_);
  logger_->log("steering_angle_rate_radps", &x_dot_vec_[x::position_rad]);
}
}  // namespace tam::ocd::steering_actuator
