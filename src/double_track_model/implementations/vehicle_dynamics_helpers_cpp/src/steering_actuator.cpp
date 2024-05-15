// Copyright 2023 Simon Sagmeister
#include "vehicle_dynamics_helpers_cpp/steering_actuator.hpp"
namespace tam::sim::helpers::steering_actuator
{
SteeringActuatorModel::SteeringActuatorModel() { declare_parameters(); }
void SteeringActuatorModel::declare_parameters()
{
  using pt = tam::types::param::ParameterType;
  param_manager_->declare_parameter("steering_actuator.T_PT1", 0.05, pt::DOUBLE, "");
  param_manager_->declare_parameter("steering_actuator.angle_max_rad", 0.3, pt::DOUBLE, "");
  param_manager_->declare_parameter(
    "steering_actuator.angle_rate_max_radps", 1.0, pt::DOUBLE, "");
  param_manager_->declare_parameter(
    "steering_actuator.static_offset_rad", 0.0, pt::DOUBLE, "'Oval steering'");
}
void SteeringActuatorModel::evaluate()
{
  double max_angle =
    param_manager_->get_parameter_value("steering_actuator.angle_max_rad").as_double();
  double max_rate =
    param_manager_->get_parameter_value("steering_actuator.angle_rate_max_radps").as_double();
  double x_dot = (u_ - x_vec_[x::position_rad]) /
                 param_manager_->get_parameter_value("steering_actuator.T_PT1").as_double();
  // Enforce the maximum rate
  x_dot = std::min(max_rate, std::max(-max_rate, x_dot));
  // Enforce the maximum angle
  if (x_vec_[x::position_rad] <= -max_angle) x_dot = std::max(0.0, x_dot);
  if (x_vec_[x::position_rad] >= max_angle) x_dot = std::min(0.0, x_dot);
  // Set the x_dot vec
  x_dot_vec_[x::position_rad] = x_dot;
  y_ = x_vec_[x::position_rad] +
       param_manager_->get_parameter_value("steering_actuator.static_offset_rad").as_double();

  // Log stuff
  debug_container_->log("target", u_);
  debug_container_->log("actual", x_vec_[x::position_rad]);
  debug_container_->log("output", y_);
  debug_container_->log("rate", x_dot_vec_[x::position_rad]);
}
}  // namespace tam::sim::helpers::steering_actuator
