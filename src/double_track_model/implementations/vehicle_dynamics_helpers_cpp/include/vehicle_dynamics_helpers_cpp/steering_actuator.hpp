// Copyright 2023 Simon Sagmeister
#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>

#include "param_manager_cpp/param_manager.hpp"
#include "param_manager_cpp/param_manager_base.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::sim::helpers::steering_actuator
{
namespace x
{
enum States { position_rad, CNT_LENGTH_STATE_VECTOR };
constexpr const char * TO_STRING(uint8_t enum_value)
{
  // clang-format off
  switch (enum_value) {
    case States::position_rad: return "position_rad";
    // Just here so silence compiler warnings
    case States::CNT_LENGTH_STATE_VECTOR: return "NOT_DEFINED";
  }
  // clang-format on
  throw std::runtime_error("ERROR: Invalid enum value");
};
}  // namespace x
class SteeringActuatorModel
{
public:
  SteeringActuatorModel();
  using state_vector_t = Eigen::Matrix<double, x::CNT_LENGTH_STATE_VECTOR, 1>;
  using state_enum = x::States;
  static constexpr std::size_t state_vector_length = x::CNT_LENGTH_STATE_VECTOR;
  static constexpr const char * state_enum_to_string(uint8_t enum_value)
  {
    return x::TO_STRING(enum_value);
  }
  tam::interfaces::ParamManagerBase::SharedPtr get_param_manager() { return param_manager_; }
  tam::types::common::TUMDebugContainer::SharedPtr get_debug_container()
  {
    return debug_container_;
  }
  void set_u(double u) { u_ = u; }
  void set_x_vec(state_vector_t x_vec) { x_vec_ = x_vec; }
  state_vector_t get_x_vec() { return x_vec_; }
  state_vector_t get_x_dot_vec() { return x_dot_vec_; }
  double get_y() { return y_; }
  void evaluate();

private:
  double u_;
  double y_;
  state_vector_t x_vec_;
  state_vector_t x_dot_vec_;

  // Additional stuff for debugging and testing
  tam::interfaces::ParamManagerBase::SharedPtr param_manager_ =
    std::make_shared<tam::core::ParamManager>();
  tam::types::common::TUMDebugContainer::SharedPtr debug_container_ =
    std::make_shared<tam::types::common::TUMDebugContainer>();

  void declare_parameters();
};
}  // namespace tam::sim::helpers::steering_actuator
