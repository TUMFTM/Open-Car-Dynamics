// Copyright 2023 Simon Sagmeister

#pragma once
#include <math.h>

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager_non_owning.hpp"
#include "simple_drivetrain_cpp/drivetrain_simple_eqns.hpp"
#include "tum_sim_types_cpp/types.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::sim::drivetrain
{
// Wrapping class around the equations
class DriveTrainModelSimple
{
public:
  DriveTrainModelSimple();
  using state_vector_t = drivetrain::state_vector_t;
  using state_enum = x::States;
  static constexpr std::size_t state_vector_length = x::CNT_LENGTH_STATE_VECTOR;
  static constexpr const char * state_enum_to_string(uint8_t enum_value)
  {
    return x::TO_STRING(enum_value);
  }
  tam::types::common::TUMDebugContainer::SharedPtr get_debug_container() { return debug_container; }
  tam::interfaces::ParamManagerBase::SharedPtr get_param_manager() { return param_manager; }
  // Setting inputs
  void set_u(const Input & u_) { eqns.set_u(u_); }
  void set_x_vec(const state_vector_t & x_vec_) { eqns.set_x_vec(x_vec_); }
  void set_external_influences(tam::types::ExternalInfluences const & external_influences)
  {
    eqns.set_external_influences(external_influences);
  }
  // Evaluate the model
  void evaluate();
  // Outputs
  Output get_y();
  state_vector_t get_x_vec() { return eqns.x_vec; }
  state_vector_t get_x_dot_vec() { return eqns.x_dot_vec; }

private:
  DriveTrainEquationsSimple eqns;
  DriveTrainEquationsSimple::Parameters param_input_struct;
  tam::types::common::TUMDebugContainer::SharedPtr debug_container =
    std::make_shared<tam::types::common::TUMDebugContainer>();
  std::shared_ptr<tam::core::ParamManagerNonOwning> param_manager =
    std::make_shared<tam::core::ParamManagerNonOwning>();
  int64_t param_state_counter_prev_ = 0;
  void declare_parameters();
  void assign_debug_outputs();
};
}  // namespace tam::sim::drivetrain
