// Copyright 2023 Simon Sagmeister

#pragma once
#include <math.h>

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>

#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager_non_owning.hpp"
#include "tum_types_cpp/common.hpp"
#include "vehicle_dynamics_double_track_cpp/vehicle_dynamics_equations.hpp"
#include "vehicle_dynamics_helpers_cpp/tire_models.hpp"
namespace tam::sim::vd_double_track
{
class VehicleDynamicsDoubleTrackModel
{
public:
  VehicleDynamicsDoubleTrackModel();
  // Stuff for integration
  using state_vector_t = vd_double_track::state_vector_t;
  using state_enum = x::States;
  static constexpr std::size_t state_vector_length = x::CNT_LENGTH_STATE_VECTOR;
  static constexpr const char * state_enum_to_string(uint8_t enum_value)
  {
    return x::TO_STRING(enum_value);
  }
  tam::types::common::TUMDebugContainer::SharedPtr get_debug_container() { return debug_container; }
  tam::interfaces::ParamManagerBase::SharedPtr get_param_manager() { return param_manager; }
  // Settings inputs
  void set_u(const Input & u_);
  void set_x_vec(const state_vector_t & x_vec_) { eqns.x_vec = x_vec_; }
  void set_external_influences(const ExternalInfluences & external_influences)
  {
    eqns.w = external_influences;
  }
  // Evaluate the model
  void evaluate();
  // Getting outputs
  Output get_y() { return eqns.y; }
  state_vector_t get_x_vec() { return eqns.x_vec; }
  state_vector_t get_x_dot_vec() { return eqns.x_dot_vec; }

private:
  VehicleDynamicsDoubleTrackEqns eqns;
  tam::types::common::TUMDebugContainer::SharedPtr debug_container =
    std::make_shared<tam::types::common::TUMDebugContainer>();
  std::shared_ptr<tam::core::ParamManagerNonOwning> param_manager =
    std::make_shared<tam::core::ParamManagerNonOwning>();
  int64_t param_state_count_prev = 0;

  void declare_parameters();
  void assign_debug_outputs();
  static void add_Vector3D_to_map(
    tam::types::common::TUMDebugContainer::SharedPtr container,
    const std::string & debug_signal_name, const tam::types::common::Vector3D<double> data);
  static void add_per_wheel_data_to_map(
    tam::types::common::TUMDebugContainer::SharedPtr container,
    const std::string & debug_signal_name, const double_per_wheel_t data);
  static void add_per_wheel_data_to_map(
    tam::types::common::TUMDebugContainer::SharedPtr container,
    const std::string & debug_signal_name, const double_vector_per_wheel_t data);
};
}  // namespace tam::sim::vd_double_track
