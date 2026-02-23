// Copyright 2026 Simon Sagmeister

#pragma once
#include <math.h>

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <tum_types_cpp/common.hpp>
#include <unordered_map>

#include "ocd_aerodynamics_model_base_cpp/concept.hpp"
#include "ocd_tire_model_base_cpp/concept.hpp"
#include "ocd_types_cpp/types.hpp"
#include "ocd_vehicle_dynamics_single_track_cpp/logging.hpp"
#include "ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_equations.hpp"
#include "param_management_cpp/base.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
namespace tam::ocd::vehicle_dynamics
{
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
class VehicleDynamicsSingleTrackModel
: public tam::ocd::interfaces::VehicleDynamicsModelBase<
    single_track::States::StateEnum::CNT_LENGTH_STATE_VECTOR, single_track::States::StateNames>
{
  using x = tam::ocd::vehicle_dynamics::single_track::States::StateEnum;
  using double_per_wheel_t = tam::types::common::DataPerWheel<double>;

public:
  VehicleDynamicsSingleTrackModel();
  // Settings inputs
  void set_wheel_speeds(const double_per_wheel_t & wheel_speeds_radps) override;
  void set_steering_angles(const double_per_wheel_t & steering_angle_per_wheel_rad) override;
  void set_external_influences(const types::ExternalInfluences & external_influences) override;
  void set_x_vec(const StateVectorType & x_vec) override;
  // Evaluate the model
  void evaluate() override;
  // Getting outputs
  double_per_wheel_t get_wheel_load() const override;
  double_per_wheel_t get_steering_load() const override;
  types::VehicleDynamicsModelOutput get_vehicle_dynamics_output() const override;
  StateVectorType get_x_vec() const override;
  StateVectorType get_x_dot_vec() const override;

  // Get logger and param manager
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() const override;
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const override;

private:
  VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT> eqns_;
  tam::tsl::ReferenceLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ReferenceLogger>();
  pmg::ParamReferenceManager::SharedPtr param_manager_ =
    std::make_shared<pmg::ParamReferenceManager>();
  int64_t param_state_count_prev_ = 0;

  void declare_parameters();
  void register_log_signals();
};
}  // namespace tam::ocd::vehicle_dynamics
#include "ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_model_impl.hpp"
