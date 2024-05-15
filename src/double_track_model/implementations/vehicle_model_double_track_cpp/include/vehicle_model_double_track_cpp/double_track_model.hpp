// Copyright 2023 Simon Sagmeister

#pragma once
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <functional>
#include <map>
#include <memory>
#include <ranges>
#include <string>

#include "simple_drivetrain_cpp/drivetrain_simple_model.hpp"
#include "param_manager_cpp/param_manager.hpp"
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager_composer.hpp"
#include "vehicle_model_double_track_cpp/numerical.hpp"
#include "vehicle_dynamics_double_track_cpp/vehicle_dynamics_model.hpp"
#include "vehicle_dynamics_helpers_cpp/aerodynamics.hpp"
#include "vehicle_dynamics_helpers_cpp/steering_actuator.hpp"
#include "vehicle_dynamics_helpers_cpp/time_delay.hpp"
#include "vehicle_model_base_cpp/vehicle_model_base.h"
namespace tam::sim
{
class VehicleModelDoubleTrack : public tam::interfaces::VehicleModelBase
{
  using VD_MODEL_T = vd_double_track::VehicleDynamicsDoubleTrackModel;
  using DT_MODEL_T = drivetrain::DriveTrainModelSimple;
  using SA_MODEL_T = helpers::steering_actuator::SteeringActuatorModel;

private:
  uint64_t internal_time_us_{0};  // Internal time -> needed for time delays
  // Delay blocks
  helpers::time_delay::TimeDelay<double> db_steering;
  helpers::time_delay::TimeDelay<double> db_accel;
  // The virtual input where the delayed signals are written to
  tam::types::DriverInput driver_input_delayed;

  // region internal models
  VD_MODEL_T vd_model;
  DT_MODEL_T drivetrain_model;
  SA_MODEL_T steering_actuator;
  //

  // region state vector management
  static const size_t OFFSET_VD = 0;
  static const size_t OFFSET_DT = VD_MODEL_T::state_vector_length;
  static const size_t OFFSET_SA = OFFSET_DT + DT_MODEL_T::state_vector_length;

  static const size_t LEN_STATE_VECTOR = VD_MODEL_T::state_vector_length +
                                         DT_MODEL_T::state_vector_length +
                                         SA_MODEL_T::state_vector_length;
  // endregion
  // typedef for simplication
  typedef Eigen::Matrix<double, LEN_STATE_VECTOR, 1> state_vector_t;

  // region input and outputs
  tam::interfaces::ParamManagerBase::SharedPtr param_manager_;
  tam::interfaces::ParamManagerBase::SharedPtr param_manager_composed_;
  tam::types::DriverInput driver_input;

  tam::types::ExternalInfluences external_influences;
  tam::types::VehicleModelOutput model_output;
  // debugging
  tam::types::common::TUMDebugContainer::SharedPtr debug_container_ =
    std::make_shared<tam::types::common::TUMDebugContainer>();
  Eigen::Matrix<double, LEN_STATE_VECTOR, 1> x;

  // region functions for structring
  void declare_parameters();
  void update_delay_blocks();
  void set_state_vector_of_models(state_vector_t x_);
  state_vector_t ode(double t, state_vector_t x_);

public:
  VehicleModelDoubleTrack();

  // Model step function
  void step() override;

  // Setter
  void set_driver_input(const tam::types::DriverInput & input) override;
  void set_external_influences(const tam::types::ExternalInfluences & input) override;

  // Vehicle dynamic state:
  tam::types::VehicleModelOutput get_output() const override;

  // Debug Output
  tam::types::common::TUMDebugContainer::SharedPtr get_debug_out() const override;

  // Parameter handling
  tam::interfaces::ParamManagerBase * get_param_manager() override;

  void reset();
};
}  // namespace tam::sim
