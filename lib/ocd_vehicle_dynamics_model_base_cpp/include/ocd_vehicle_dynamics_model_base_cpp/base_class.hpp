// Copyright 2026 Simon Sagmeister

#pragma once
#include <eigen3/Eigen/Dense>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

#include "ocd_types_cpp/types.hpp"
#include "param_management_cpp/base.hpp"
#include "tsl_logger_cpp/base.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::ocd::interfaces
{
template <std::size_t state_vector_length, typename StateNamesT>
class VehicleDynamicsModelBase
{
public:
  // Declare the types transpararently so they can be accessed from outside
  using StateNamesTrait = StateNamesT;
  using StateVectorType = Eigen::Matrix<double, state_vector_length, 1>;
  // Declare public constants for describing the state vector
  static constexpr std::size_t k_state_vector_length = state_vector_length;

public:
  VehicleDynamicsModelBase() = default;
  virtual ~VehicleDynamicsModelBase() = default;

  // setters
  virtual void set_x_vec(const StateVectorType & x_vec) = 0;
  virtual void set_steering_angles(
    const tam::types::common::DataPerWheel<double> & steering_angle_per_wheel_rad) = 0;
  virtual void set_wheel_speeds(
    const tam::types::common::DataPerWheel<double> & wheel_speeds_per_wheel_radps) = 0;
  virtual void set_external_influences(const types::ExternalInfluences & external_influences) = 0;

  // Model step function
  virtual void evaluate() = 0;

  // getters
  virtual tam::types::common::DataPerWheel<double> get_steering_load() const = 0;
  virtual tam::types::common::DataPerWheel<double> get_wheel_load() const = 0;
  virtual types::VehicleDynamicsModelOutput get_vehicle_dynamics_output() const = 0;
  virtual StateVectorType get_x_vec() const = 0;
  virtual StateVectorType get_x_dot_vec() const = 0;

  virtual tam::tsl::LoggerAccessInterface::SharedPtr get_logger() const = 0;
  virtual tam::pmg::MgmtInterface::SharedPtr get_param_manager() const = 0;
  // Function to convert state enum to string
};
}  // namespace tam::ocd::interfaces
