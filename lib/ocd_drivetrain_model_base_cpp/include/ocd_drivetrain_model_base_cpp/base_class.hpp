// Copyright 2026 Simon Sagmeister

#pragma once
#include <concepts>
#include <eigen3/Eigen/Dense>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

#include "ocd_drivetrain_model_base_cpp/conditional_auxiliary_input.hpp"
#include "ocd_types_cpp/types.hpp"
#include "param_management_cpp/base.hpp"
#include "tsl_logger_cpp/base.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::ocd::interfaces
{
template <
  typename DriverInputT, typename FeedbackT, std::size_t state_vector_length, typename StateNamesT,
  typename AuxiliaryInputT = void>
class DrivetrainModelBase
// Derive from a helper struct that adds a function only if AuxiliaryInputT is not void
: public auxiliary::conditional_compilation::AuxiliaryInputHelperDT<AuxiliaryInputT>
{
public:
  // Declare the types transpararently so they can be accessed from outside
  using DriverInputType = DriverInputT;
  using FeedbackType = FeedbackT;
  using StateNamesTrait = StateNamesT;
  using StateVectorType = Eigen::Matrix<double, state_vector_length, 1>;
  using AuxiliaryInputType = AuxiliaryInputT;
  // Declare public constants for describing the state vector
  static constexpr std::size_t k_state_vector_length = state_vector_length;

public:
  DrivetrainModelBase() = default;
  virtual ~DrivetrainModelBase() = default;

  // setters
  // virtual void set_auxiliary_input(const AuxiliaryInputT & value) = 0; | INHERITED FROM HELPER
  virtual void set_x_vec(const StateVectorType & x_vec) = 0;
  virtual void set_driver_input(const DriverInputT & input) = 0;
  virtual void set_load(
    const tam::types::common::DataPerWheel<double> & drivetrain_load_torque_per_wheel_Nm) = 0;

  // Model step function
  virtual void evaluate() = 0;

  // getters
  virtual tam::types::common::DataPerWheel<double> get_wheel_speeds() const = 0;
  virtual FeedbackT get_feedback() const = 0;
  virtual StateVectorType get_x_vec() const = 0;
  virtual StateVectorType get_x_dot_vec() const = 0;

  virtual tam::tsl::LoggerAccessInterface::SharedPtr get_logger() const = 0;
  virtual tam::pmg::MgmtInterface::SharedPtr get_param_manager() const = 0;
  // Function to convert state enum to string
};
}  // namespace tam::ocd::interfaces
