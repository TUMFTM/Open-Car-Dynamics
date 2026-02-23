// Copyright 2026 Simon Sagmeister

#pragma once

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "ocd_steering_actuator_model_base_cpp/base_class.hpp"
#include "ocd_steering_actuator_model_base_cpp/concept.hpp"
#include "ocd_steering_actuator_pt1_cpp/states.hpp"
#include "param_management_cpp/base.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::ocd::steering_actuator
{
class PT1SteeringActuatorModel
: public tam::ocd::interfaces::SteeringActuatorModelBase<
    PT1::DriverInput, PT1::Feedback, PT1::States::StateEnum::CNT_LENGTH_STATE_VECTOR,
    PT1::States::StateNames>
{
  using x = tam::ocd::steering_actuator::PT1::States::StateEnum;
  using DoublePerWheelType = tam::types::common::DataPerWheel<double>;

public:
  PT1SteeringActuatorModel();
  // setters
  void set_x_vec(const StateVectorType & x_vec) override;
  void set_driver_input(const DriverInputType & input) override;
  void set_load(const DoublePerWheelType & steering_load_torque_per_wheel_Nm) override;
  // Model step function
  void evaluate() override;

  // getters
  DoublePerWheelType get_steering_angles() const override;
  FeedbackType get_feedback() const override;
  StateVectorType get_x_vec() const override;
  StateVectorType get_x_dot_vec() const override;
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() const override;
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const override;

private:
  struct Parameters
  {
    double T_PT1, angle_max_rad, angle_rate_max_radps, static_offset_rad;
  } p_;
  StateVectorType x_vec_;
  StateVectorType x_dot_vec_;
  double requested_steering_angle_rad_;
  double measured_steering_angle_rad_;
  double actual_steering_angle_rad_;
  DoublePerWheelType steering_load_torque_per_wheel_Nm_;

  tam::tsl::ReferenceLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ReferenceLogger>();
  tam::pmg::ParamReferenceManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamReferenceManager>();

  void declare_parameters();
  void register_log_signals();
};
// Check that the class fulfills the concept
// This checks if the base class was properly implemented without having to create an instance
static_assert(
  tam::ocd::interfaces::concepts::SteeringActuatorModel<PT1SteeringActuatorModel>,
  "PT1SteeringActuatorModel does not fulfill the SteeringActuatorModel concept");

}  // namespace tam::ocd::steering_actuator
