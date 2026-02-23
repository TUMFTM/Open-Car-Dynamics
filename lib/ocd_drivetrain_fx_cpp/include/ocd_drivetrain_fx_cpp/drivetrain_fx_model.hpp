// Copyright 2026 Simon Sagmeister

#pragma once
#include <math.h>

#include "ocd_drivetrain_model_base_cpp/base_class.hpp"
#include "ocd_drivetrain_model_base_cpp/concept.hpp"
#include "ocd_drivetrain_rwd_lsd_cpp/drivetrain_rwd_lsd_model.hpp"
namespace tam::ocd::drivetrain
{
// Wrapping class around the equations
class DrivetrainFxModel : public tam::ocd::interfaces::DrivetrainModelBase<
                            double, void, DrivetrainModel_RWD_LSD::k_state_vector_length,
                            DrivetrainModel_RWD_LSD::StateNamesTrait>
{
  using double_per_wheel_t = tam::types::common::DataPerWheel<double>;
  struct AdditionalParams
  {
    double brake_bias_front;
    double wheel_radius_m;
    double intertia_kgm2;
  } p_;

public:
  DrivetrainFxModel()
  {
    base_model_.param_manager_->declare_parameter(
      "drivetrain.brake_bias_front", &p_.brake_bias_front, 0.6, tam::pmg::ParameterType::DOUBLE,
      "");
    base_model_.param_manager_->declare_parameter(
      "drivetrain.wheel_radius_m", &p_.wheel_radius_m, 0.3, tam::pmg::ParameterType::DOUBLE, "");
    base_model_.param_manager_->declare_parameter(
      "drivetrain.intertia_kgm2", &p_.intertia_kgm2, 0.0, tam::pmg::ParameterType::DOUBLE, "");
  };
  // setters
  void set_x_vec(const StateVectorType & x_vec) override { base_model_.set_x_vec(x_vec); }
  void set_driver_input(const double & input) override
  {
    DrivetrainModel_RWD_LSD::DriverInputType driver_input;
    driver_input.current_engine_inertia_at_wheels_kgm2 = p_.intertia_kgm2;
    if (input >= 0.0) {
      // Throttle input
      driver_input.transmission_output_torque_Nm = input * p_.wheel_radius_m;
      driver_input.brake_torque_per_wheel_Nm = double_per_wheel_t{0.0};
    } else {
      // Brake input
      driver_input.transmission_output_torque_Nm = 0.0;
      // Distribute brake torque according to brake bias
      double fx_front = -p_.brake_bias_front * input;
      double fx_rear = -(1.0 - p_.brake_bias_front) * input;
      driver_input.brake_torque_per_wheel_Nm = double_per_wheel_t::from_front_and_rear(
        fx_front * p_.wheel_radius_m / 2, fx_rear * p_.wheel_radius_m / 2);
    }
    base_model_.set_driver_input(driver_input);
  }
  void set_load(const double_per_wheel_t & drivetrain_load_torque_per_wheel_Nm) override
  {
    base_model_.set_load(drivetrain_load_torque_per_wheel_Nm);
  }
  // model step function
  void evaluate() { base_model_.evaluate(); }
  // getters
  double_per_wheel_t get_wheel_speeds() const override { return base_model_.get_wheel_speeds(); }
  void get_feedback() const override { base_model_.get_feedback(); }
  StateVectorType get_x_vec() const override { return base_model_.get_x_vec(); }
  StateVectorType get_x_dot_vec() const override { return base_model_.get_x_dot_vec(); }
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() const override
  {
    return base_model_.get_logger();
  }
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const override
  {
    return base_model_.get_param_manager();
  }

private:
  DrivetrainModel_RWD_LSD base_model_;
};
// Check that the class fulfills the concept
// This checks if the base class was properly implemented without having to create an instance
static_assert(
  tam::ocd::interfaces::concepts::DrivetrainModel<DrivetrainFxModel>,
  "DrivetrainFxModel does not fulfill the DrivetrainModel concept");
}  // namespace tam::ocd::drivetrain
