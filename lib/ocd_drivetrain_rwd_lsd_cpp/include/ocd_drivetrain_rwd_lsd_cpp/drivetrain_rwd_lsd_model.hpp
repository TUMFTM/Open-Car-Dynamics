// Copyright 2026 Simon Sagmeister

#pragma once
#include <math.h>

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ocd_drivetrain_model_base_cpp/base_class.hpp"
#include "ocd_drivetrain_model_base_cpp/concept.hpp"
#include "ocd_drivetrain_rwd_lsd_cpp/drivetrain_rwd_lsd_eqns.hpp"
#include "param_management_cpp/base.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::ocd::drivetrain
{
// Wrapping class around the equations
class DrivetrainModel_RWD_LSD
: public tam::ocd::interfaces::DrivetrainModelBase<
    drivetrain::DrivetrainEquations_RWD_LSD::DriverInput, void,
    drivetrain::rwd_lsd::States::CNT_LENGTH_STATE_VECTOR, drivetrain::rwd_lsd::States::StateNames>
{
  using double_per_wheel_t = tam::types::common::DataPerWheel<double>;
  using x = tam::ocd::drivetrain::rwd_lsd::States::StateEnum;

  friend class DrivetrainFxModel;  // allow access to private members

public:
  DrivetrainModel_RWD_LSD();
  // setters
  void set_x_vec(const StateVectorType & x_vec) override;
  void set_driver_input(const DriverInputType & input) override;
  void set_load(const double_per_wheel_t & drivetrain_load_torque_per_wheel_Nm) override;
  // model step function
  void evaluate();

  // getters
  double_per_wheel_t get_wheel_speeds() const override;
  FeedbackType get_feedback() const override;
  StateVectorType get_x_vec() const override;
  StateVectorType get_x_dot_vec() const override;
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() const override;
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const override;

private:
  DrivetrainEquations_RWD_LSD eqns_;
  tam::tsl::ReferenceLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ReferenceLogger>();
  pmg::ParamReferenceManager::SharedPtr param_manager_ =
    std::make_shared<pmg::ParamReferenceManager>();
  void declare_parameters();
  void register_log_signals();
};
// Check that the class fulfills the concept
// This checks if the base class was properly implemented without having to create an instance
static_assert(
  tam::ocd::interfaces::concepts::DrivetrainModel<DrivetrainModel_RWD_LSD>,
  "DrivetrainModel_RWD_LSD does not fulfill the DrivetrainModel concept");
}  // namespace tam::ocd::drivetrain
