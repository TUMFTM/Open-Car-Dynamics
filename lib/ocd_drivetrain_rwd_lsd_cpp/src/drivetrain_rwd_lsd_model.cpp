// Copyright 2026 Simon Sagmeister

#include "ocd_drivetrain_rwd_lsd_cpp/drivetrain_rwd_lsd_model.hpp"
namespace tam::ocd::drivetrain
{
void DrivetrainModel_RWD_LSD::set_x_vec(const StateVectorType & x_vec) { eqns_.set_x_vec(x_vec); }
void DrivetrainModel_RWD_LSD::set_driver_input(const DriverInputType & input)
{
  eqns_.set_driver_input(input);
}
void DrivetrainModel_RWD_LSD::set_load(
  const double_per_wheel_t & drivetrain_load_torque_per_wheel_Nm)
{
  eqns_.set_load(drivetrain_load_torque_per_wheel_Nm);
};
DrivetrainModel_RWD_LSD::StateVectorType DrivetrainModel_RWD_LSD::get_x_vec() const
{
  return eqns_.x_vec_;
}
DrivetrainModel_RWD_LSD::StateVectorType DrivetrainModel_RWD_LSD::get_x_dot_vec() const
{
  return eqns_.x_dot_vec_;
}
tam::tsl::LoggerAccessInterface::SharedPtr DrivetrainModel_RWD_LSD::get_logger() const
{
  return logger_;
}
tam::pmg::MgmtInterface::SharedPtr DrivetrainModel_RWD_LSD::get_param_manager() const
{
  return param_manager_;
}
DrivetrainModel_RWD_LSD::DrivetrainModel_RWD_LSD()
{
  declare_parameters();
  register_log_signals();
}
DrivetrainModel_RWD_LSD::double_per_wheel_t DrivetrainModel_RWD_LSD::get_wheel_speeds() const
{
  double_per_wheel_t out;
  out.front_left = eqns_.x_vec_[x::omega_FL_radps];
  out.front_right = eqns_.x_vec_[x::omega_FR_radps];
  out.rear_left = eqns_.x_vec_[x::omega_rear_axle_radps] + eqns_.x_vec_[x::omega_diff_rear_radps];
  out.rear_right = eqns_.x_vec_[x::omega_rear_axle_radps] - eqns_.x_vec_[x::omega_diff_rear_radps];
  return out;
}
DrivetrainModel_RWD_LSD::FeedbackType DrivetrainModel_RWD_LSD::get_feedback() const {}
void DrivetrainModel_RWD_LSD::declare_parameters()
{
  auto p_def_d =
    [this](std::string param_name, const double & initial_value, double * storage_location) {
      param_manager_->declare_parameter(
        std::string("drivetrain.") + param_name, storage_location, initial_value,
        tam::pmg::ParameterType::DOUBLE, "");
    };

  p_def_d("I_wheel_front_kgm2", 1.15, &eqns_.p.I_wheel_front_kgm2);
  p_def_d("I_wheel_rear_kgm2", 1.35, &eqns_.p.I_wheel_rear_kgm2);
  p_def_d("transmission_ratio", 3.0, &eqns_.p.final_drive_ratio);
  p_def_d("LSD_preload", 0.0, &eqns_.p.LSD_preload);
  p_def_d("LSD_lock_drive", 0.0, &eqns_.p.LSD_lock_drive);
  p_def_d("LSD_lock_coast", 0.0, &eqns_.p.LSD_lock_coast);
}
void DrivetrainModel_RWD_LSD::register_log_signals()
{
  logger_->log("drivetrain_load_torque_per_wheel_Nm", &eqns_.drivetrain_load_torque_per_wheel_Nm_);
  logger_->log("M_LSD_Nm", &eqns_.M_LSD_Nm);
  logger_->log(
    "input/brake_torque_per_wheel_Nm", &eqns_.drivetrain_input_.brake_torque_per_wheel_Nm);
  logger_->log(
    "input/current_engine_inertia_at_wheels_kgm2",
    &eqns_.drivetrain_input_.current_engine_inertia_at_wheels_kgm2);
  logger_->log(
    "input/transmission_output_torque_Nm", &eqns_.drivetrain_input_.transmission_output_torque_Nm);
  // Loop over state vec
  for (int i = 0; i < x::CNT_LENGTH_STATE_VECTOR; i++) {
    logger_->log("x_vec/" + std::string(StateNamesTrait::value[i]), &eqns_.x_vec_[i]);
    logger_->log("x_dot_vec/" + std::string(StateNamesTrait::value[i]), &eqns_.x_dot_vec_[i]);
  }
}
void DrivetrainModel_RWD_LSD::evaluate() { eqns_.evaluate(); }
}  // namespace tam::ocd::drivetrain
