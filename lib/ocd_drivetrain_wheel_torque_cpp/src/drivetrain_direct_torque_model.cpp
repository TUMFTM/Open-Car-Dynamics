// Copyright 2026 Simon Sagmeister

#include "ocd_drivetrain_wheel_torque_cpp/drivetrain_direct_torque_model.hpp"
namespace tam::ocd::drivetrain
{
void DrivetrainWheelTorqueModel::set_x_vec(const StateVectorType & x_vec)
{
  eqns_.set_x_vec(x_vec);
}
void DrivetrainWheelTorqueModel::set_driver_input(const DriverInputType & input)
{
  eqns_.set_driver_input(input);
}
void DrivetrainWheelTorqueModel::set_load(
  const double_per_wheel_t & drivetrain_load_torque_per_wheel_Nm)
{
  eqns_.set_load(drivetrain_load_torque_per_wheel_Nm);
};
// getters
DrivetrainWheelTorqueModel::StateVectorType DrivetrainWheelTorqueModel::get_x_vec() const
{
  return eqns_.x_vec_;
}
DrivetrainWheelTorqueModel::StateVectorType DrivetrainWheelTorqueModel::get_x_dot_vec() const
{
  return eqns_.x_dot_vec_;
}
tam::tsl::LoggerAccessInterface::SharedPtr DrivetrainWheelTorqueModel::get_logger() const
{
  return logger_;
}
tam::pmg::MgmtInterface::SharedPtr DrivetrainWheelTorqueModel::get_param_manager() const
{
  return param_manager_;
}
DrivetrainWheelTorqueModel::DrivetrainWheelTorqueModel()
{
  declare_parameters();
  register_log_signals();
}
DrivetrainWheelTorqueModel::double_per_wheel_t DrivetrainWheelTorqueModel::get_wheel_speeds() const
{
  double_per_wheel_t out;
  out.front_left = eqns_.x_vec_[x::omega_FL_radps];
  out.front_right = eqns_.x_vec_[x::omega_FR_radps];
  out.rear_left = eqns_.x_vec_[x::omega_RL_radps];
  out.rear_right = eqns_.x_vec_[x::omega_RR_radps];
  return out;
}
DrivetrainWheelTorqueModel::FeedbackType DrivetrainWheelTorqueModel::get_feedback() const
{
  FeedbackType out;
  out.drivetrain_input_torque_per_wheel_Nm =
    eqns_.driver_input_.drivetrain_input_torque_per_wheel_Nm;
  return out;
}
void DrivetrainWheelTorqueModel::declare_parameters()
{
  auto p_def_d =
    [this](std::string param_name, const double & initial_value, double * storage_location) {
      param_manager_->declare_parameter(
        std::string("drivetrain.") + param_name, storage_location, initial_value,
        tam::pmg::ParameterType::DOUBLE, "");
    };
  p_def_d("I_engine_kgm2", 2.0, &eqns_.p.I_engine_kgm2);
  p_def_d("I_wheel_front_kgm2", 1.15, &eqns_.p.I_wheel_front_kgm2);
  p_def_d("I_wheel_rear_kgm2", 1.35, &eqns_.p.I_wheel_rear_kgm2);
}
void DrivetrainWheelTorqueModel::register_log_signals()
{
  logger_->log("drivetrain_load_torque_per_wheel_Nm_", &eqns_.drivetrain_load_torque_per_wheel_Nm_);
  logger_->log(
    "drivetrain_input_torque_per_wheel_Nm",
    &eqns_.driver_input_.drivetrain_input_torque_per_wheel_Nm);
  logger_->log("imr/I_wheel_low_speed_kgm2", &eqns_.I_wheel_low_speed_kgm2);
  logger_->log("p/I_engine_kgm2", &eqns_.p.I_engine_kgm2);
  logger_->log("p/I_wheel_front_kgm2", &eqns_.p.I_wheel_front_kgm2);
  logger_->log("p/I_wheel_rear_kgm2", &eqns_.p.I_wheel_rear_kgm2);
  logger_->log("imr/bool_clamp_xdot_to_avoid_reverse", &eqns_.bool_clamp_xdot_to_avoid_reverse);
  // Loop over state vec
  auto x_vec_ = eqns_.x_vec_;
  auto x_dot_vec_ = eqns_.x_dot_vec_;
  for (int i = 0; i < x::CNT_LENGTH_STATE_VECTOR; i++) {
    logger_->log("x_vec/" + std::string(StateNamesTrait::value[i]), &x_vec_[i]);
    logger_->log("x_dot_vec/" + std::string(StateNamesTrait::value[i]), &x_dot_vec_[i]);
  }
}
void DrivetrainWheelTorqueModel::evaluate() { eqns_.evaluate(); }
}  // namespace tam::ocd::drivetrain
