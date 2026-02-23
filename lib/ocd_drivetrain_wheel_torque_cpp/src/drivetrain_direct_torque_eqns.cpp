// Copyright 2026 Simon Sagmeister

#include "ocd_drivetrain_wheel_torque_cpp/drivetrain_direct_torque_eqns.hpp"
namespace tam::ocd::drivetrain
{
void DrivetrainEquationsDirectTorque::calc_I_wheel_low_speed_kgm2()
{
  double omega_mean_radps = (x_vec_[x::omega_FL_radps] + x_vec_[x::omega_FR_radps] +
                             x_vec_[x::omega_RL_radps] + x_vec_[x::omega_RR_radps]) /
                            4;
  I_wheel_low_speed_kgm2 = std::max(0.0, 15 * p.I_wheel_rear_kgm2 * (1 - omega_mean_radps / 20));
}
void DrivetrainEquationsDirectTorque::evaluate()
{
  calc_I_wheel_low_speed_kgm2();
  // Fill in the x_dot vector
  x_dot_vec_[x::omega_FL_radps] = (-drivetrain_load_torque_per_wheel_Nm_.front_left +
                                   driver_input_.drivetrain_input_torque_per_wheel_Nm.front_left) /
                                  (p.I_wheel_front_kgm2 + I_wheel_low_speed_kgm2);
  x_dot_vec_[x::omega_FR_radps] = (-drivetrain_load_torque_per_wheel_Nm_.front_right +
                                   driver_input_.drivetrain_input_torque_per_wheel_Nm.front_right) /
                                  (p.I_wheel_front_kgm2 + I_wheel_low_speed_kgm2);
  x_dot_vec_[x::omega_RL_radps] = (-drivetrain_load_torque_per_wheel_Nm_.rear_left +
                                   driver_input_.drivetrain_input_torque_per_wheel_Nm.rear_left) /
                                  (p.I_wheel_rear_kgm2 + I_wheel_low_speed_kgm2);
  x_dot_vec_[x::omega_RR_radps] = (-drivetrain_load_torque_per_wheel_Nm_.rear_right +
                                   driver_input_.drivetrain_input_torque_per_wheel_Nm.rear_right) /
                                  (p.I_wheel_rear_kgm2 + I_wheel_low_speed_kgm2);

  // Do not allow negative wheel speeds.
  bool_clamp_xdot_to_avoid_reverse = false;
  for (std::size_t i = x::omega_FL_radps; i <= x::omega_RR_radps; i++) {
    if (x_vec_[i] < 0.00001) {
      x_dot_vec_[i] = std::max(0.0, x_dot_vec_[i]);
      bool_clamp_xdot_to_avoid_reverse = true;
    }
  }
}
}  // namespace tam::ocd::drivetrain
