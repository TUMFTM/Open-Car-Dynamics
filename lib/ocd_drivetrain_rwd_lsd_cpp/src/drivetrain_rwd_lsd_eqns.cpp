// Copyright 2026 Simon Sagmeister

#include "ocd_drivetrain_rwd_lsd_cpp/drivetrain_rwd_lsd_eqns.hpp"
namespace tam::ocd::drivetrain
{
void DrivetrainEquations_RWD_LSD::calc_I_wheel_low_speed_kgm2()
{
  double omega_mean =
    (x_vec_[x::omega_FL_radps] + x_vec_[x::omega_FR_radps] + 2 * x_vec_[x::omega_rear_axle_radps]) /
    4;
  I_wheel_low_speed_kgm2 = std::max(0.0, 15 * p.I_wheel_rear_kgm2 * (1 - omega_mean / 20));
}
void DrivetrainEquations_RWD_LSD::calc_LSD_torque_Nm()
{
  // J. Looman, Zahnradgetriebe, p. 409
  double sigma_diff = std::tanh(x_vec_[x::omega_diff_rear_radps]);
  double ratio =
    drivetrain_input_.transmission_output_torque_Nm > 0 ? p.LSD_lock_drive : p.LSD_lock_coast;
  double lsd_torque_friction_Nm = ratio * drivetrain_input_.transmission_output_torque_Nm;
  M_LSD_Nm = lsd_torque_friction_Nm > p.LSD_preload ? sigma_diff * lsd_torque_friction_Nm
                                                    : sigma_diff * p.LSD_preload;
}
void DrivetrainEquations_RWD_LSD::evaluate()
{
  // Calculate intermediate results
  calc_I_wheel_low_speed_kgm2();
  calc_LSD_torque_Nm();

  double I_rear_reduced_kgm2 =
    2 * p.I_wheel_rear_kgm2 + drivetrain_input_.current_engine_inertia_at_wheels_kgm2;

  // Fill in the x_dot vectpr
  x_dot_vec_[x::omega_FL_radps] = (-drivetrain_load_torque_per_wheel_Nm_.front_left -
                                   drivetrain_input_.brake_torque_per_wheel_Nm.front_left) /
                                  (p.I_wheel_front_kgm2 + I_wheel_low_speed_kgm2);
  x_dot_vec_[x::omega_FR_radps] = (-drivetrain_load_torque_per_wheel_Nm_.front_right -
                                   drivetrain_input_.brake_torque_per_wheel_Nm.front_right) /
                                  (p.I_wheel_front_kgm2 + I_wheel_low_speed_kgm2);

  x_dot_vec_[x::omega_rear_axle_radps] = (drivetrain_input_.transmission_output_torque_Nm -
                                          drivetrain_load_torque_per_wheel_Nm_.rear_left -
                                          drivetrain_load_torque_per_wheel_Nm_.rear_right -
                                          drivetrain_input_.brake_torque_per_wheel_Nm.rear_left -
                                          drivetrain_input_.brake_torque_per_wheel_Nm.rear_right) /
                                         (I_rear_reduced_kgm2 + 2 * I_wheel_low_speed_kgm2);
  x_dot_vec_[x::omega_diff_rear_radps] =
    (-2 * M_LSD_Nm - drivetrain_load_torque_per_wheel_Nm_.rear_left -
     drivetrain_input_.brake_torque_per_wheel_Nm.rear_left +
     drivetrain_load_torque_per_wheel_Nm_.rear_right +
     drivetrain_input_.brake_torque_per_wheel_Nm.rear_right) /
    (2 * (p.I_wheel_rear_kgm2 + I_wheel_low_speed_kgm2));

  // Do not allow negative wheel speeds.
  for (std::size_t i = x::omega_FL_radps; i <= x::omega_rear_axle_radps; i++) {
    if (x_vec_[i] < 0.00001) {
      x_dot_vec_[i] = std::max(0.0, x_dot_vec_[i]);
    }
  }
  if (x_vec_[x::omega_rear_axle_radps] + x_vec_[x::omega_diff_rear_radps] < 0.00001) {
    x_dot_vec_[x::omega_diff_rear_radps] = std::max(0.0, x_dot_vec_[x::omega_diff_rear_radps]);
  }
  if (x_vec_[x::omega_rear_axle_radps] - x_vec_[x::omega_diff_rear_radps] < 0.00001) {
    x_dot_vec_[x::omega_diff_rear_radps] = std::min(0.0, x_dot_vec_[x::omega_diff_rear_radps]);
  }
}
}  // namespace tam::ocd::drivetrain
