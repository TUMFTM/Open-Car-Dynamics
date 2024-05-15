// Copyright 2023 Simon Sagmeister

#include "simple_drivetrain_cpp/drivetrain_simple_eqns.hpp"
namespace tam::sim::drivetrain
{
double DriveTrainEquationsSimple::calc_I_wheel_low_speed_per_tire_kgm2(double omega_tire_radps)
{
  return std::max(0.0, 15 * p.I_wheel_rear_kgm2 * (1 - omega_tire_radps / 20));
}
void DriveTrainEquationsSimple::calc_I_wheel_low_speed_kgm2()
{
  I_wheel_low_speed_kgm2.front_left =
    calc_I_wheel_low_speed_per_tire_kgm2(x_vec[x::omega_FL_radps]);
  I_wheel_low_speed_kgm2.front_right =
    calc_I_wheel_low_speed_per_tire_kgm2(x_vec[x::omega_FR_radps]);
  I_wheel_low_speed_kgm2.rear_left = calc_I_wheel_low_speed_per_tire_kgm2(x_vec[x::omega_RL_radps]);
  I_wheel_low_speed_kgm2.rear_right =
    calc_I_wheel_low_speed_per_tire_kgm2(x_vec[x::omega_RR_radps]);
}
void DriveTrainEquationsSimple::evaluate()
{
  // Calculate intermediate results
  calc_I_wheel_low_speed_kgm2();


  double target_fx = x_vec[x::a_x] * p.m_vehicle_kg - external_fx;

  double bias = (target_fx > 0) ? p.AccelBias_Front : p.BrakeBias_Front;
  dt_torque_Nm = double_per_wheel_t::from_front_and_rear(bias, 1 - bias) * 0.5 *
                 double_per_wheel_t::from_front_and_rear(p.rr_tire_front, p.rr_tire_rear) *
                 target_fx;
  auto wheel_inertias =
    double_per_wheel_t::from_front_and_rear(p.I_wheel_front_kgm2, p.I_wheel_rear_kgm2) +
    I_wheel_low_speed_kgm2;

  for (std::size_t i = x::omega_FL_radps; i <= x::omega_RR_radps; i++) {
    x_dot_vec[i] = (dt_torque_Nm[i] - u.tire_torque_rotational_Nm[i]) / (wheel_inertias[i]);
  }
  // Do not allow negative wheel speeds.
  for (std::size_t i = x::omega_FL_radps; i <= x::omega_RR_radps; i++) {
    if (x_vec[i] < 0.00001) {
      x_dot_vec[i] = std::max(0.0, x_dot_vec[i]);
    }
  }
  x_dot_vec[x::a_x] = (u.ax_target - x_vec[x::a_x]) / p.T_PT1_accel;
}
}  // namespace tam::sim::drivetrain
