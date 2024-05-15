// Copyright 2023 Simon Sagmeister

#include "simple_drivetrain_cpp/drivetrain_simple_model.hpp"
namespace tam::sim::drivetrain
{
DriveTrainModelSimple::DriveTrainModelSimple() { declare_parameters(); }
void DriveTrainModelSimple::assign_debug_outputs()
{
  debug_container->log("u/tire_force_torque_per_wheel_Nm", eqns.u.tire_torque_rotational_Nm);
  debug_container->log("external_influences/force_x", eqns.external_fx);
  debug_container->log("dt_torque_per_wheel_Nm", eqns.dt_torque_Nm);
  // Loop over state vec
  auto x_vec = eqns.x_vec;
  auto x_dot_vec = eqns.x_dot_vec;
  for (int i = 0; i < x::CNT_LENGTH_STATE_VECTOR; i++) {
    debug_container->log("x_vec/" + std::string(x::TO_STRING(i)), x_vec[i]);
    debug_container->log("x_dot_vec/" + std::string(x::TO_STRING(i)), x_dot_vec[i]);
  }
}
Output DriveTrainModelSimple::get_y()
{
  Output out;
  out.omega_wheel_radps.front_left = eqns.x_vec[x::omega_FL_radps];
  out.omega_wheel_radps.front_right = eqns.x_vec[x::omega_FR_radps];
  out.omega_wheel_radps.rear_left = eqns.x_vec[x::omega_RL_radps];
  out.omega_wheel_radps.rear_right = eqns.x_vec[x::omega_RR_radps];
  return out;
}
void DriveTrainModelSimple::declare_parameters()
{
  auto p_def_d =
    [this](std::string param_name, const double & initial_value, double * storage_location) {
      param_manager->declare_parameter_non_owning(
        std::string("drivetrain.") + param_name, initial_value,
        tam::types::param::ParameterType::DOUBLE, "", storage_location);
    };

  p_def_d("I_wheel_front_kgm2", 1.2, &eqns.p.I_wheel_front_kgm2);
  p_def_d("I_wheel_rear_kgm2", 1.4, &eqns.p.I_wheel_rear_kgm2);
  p_def_d("BrakeBias_Front", 0.55, &eqns.p.BrakeBias_Front);
  p_def_d("AccelBias_Front", 0.0, &eqns.p.AccelBias_Front);
  p_def_d("tire_rolling_radius_front_m", 0.30, &eqns.p.rr_tire_front);
  p_def_d("tire_rolling_radius_rear_m", 0.30, &eqns.p.rr_tire_rear);
  p_def_d("m_vehicle_kg", 800, &eqns.p.m_vehicle_kg);
  p_def_d("T_PT1_accel", 0.030, &eqns.p.T_PT1_accel);
}
void DriveTrainModelSimple::evaluate()
{
  eqns.evaluate();
  assign_debug_outputs();
}
}  // namespace tam::sim::drivetrain

