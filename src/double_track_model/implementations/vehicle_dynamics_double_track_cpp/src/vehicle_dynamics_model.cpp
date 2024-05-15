// Copyright 2023 Simon Sagmeister

#include "vehicle_dynamics_double_track_cpp/vehicle_dynamics_model.hpp"
namespace tam::sim::vd_double_track
{
VehicleDynamicsDoubleTrackModel::VehicleDynamicsDoubleTrackModel() { declare_parameters(); }
void VehicleDynamicsDoubleTrackModel::set_u(const Input & u_)
{
  eqns.u.omega_wheel_radps.front_left = std::max(0.0, u_.omega_wheel_radps.front_left);
  eqns.u.omega_wheel_radps.front_right = std::max(0.0, u_.omega_wheel_radps.front_right);
  eqns.u.omega_wheel_radps.rear_left = std::max(0.0, u_.omega_wheel_radps.rear_left);
  eqns.u.omega_wheel_radps.rear_right = std::max(0.0, u_.omega_wheel_radps.rear_right);
  eqns.u.steering_angle_tire_rad = u_.steering_angle_tire_rad;
}
void VehicleDynamicsDoubleTrackModel::evaluate()
{
  eqns.evaluate();
  assign_debug_outputs();
}
void VehicleDynamicsDoubleTrackModel::add_Vector3D_to_map(
  tam::types::common::TUMDebugContainer::SharedPtr container, const std::string & debug_signal_name,
  const tam::types::common::Vector3D<double> data)
{
  container->log(debug_signal_name + "/x", data.x);
  container->log(debug_signal_name + "/y", data.y);
  container->log(debug_signal_name + "/z", data.z);
}
void VehicleDynamicsDoubleTrackModel::add_per_wheel_data_to_map(
  tam::types::common::TUMDebugContainer::SharedPtr container, const std::string & debug_signal_name,
  const double_per_wheel_t data)
{
  container->log(debug_signal_name + "/front_left", data.front_left);
  container->log(debug_signal_name + "/front_right", data.front_right);
  container->log(debug_signal_name + "/rear_left", data.rear_left);
  container->log(debug_signal_name + "/rear_right", data.rear_right);
}
void VehicleDynamicsDoubleTrackModel::add_per_wheel_data_to_map(
  tam::types::common::TUMDebugContainer::SharedPtr container, const std::string & debug_signal_name,
  const double_vector_per_wheel_t data)
{
  container->log(debug_signal_name + "/front_left/longitudinal", data.front_left[0]);
  container->log(debug_signal_name + "/front_right/longitudinal", data.front_right[0]);
  container->log(debug_signal_name + "/rear_left/longitudinal", data.rear_left[0]);
  container->log(debug_signal_name + "/rear_right/longitudinal", data.rear_right[0]);
  container->log(debug_signal_name + "/front_left/lateral", data.front_left[1]);
  container->log(debug_signal_name + "/front_right/lateral", data.front_right[1]);
  container->log(debug_signal_name + "/rear_left/lateral", data.rear_left[1]);
  container->log(debug_signal_name + "/rear_right/lateral", data.rear_right[1]);
}
void VehicleDynamicsDoubleTrackModel::assign_debug_outputs()
{
  // Assign the debug outputs correctly
  for (int i = 0; i < x::CNT_LENGTH_STATE_VECTOR; i++) {
    debug_container->log("x_vec/" + std::string(vd_double_track::x::TO_STRING(i)), eqns.x_vec[i]);
    debug_container->log(
      "x_dot_vec/" + std::string(vd_double_track::x::TO_STRING(i)), eqns.x_dot_vec[i]);
  }
  add_Vector3D_to_map(
    debug_container, "external_influences/force_cog_N", eqns.w.external_forces_cog_N);
  add_Vector3D_to_map(
    debug_container, "external_influences/moments_cog_Nm", eqns.w.external_moments_cog_Nm);
  add_per_wheel_data_to_map(
    debug_container, "imr/tire_spring_initial_compression_m",
    eqns.imr.tire_spring_initial_compression_m);
  add_per_wheel_data_to_map(
    debug_container, "imr/tire_spring_force_N", eqns.imr.tire_spring_force_N);
  add_per_wheel_data_to_map(debug_container, "imr/F_z_tire_N", eqns.imr.F_z_tire_N);
  add_per_wheel_data_to_map(
    debug_container, "imr/antiroll_bar_force_N", eqns.imr.antiroll_bar_force_N);
  add_per_wheel_data_to_map(
    debug_container, "imr/velocity_wheel_over_ground_mps", eqns.imr.velocity_wheel_over_ground_mps);
  add_per_wheel_data_to_map(
    debug_container, "imr/velocity_wheel_over_ground_tire_frame_mps",
    eqns.imr.velocity_wheel_over_ground_tire_frame_mps);
  add_per_wheel_data_to_map(
    debug_container, "imr/velocity_tire_rotation_mps", eqns.imr.velocity_tire_rotation_mps);
  add_per_wheel_data_to_map(
    debug_container, "imr/tire_longitudinal_slip", eqns.imr.tire_longitudinal_slip);
  add_per_wheel_data_to_map(
    debug_container, "imr/tire_slip_angle_rad", eqns.imr.tire_slip_angle_rad);
  add_per_wheel_data_to_map(
    debug_container, "imr/tire_forces_tire_frame_N", eqns.imr.tire_forces_tire_frame_N);
  add_per_wheel_data_to_map(debug_container, "imr/tire_forces_N", eqns.imr.tire_forces_N);
  add_per_wheel_data_to_map(
    debug_container, "imr/rolling_resistance_N", eqns.imr.tire_rolling_resistance_N);
  add_per_wheel_data_to_map(
    debug_container, "imr/suspension_spring_initial_compression_m",
    eqns.imr.suspension_spring_initial_compression_m);
  add_per_wheel_data_to_map(
    debug_container, "imr/suspension_spring_compression_m",
    eqns.imr.suspension_spring_compression_m);
  add_per_wheel_data_to_map(
    debug_container, "imr/suspension_damper_compression_speed_mps",
    eqns.imr.suspension_damper_compression_speed_mps);
  add_per_wheel_data_to_map(
    debug_container, "imr/suspension_spring_force_N", eqns.imr.suspension_spring_force_N);
  add_per_wheel_data_to_map(
    debug_container, "imr/suspension_damper_force_N", eqns.imr.suspension_damper_force_N);
  add_per_wheel_data_to_map(
    debug_container, "imr/axle_vertical_force_N", eqns.imr.axle_vertical_force_N);
  add_per_wheel_data_to_map(
    debug_container, "imr/resulting_suspension_force_N", eqns.imr.resulting_suspension_force_N);
  add_per_wheel_data_to_map(
    debug_container, "imr/resulting_vertical_force_on_wheel_N",
    eqns.imr.resulting_vertical_force_on_wheel_N);
  add_Vector3D_to_map(debug_container, "imr/resulting_force_N", eqns.imr.resulting_force_N);
  add_Vector3D_to_map(debug_container, "imr/resulting_torque_Nm", eqns.imr.resulting_torque_Nm);
}
void VehicleDynamicsDoubleTrackModel::declare_parameters()
{
  auto p_def_d =
    [this](std::string param_name, const double & initial_value, double * storage_location) {
      param_manager->declare_parameter_non_owning(
        std::string("vehicle_dynamics_double_track.") + param_name, initial_value,
        tam::types::param::ParameterType::DOUBLE, "", storage_location);
    };
  // Masses & Inertial moments
  p_def_d("m", 800, &eqns.p.m);         // [kg]
  p_def_d("m_t_f", 15, &eqns.p.m_t_f);  // [kg]
  p_def_d("m_t_r", 15, &eqns.p.m_t_r);  // [kg]
  p_def_d("I_z", 1000, &eqns.p.I_z);    // [kg m^2]
  p_def_d("I_y", 500, &eqns.p.I_y);     // [kg m^2]
  p_def_d("I_x", 100, &eqns.p.I_x);      // [kg m^2]

  // Geometric parameters
  p_def_d("l_f", 1.8, &eqns.p.l_f);        // [m]
  p_def_d("l", 3.2, &eqns.p.l);            // [m]
  p_def_d("b_f", 1.6, &eqns.p.b_f);        // [m]
  p_def_d("b_r", 1.5, &eqns.p.b_r);        // [m]
  p_def_d("rr_w_f", 0.3, &eqns.p.rr_w_f);  // [m]
  p_def_d("rr_w_r", 0.3, &eqns.p.rr_w_r);  // [m]
  // Simulink currently assumes that the cog lies in the axle plane
  p_def_d("h_cg", 0.3, &eqns.p.h_cg);              //  [m]
  p_def_d("h_pc_accel", 0.2, &eqns.p.h_pc_accel);  //  [m]
  p_def_d("h_pc_decel", 0.2, &eqns.p.h_pc_decel);  //  [m]
  p_def_d("h_rc", 0.2, &eqns.p.h_rc);              //  [m]

  // Spring & Damping constants
  p_def_d("c_f", 120000, &eqns.p.c_f);          // [N/m]
  p_def_d("c_r", 90000.0, &eqns.p.c_r);          // [N/m]
  p_def_d("d_f", 1300.0, &eqns.p.d_f);       // [N*s/m]
  p_def_d("d_r", 900.0, &eqns.p.d_r);     // [N*s/m]
  p_def_d("c_t_f", 250000.0, &eqns.p.c_t_f);  // [N/m]
  p_def_d("c_t_r", 250000.0, &eqns.p.c_t_r);  // [N/m]

  // Anti-Roll bar
  p_def_d("c_ar_f", 12000, &eqns.p.c_ar_f);  // [N/m]
  p_def_d("c_ar_r", 5000, &eqns.p.c_ar_r);                               // [N/m]

  // Rolling resistance coefficient
  p_def_d("c_rr", 0.025, &eqns.p.c_rr);

  // Parameters for Tire Model
  tam::sim::helpers::tire_models::declare_tire_model_parameters(
    "vehicle_dynamics_double_track.", &eqns.p.tire, param_manager.get());
}
}  // namespace tam::sim::vd_double_track
