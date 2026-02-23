// Copyright 2026 Simon Sagmeister
#pragma once

#include <string>
#include <vector>

#include "ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_model.hpp"
#include "tsl_logger_cpp/type_support.hpp"
namespace tam::ocd::vehicle_dynamics
{
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
tam::tsl::LoggerAccessInterface::SharedPtr
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::get_logger() const
{
  return logger_;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
tam::pmg::MgmtInterface::SharedPtr
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::get_param_manager() const
{
  return param_manager_;
}
// Settings inputs
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::set_wheel_speeds(
  const double_per_wheel_t & wheel_speeds_radps)
{
  eqns_.set_wheel_speeds(wheel_speeds_radps);
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::set_steering_angles(
  const double_per_wheel_t & steering_angle_per_wheel_rad)
{
  eqns_.set_steering_angles(steering_angle_per_wheel_rad);
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::set_external_influences(
  const types::ExternalInfluences & external_influences)
{
  eqns_.set_external_influences(external_influences);
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::set_x_vec(
  const StateVectorType & x_vec)
{
  eqns_.set_x_vec(x_vec);
}
// Getting outputs
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::double_per_wheel_t
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::get_wheel_load() const
{
  return eqns_.drivetrain_load_torque_per_wheel_Nm_;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::double_per_wheel_t
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::get_steering_load() const
{
  return eqns_.steering_load_torque_per_wheel_Nm_;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
types::VehicleDynamicsModelOutput
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::get_vehicle_dynamics_output() const
{
  return eqns_.vd_output_;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>

VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::StateVectorType
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::get_x_vec() const
{
  return eqns_.x_vec_;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::StateVectorType
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::get_x_dot_vec() const
{
  return eqns_.x_dot_vec_;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::VehicleDynamicsDoubleTrackModel()
{
  declare_parameters();
  register_log_signals();
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::evaluate()
{
  eqns_.evaluate();
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::register_log_signals()
{
  // Assign the debug outputs correctly
  for (int i = 0; i < x::CNT_LENGTH_STATE_VECTOR; i++) {
    logger_->log("x_vec/" + std::string(StateNamesTrait::value[i]), &eqns_.x_vec_[i]);
    logger_->log("x_dot_vec/" + std::string(StateNamesTrait::value[i]), &eqns_.x_dot_vec_[i]);
  }
  // clang-format off
  logger_->log("external_influences/external_force_N", &eqns_.external_influences_.external_force_N); // NOLINT
  logger_->log("external_influences/external_force_N", &eqns_.external_influences_.external_force_N); // NOLINT
  logger_->log("external_influences/external_torque_Nm", &eqns_.external_influences_.external_torque_Nm); // NOLINT
  logger_->log("external_influences/lambda_mue", &eqns_.external_influences_.lambda_mue);
  logger_->log("external_influences/z_height_road_m", &eqns_.external_influences_.z_height_road_m);
  logger_->log("imr/dynamic_tire_radius", &eqns_.imr_.dynamic_tire_radius_m);
  logger_->log("imr/effective_steering_angle_per_wheel_rad", &eqns_.imr_.effective_steering_angle_per_wheel_rad); // NOLINT
  logger_->log("imr/tire_spring_initial_compression_m", &eqns_.imr_.tire_spring_initial_compression_m); // NOLINT
  logger_->log("imr/tire_spring_force_N", &eqns_.imr_.tire_spring_force_N);
  logger_->log("imr/vertical_tire_force_N", &eqns_.imr_.vertical_tire_force_N);
  logger_->log("imr/antiroll_bar_force_N", &eqns_.imr_.antiroll_bar_force_N);
  logger_->log("imr/velocity_wheel_over_ground_mps", &eqns_.imr_.velocity_wheel_over_ground_mps);
  logger_->log("imr/velocity_wheel_over_ground_tire_frame_mps", &eqns_.imr_.velocity_wheel_over_ground_tire_frame_mps); // NOLINT
  logger_->log("imr/velocity_tire_rotation_mps", &eqns_.imr_.velocity_tire_rotation_mps);
  logger_->log("imr/tire_longitudinal_slip", &eqns_.imr_.tire_longitudinal_slip);
  logger_->log("imr/tire_slip_angle_rad", &eqns_.imr_.tire_slip_angle_rad);
  logger_->log("imr/tire_forces_tire_frame_N", &eqns_.imr_.tire_forces_tire_frame_N);
  logger_->log("imr/tire_forces_N", &eqns_.imr_.tire_forces_N);
  logger_->log("imr/rolling_resistance_N", &eqns_.imr_.tire_rolling_resistance_N);
  logger_->log("imr/suspension_spring_initial_compression_m", &eqns_.imr_.suspension_spring_initial_compression_m); // NOLINT
  logger_->log("imr/suspension_spring_compression_m", &eqns_.imr_.suspension_spring_compression_m);
  logger_->log("imr/suspension_damper_compression_speed_mps", &eqns_.imr_.suspension_damper_compression_speed_mps); // NOLINT
  logger_->log("imr/suspension_spring_force_N", &eqns_.imr_.suspension_spring_force_N);
  logger_->log("imr/suspension_damper_force_N", &eqns_.imr_.suspension_damper_force_N);
  logger_->log("imr/axle_vertical_force_N", &eqns_.imr_.axle_vertical_force_N);
  logger_->log("imr/resulting_suspension_force_N", &eqns_.imr_.resulting_suspension_force_N);
  logger_->log("imr/resulting_vertical_force_on_wheel_N", &eqns_.imr_.resulting_vertical_force_on_wheel_N); // NOLINT
  logger_->log("imr/aero_force_N", &eqns_.imr_.aero_force_N);
  logger_->log("imr/aero_torque_Nm", &eqns_.imr_.aero_torque_Nm);
  logger_->log("imr/resulting_force_N", &eqns_.imr_.resulting_force_N);
  logger_->log("imr/resulting_torque_Nm", &eqns_.imr_.resulting_torque_Nm);
  // clang-format on

  eqns_.aero_model_.register_log_signals(logger_.get(), "imr/aerodynamics/");

  eqns_.tire_models_.front_left.register_log_signals(logger_.get(), "imr/tire_model/front_left/");
  eqns_.tire_models_.front_right.register_log_signals(logger_.get(), "imr/tire_model/front_right/");
  eqns_.tire_models_.rear_left.register_log_signals(logger_.get(), "imr/tire_model/rear_left/");
  eqns_.tire_models_.rear_right.register_log_signals(logger_.get(), "imr/tire_model/rear_right/");
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackModel<TireModelT, AeroModelT>::declare_parameters()
{
  auto p_def_d = [this](
                   std::string param_name, const double & initial_value, double * storage_location,
                   const std::string & description = "") {
    param_manager_->declare_parameter(
      std::string("vehicle_dynamics_double_track.") + param_name, storage_location, initial_value,
      tam::pmg::ParameterType::DOUBLE, description);
  };
  // Masses & Inertial moments
  p_def_d("mass_vehicle_kg", 800, &eqns_.p_.m, "Overall vehicle mass including tires");  // [kg]
  p_def_d("mass_wheel_kg.front", 15, &eqns_.p_.m_t_f, "Mass for an individual tire");    // [kg]
  p_def_d("mass_wheel_kg.rear", 15, &eqns_.p_.m_t_r, "Mass for an individual tire");     // [kg]
  p_def_d("moment_of_inertia_kgpm2.z", 1000, &eqns_.p_.I_z);                             // [kg m^2]
  p_def_d("moment_of_inertia_kgpm2.y", 500, &eqns_.p_.I_y);                              // [kg m^2]
  p_def_d("moment_of_inertia_kgpm2.x", 100, &eqns_.p_.I_x);                              // [kg m^2]

  // Geometric parameters
  p_def_d("wheelbase_m", 3.2, &eqns_.p_.l, "Wheelbase of the vehicle");  // [m]
  p_def_d("track_width_m.front", 1.6, &eqns_.p_.b_f);                    // [m]
  p_def_d("track_width_m.rear", 1.5, &eqns_.p_.b_r);                     // [m]

  // Simulink currently assumes that the cog lies in the axle plane
  p_def_d(
    "cog.distance_from_front_axle_m", 1.724, &eqns_.p_.l_f,
    "Distance from the front axle to the COG");  // [m]
  p_def_d("cog.height_m", 0.3, &eqns_.p_.h_cg);  //  [m]
  p_def_d(
    "suspension.pitch_center_height_m.accel", 0.2, &eqns_.p_.h_pc_accel,
    "Height of the pitch center during acceleration");  //  [m]
  p_def_d(
    "suspension.pitch_center_height_m.decel", 0.2, &eqns_.p_.h_pc_decel,
    "Height of the pitch center during deceleration");  //  [m]
  p_def_d(
    "suspension.roll_center_height_m", 0.2, &eqns_.p_.h_rc, "Height of the roll center");  //  [m]

  // Spring & Damping constants
  p_def_d(
    "suspension.vehicle_spring_stiffness_Npm.front", 120000, &eqns_.p_.c_f,
    "Considerung the travel at the virtual suspension point at 0.5 track width");  // [N/m]
  p_def_d(
    "suspension.vehicle_spring_stiffness_Npm.rear", 90000.0, &eqns_.p_.c_r,
    "Considerung the travel at the virtual suspension point at 0.5 track width");  // [N/m]
  p_def_d(
    "suspension.vehicle_damper_coefficient_Nspm.front", 1300.0, &eqns_.p_.d_f,
    "Considerung the travel at the virtual suspension point at 0.5 track width");  // [N*s/m]
  p_def_d(
    "suspension.vehicle_damper_coefficient_Nspm.rear", 900.0, &eqns_.p_.d_r,
    "Considerung the travel at the virtual suspension point at 0.5 track width");  // [N*s/m]
  // Anti-Roll bar
  p_def_d(
    "suspension.antirollbar_virtual_spring_stiffness_Npm.front", 12000, &eqns_.p_.c_ar_f,
    "Considerung the travel at the virtual suspension point at 0.5 track width");  // [N/m]
  p_def_d(
    "suspension.antirollbar_virtual_spring_stiffness_Npm.rear", 5000, &eqns_.p_.c_ar_r,
    "Considerung the travel at the virtual suspension point at 0.5 track width");  // [N/m]

  // Add toe
  p_def_d("suspension.toe_out_rad.front_left", 0.0, &eqns_.p_.toe_out_rad.front_left);
  p_def_d("suspension.toe_out_rad.front_right", 0.0, &eqns_.p_.toe_out_rad.front_right);
  p_def_d("suspension.toe_out_rad.rear_left", 0.0, &eqns_.p_.toe_out_rad.rear_left);
  p_def_d("suspension.toe_out_rad.rear_right", 0.0, &eqns_.p_.toe_out_rad.rear_right);

  p_def_d("suspension.camber_out_rad.front_left", 0.0, &eqns_.p_.camber_out_rad.front_left);
  p_def_d("suspension.camber_out_rad.front_right", 0.0, &eqns_.p_.camber_out_rad.front_right);
  p_def_d("suspension.camber_out_rad.rear_left", 0.0, &eqns_.p_.camber_out_rad.rear_left);
  p_def_d("suspension.camber_out_rad.rear_right", 0.0, &eqns_.p_.camber_out_rad.rear_right);

  // Tire characteristics
  p_def_d("tire.rolling_radius_m.front", 0.3, &eqns_.p_.rr_w_f, "At 20 mps");  // [m]@20mps
  p_def_d("tire.rolling_radius_m.rear", 0.3, &eqns_.p_.rr_w_r, "At 20 mps");   // [m]@20mps
  p_def_d("tire.spring_stiffness_Npm.front", 250000.0, &eqns_.p_.c_t_f);       // [N/m]
  p_def_d("tire.spring_stiffness_Npm.rear", 250000.0, &eqns_.p_.c_t_r);        // [N/m]
  p_def_d(
    "tire.rolling_resistance_coefficient", 0.025, &eqns_.p_.c_rr,
    "Rolling resistance coefficient. Ratio of F_roll to F_n");

  // Tire Radii
  param_manager_->declare_parameter(
    "vehicle_dynamics_double_track.tire.rolling_radius_m.velocity_scaling.velocity_points_mps",
    &eqns_.p_.rr_vel_scale_vel_points_mps, std::vector<double>({0, 100}),
    tam::pmg::ParameterType::DOUBLE_ARRAY,
    "Velocity points for scaling the tire radius dynamically with velocity");
  param_manager_->declare_parameter(
    "vehicle_dynamics_double_track.tire.rolling_radius_m.velocity_scaling.factors",
    &eqns_.p_.rr_vel_scale_scale_factors, std::vector<double>({1.0, 1.0}),
    tam::pmg::ParameterType::DOUBLE_ARRAY,
    "Scale factors for scaling the tire radius dynamically with velocity");

  // Parameters for Tire Model
  eqns_.tire_models_.front_left.declare_parameters(
    param_manager_.get(), "vehicle_dynamics_double_track.z.tire_model.front_left.");
  eqns_.tire_models_.front_right.declare_parameters(
    param_manager_.get(), "vehicle_dynamics_double_track.z.tire_model.front_right.");
  eqns_.tire_models_.rear_left.declare_parameters(
    param_manager_.get(), "vehicle_dynamics_double_track.z.tire_model.rear_left.");
  eqns_.tire_models_.rear_right.declare_parameters(
    param_manager_.get(), "vehicle_dynamics_double_track.z.tire_model.rear_right.");

  eqns_.aero_model_.declare_parameters(
    param_manager_.get(), "vehicle_dynamics_double_track.aerodynamics.");
}
}  // namespace tam::ocd::vehicle_dynamics
