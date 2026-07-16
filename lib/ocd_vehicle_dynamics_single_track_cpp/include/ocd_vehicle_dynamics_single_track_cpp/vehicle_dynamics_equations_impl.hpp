// Copyright 2026 Simon Sagmeister
#pragma once

#include <algorithm>
#include <limits>

#include "ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_equations.hpp"

#define EVAL_MACRO_PER_WHEEL(x) \
  x(front_left);                \
  x(front_right);               \
  x(rear_left);                 \
  x(rear_right)
namespace tam::ocd::vehicle_dynamics
{
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_dependent_parameters()
{
  p_dep_.l_r = p_.l - p_.l_f;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_effective_steering_angle()
{
  // Single track: average left/right steering angles per axle
  double mean_front =
    0.5 * (steering_angle_per_wheel_rad_.front_left + steering_angle_per_wheel_rad_.front_right);
  double mean_rear =
    0.5 * (steering_angle_per_wheel_rad_.rear_left + steering_angle_per_wheel_rad_.rear_right);
  imr_.effective_steering_angle_per_wheel_rad.front_left = mean_front;
  imr_.effective_steering_angle_per_wheel_rad.front_right = mean_front;
  imr_.effective_steering_angle_per_wheel_rad.rear_left = mean_rear;
  imr_.effective_steering_angle_per_wheel_rad.rear_right = mean_rear;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calc_dynamic_tire_radius()
{
  // Interpolation function
  imr_.dynamic_tire_radius_m =
    double_per_wheel_t::from_front_and_rear(p_.rr_w_f, p_.rr_w_r) *
    tam::helpers::numerical::interp(
      x_vec_[x::v_x_mps], p_.rr_vel_scale_vel_points_mps, p_.rr_vel_scale_scale_factors);
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_vertical_tire_force_N()
{
  // Quasi-static vertical load distribution including aero and external forces/moments.
  // z-up convention: downforce = negative F_z, so -F_z adds to load.
  double total_vertical_load =
    p_.m * G_CONST - imr_.aero_force_N.z - external_influences_.external_force_N.z;

  // Pitch moment contributions (positive M_y = nose down = increases front axle load).
  // Neglect roll moment (x-axis) per single track assumption.
  double pitch_moment = imr_.aero_torque_Nm.y + external_influences_.external_torque_Nm.y;

  // Moment equilibrium about rear axle contact point
  double front_axle_load = (total_vertical_load * p_dep_.l_r + pitch_moment) / p_.l;
  double rear_axle_load = total_vertical_load - front_axle_load;

  // Per-wheel (half axle load), clamped to small positive value for numeric stability
  imr_.vertical_tire_force_N.front_left = std::max(0.001, front_axle_load / 2.0);
  imr_.vertical_tire_force_N.front_right = imr_.vertical_tire_force_N.front_left;
  imr_.vertical_tire_force_N.rear_left = std::max(0.001, rear_axle_load / 2.0);
  imr_.vertical_tire_force_N.rear_right = imr_.vertical_tire_force_N.rear_left;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<
  TireModelT, AeroModelT>::calculate_velocity_wheel_over_ground_mps()
{
  // Single track: no track width (b_f = b_r = 0), so left = right
  Eigen::Vector2d v_front = {
    x_vec_[x::v_x_mps], x_vec_[x::v_y_mps] + p_.l_f * x_vec_[x::psi_dot_radps]};
  Eigen::Vector2d v_rear = {
    x_vec_[x::v_x_mps], x_vec_[x::v_y_mps] - p_dep_.l_r * x_vec_[x::psi_dot_radps]};
  imr_.velocity_wheel_over_ground_mps.front_left = v_front;
  imr_.velocity_wheel_over_ground_mps.front_right = v_front;
  imr_.velocity_wheel_over_ground_mps.rear_left = v_rear;
  imr_.velocity_wheel_over_ground_mps.rear_right = v_rear;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<
  TireModelT, AeroModelT>::calculate_velocity_wheel_over_ground_tire_frame_mps()
{
  // Rotate the wheel velocity over ground into the tire coordinate frame
  double angle = 0;
  Eigen::Matrix2d rot_matrix;

#define EVAL_VELOCITY_WHEEL_OVER_GROUND_TIRE_FRAME(wheel)        \
                                                                 \
  angle = imr_.effective_steering_angle_per_wheel_rad.wheel;     \
  rot_matrix << cos(angle), sin(angle), -sin(angle), cos(angle); \
  imr_.velocity_wheel_over_ground_tire_frame_mps.wheel =         \
    rot_matrix * imr_.velocity_wheel_over_ground_mps.wheel;

  EVAL_MACRO_PER_WHEEL(EVAL_VELOCITY_WHEEL_OVER_GROUND_TIRE_FRAME);
#undef EVAL_VELOCITY_WHEEL_OVER_GROUND_TIRE_FRAME
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_velocity_tire_rotation_mps()
{
  // Velocity due to tire rotation
  imr_.velocity_tire_rotation_mps.front_left =
    imr_.dynamic_tire_radius_m.front_left * wheel_speeds_radps_.front_left;
  imr_.velocity_tire_rotation_mps.front_right =
    imr_.dynamic_tire_radius_m.front_right * wheel_speeds_radps_.front_right;
  imr_.velocity_tire_rotation_mps.rear_left =
    imr_.dynamic_tire_radius_m.rear_left * wheel_speeds_radps_.rear_left;
  imr_.velocity_tire_rotation_mps.rear_right =
    imr_.dynamic_tire_radius_m.rear_right * wheel_speeds_radps_.rear_right;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_tire_longitudinal_slip()
{
  constexpr static auto long_slip_def =
    [](double rotational_velocity_mps, double vel_over_ground_mps) {
      return std::clamp(
        (rotational_velocity_mps - vel_over_ground_mps) / std::max(3.0, vel_over_ground_mps), -1.0,
        1.0);
    };

#define CALC_LONG_SLIP(x)                        \
  imr_.tire_longitudinal_slip.x = long_slip_def( \
    imr_.velocity_tire_rotation_mps.x, imr_.velocity_wheel_over_ground_tire_frame_mps.x[0]);

  EVAL_MACRO_PER_WHEEL(CALC_LONG_SLIP);
#undef CALC_LONG_SLIP
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_tire_slip_angle_rad()
{
  auto slip_angle_def = [this](double vy, double vx) { return -atan2(vy, std::max(1.0, vx)); };

  // clang-format off
  #define CALC_TIRE_SLIP(x)                               \
  imr_.tire_slip_angle_rad.x = slip_angle_def(           \
    imr_.velocity_wheel_over_ground_tire_frame_mps.x[1], \
    imr_.velocity_wheel_over_ground_tire_frame_mps.x[0])

  EVAL_MACRO_PER_WHEEL(CALC_TIRE_SLIP);
  #undef CALC_TIRE_SLIP
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_tire_forces_tire_frame_N()
{
  // Zero out tire forces when at rest
  if (
    std::pow(x_vec_[x::v_x_mps], 2) < 0.001 && std::pow(x_vec_[x::v_y_mps], 2) < 0.001 &&
    std::pow(wheel_speeds_radps_.front_left, 2) < 0.001 &&
    std::pow(wheel_speeds_radps_.front_right, 2) < 0.001 &&
    std::pow(wheel_speeds_radps_.rear_left, 2) < 0.001 &&
    std::pow(wheel_speeds_radps_.rear_right, 2) < 0.001) {
    imr_.tire_forces_tire_frame_N.front_right = {0, 0};
    imr_.tire_forces_tire_frame_N.front_left = {0, 0};
    imr_.tire_forces_tire_frame_N.rear_left = {0, 0};
    imr_.tire_forces_tire_frame_N.rear_right = {0, 0};
    return;
  }

  auto tire_model_eval = [this](
                           double F_z_N, double long_slip, double slip_angle_rad,
                           TireModelT & model, Eigen::Vector2d & target, bool mirror,
                           double external_scale) {
    // Mirror the tire model to account for asymmetry
    int mirror_factor = (mirror) ? -1 : 1;
    // No camber in single track model
    auto out = model.evaluate({long_slip, mirror_factor * slip_angle_rad, F_z_N, 0.0});
    target[0] = out.longitudinal_force_N * external_scale;
    target[1] = mirror_factor * out.lateral_force_N * external_scale;
  };

  // Use directly computed kappa and alpha (no tire relaxation delay in single track)
  // clang-format off
  #define EVAL_TIRE_MODEL(x, mirror)   \
  tire_model_eval( \
    imr_.vertical_tire_force_N.x, \
    imr_.tire_longitudinal_slip.x, \
    imr_.tire_slip_angle_rad.x, \
    tire_models_.x, \
    imr_.tire_forces_tire_frame_N.x, \
    mirror, \
    external_influences_.lambda_mue.x \
  )

  EVAL_TIRE_MODEL(front_left, false);
  EVAL_TIRE_MODEL(rear_left, false);
  EVAL_TIRE_MODEL(front_right, true);
  EVAL_TIRE_MODEL(rear_right, true);
  #undef EVAL_TIRE_MODEL
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_tire_forces_N()
{
  // Tire Forces @ x - y axis of the vehicle
  // Rotate the tire frame forces into the vehicle frame
  // clang-format off
  #define EVAL_WHEEL_TIRE_FORCES(wheel) do {\
    Eigen::Matrix2d rot_matrix;\
    double angle = -imr_.effective_steering_angle_per_wheel_rad.wheel;\
    rot_matrix << cos(angle), sin(angle), \
                  -sin(angle), cos(angle); \
    imr_.tire_forces_N.wheel = rot_matrix * imr_.tire_forces_tire_frame_N.wheel;\
  } while(0)

  EVAL_MACRO_PER_WHEEL(EVAL_WHEEL_TIRE_FORCES);
  #undef EVAL_WHEEL_TIRE_FORCES
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_tire_rolling_resistance_N()
{
// Always positive since always going forward
#define CALC_TIRE_RR(x_in) \
  imr_.tire_rolling_resistance_N.x_in = imr_.vertical_tire_force_N.x_in * p_.c_rr;
  EVAL_MACRO_PER_WHEEL(CALC_TIRE_RR);
#undef CALC_TIRE_RR
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_aerodynamics()
{
  // Aerodynamic forces (no pitch or heave in single track)
  types::AeroModelInput input;
  input.vx_mps = x_vec_[x::v_x_mps];
  input.vy_mps = x_vec_[x::v_y_mps];
  input.wind_mps = external_influences_.wind_mps;
  input.pitch_angle_rad = 0.0;
  input.z_m = 0.0;

  types::AeroModelOutput aero_model_out = aero_model_.evaluate(input);
  imr_.aero_force_N = aero_model_out.force_cog_N;
  imr_.aero_torque_Nm = aero_model_out.torque_Nm;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_resulting_force_N()
{
  // Scale external influences at low speed
  double external_influence_scale_factor_lowspeed =
    std::clamp((0.2 * (x_vec_[x::v_x_mps] - 2.0)), 0.0, 1.0);

  // Sum of tire forces @ x axis
  imr_.resulting_force_N.x =
    imr_.tire_forces_N.front_left[0] + imr_.tire_forces_N.front_right[0] +
    imr_.tire_forces_N.rear_left[0] + imr_.tire_forces_N.rear_right[0] + imr_.aero_force_N.x +
    external_influences_.external_force_N.x * external_influence_scale_factor_lowspeed;

  // Sum of tire forces @ y axis
  imr_.resulting_force_N.y =
    imr_.tire_forces_N.front_left[1] + imr_.tire_forces_N.front_right[1] +
    imr_.tire_forces_N.rear_left[1] + imr_.tire_forces_N.rear_right[1] + imr_.aero_force_N.y +
    external_influences_.external_force_N.y * external_influence_scale_factor_lowspeed;

  // No vertical dynamics in single track model
  imr_.resulting_force_N.z = 0.0;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_resulting_torque_Nm()
{
  // clang-format off
  // Sum of tire moments @ z axis
  // With b_f = b_r = 0 (single track), the differential longitudinal force terms vanish
  imr_.resulting_torque_Nm.z =
      (imr_.tire_forces_N.front_left[1] + imr_.tire_forces_N.front_right[1]) * p_.l_f
    - (imr_.tire_forces_N.rear_left[1] + imr_.tire_forces_N.rear_right[1]) * p_dep_.l_r
    + imr_.aero_torque_Nm.z
    + external_influences_.external_torque_Nm.z;

  // No pitch or roll dynamics in single track model
  imr_.resulting_torque_Nm.x = 0.0;
  imr_.resulting_torque_Nm.y = 0.0;
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_drivetrain_load_torque_Nm()
{
  // clang-format off
  #define CALC_DT_LOAD_TRQ(in_tire)                                      \
    drivetrain_load_torque_per_wheel_Nm_.in_tire =                                                 \
      (imr_.tire_forces_tire_frame_N.in_tire[0] + imr_.tire_rolling_resistance_N.in_tire) * \
      imr_.dynamic_tire_radius_m.in_tire

  EVAL_MACRO_PER_WHEEL(CALC_DT_LOAD_TRQ);
  #undef CALC_DT_LOAD_TRQ
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_steering_load_torque_Nm()
{
  // clang-format off
  #define CALC_SA_LOAD_TRQ(in_tire)\
    steering_load_torque_per_wheel_Nm_.in_tire = 0

  EVAL_MACRO_PER_WHEEL(CALC_SA_LOAD_TRQ);
  #undef CALC_SA_LOAD_TRQ
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::calculate_intermediate_results()
{
  calculate_dependent_parameters();
  calculate_effective_steering_angle();
  calc_dynamic_tire_radius();
  calculate_aerodynamics();
  calculate_vertical_tire_force_N();
  calculate_velocity_wheel_over_ground_mps();
  calculate_velocity_wheel_over_ground_tire_frame_mps();
  calculate_velocity_tire_rotation_mps();
  calculate_tire_longitudinal_slip();
  calculate_tire_slip_angle_rad();
  calculate_tire_forces_tire_frame_N();
  calculate_tire_forces_N();
  calculate_tire_rolling_resistance_N();
  calculate_resulting_force_N();
  calculate_resulting_torque_Nm();
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsSingleTrackEqns<TireModelT, AeroModelT>::evaluate()
{
  // Calc the required forces and moments
  calculate_intermediate_results();
  calculate_drivetrain_load_torque_Nm();
  calculate_steering_load_torque_Nm();

  // State Equation
  // ============================================================================================
  // clang-format off
  // Vehicle Movement
  x_dot_vec_[x::x_m] = cos(x_vec_[x::psi_rad]) * x_vec_[x::v_x_mps] - sin(x_vec_[x::psi_rad]) * x_vec_[x::v_y_mps]; // NOLINT
  x_dot_vec_[x::y_m] = sin(x_vec_[x::psi_rad]) * x_vec_[x::v_x_mps] + cos(x_vec_[x::psi_rad]) * x_vec_[x::v_y_mps];  // NOLINT
  x_dot_vec_[x::psi_rad] = x_vec_[x::psi_dot_radps];
  x_dot_vec_[x::v_x_mps] = imr_.resulting_force_N.x / p_.m + x_vec_[x::psi_dot_radps] * x_vec_[x::v_y_mps];  // NOLINT
  x_dot_vec_[x::v_y_mps] = imr_.resulting_force_N.y / p_.m - x_vec_[x::psi_dot_radps] * x_vec_[x::v_x_mps]; // NOLINT
  x_dot_vec_[x::psi_dot_radps] = imr_.resulting_torque_Nm.z / p_.I_z;
  // clang-format on

  // Output
  // ============================================================================================
  vd_output_.position_m.x = x_vec_[x::x_m];
  vd_output_.position_m.y = x_vec_[x::y_m];
  vd_output_.orientation_rad.z = x_vec_[x::psi_rad];
  vd_output_.velocity_mps.x = x_vec_[x::v_x_mps];
  vd_output_.velocity_mps.y = x_vec_[x::v_y_mps];
  vd_output_.angular_velocity_radps.z = x_vec_[x::psi_dot_radps];
  vd_output_.angular_acceleration_radps2.z = x_dot_vec_[x::psi_dot_radps];
  vd_output_.tire_longitudinal_slip_per_wheel = imr_.tire_longitudinal_slip;
  vd_output_.tire_slip_angle_per_wheel_rad = imr_.tire_slip_angle_rad;
#define GET_TIRE_FORCES(wheel)                                      \
  vd_output_.longitudinal_tire_force_tire_frame_per_wheel_N.wheel = \
    imr_.tire_forces_tire_frame_N.wheel[0];                         \
  vd_output_.lateral_tire_force_tire_frame_per_wheel_N.wheel =      \
    imr_.tire_forces_tire_frame_N.wheel[1];
  GET_TIRE_FORCES(front_left);
  GET_TIRE_FORCES(front_right);
  GET_TIRE_FORCES(rear_left);
  GET_TIRE_FORCES(rear_right);
#undef GET_TIRE_FORCES
  vd_output_.vertical_tire_force_per_wheel_N = imr_.vertical_tire_force_N;
  //
  // Adapt the local accelerations to global accelerations since this is what the imu would measure
  vd_output_.acceleration_mps2.x =
    x_dot_vec_[x::v_x_mps] - x_vec_[x::psi_dot_radps] * x_vec_[x::v_y_mps];
  vd_output_.acceleration_mps2.y =
    x_dot_vec_[x::v_y_mps] + x_vec_[x::psi_dot_radps] * x_vec_[x::v_x_mps];
  vd_output_.acceleration_mps2.z = tam::constants::g_earth;
};
}  // namespace tam::ocd::vehicle_dynamics
#undef EVAL_MACRO_PER_WHEEL
