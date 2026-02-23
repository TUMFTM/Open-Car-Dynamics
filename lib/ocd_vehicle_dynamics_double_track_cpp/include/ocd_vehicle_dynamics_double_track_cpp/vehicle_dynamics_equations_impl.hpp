// Copyright 2026 Simon Sagmeister
#pragma once

#include <algorithm>
#include <limits>

#include "ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_equations.hpp"

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
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_dependent_parameters()
{
  p_dep_.l_r = p_.l - p_.l_f;
  p_dep_.m_sprung = p_.m - 2 * p_.m_t_f - 2 * p_.m_t_r;
  // Calculate the cog for the unsprung mass
  p_dep_.l_r_sprung = (p_.m * p_dep_.l_r - 2 * p_.m_t_f * p_.l) / p_dep_.m_sprung;
  p_dep_.l_f_sprung = p_.l - p_dep_.l_r_sprung;
  // Calculate the parameters used for squat force calc
  p_dep_.slf = (p_.h_rc / (0.5 * p_.b_f));
  p_dep_.slr = (p_.h_rc / (0.5 * p_.b_r));
  p_dep_.sadf_deceleration = (p_.h_pc_decel / (p_.l_f));
  p_dep_.sadr_deceleration = (p_.h_pc_decel / (p_dep_.l_r));
  p_dep_.sadf_acceleration = (p_.h_pc_accel / (p_.l_f));
  p_dep_.sadr_acceleration = (p_.h_pc_accel / (p_dep_.l_r));
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_effective_steering_angle()
{
  double_per_wheel_t toe_addition = p_.toe_out_rad;
  // Toe out means steering to right on the right side -> Therefore negate the sign.
  toe_addition.front_right *= -1;
  toe_addition.rear_right *= -1;

  imr_.effective_steering_angle_per_wheel_rad = steering_angle_per_wheel_rad_ + toe_addition;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calc_dynamic_tire_radius()
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
void VehicleDynamicsDoubleTrackEqns<
  TireModelT, AeroModelT>::calculate_tire_spring_initial_compression_m()
{
  // Tire spring compression @ standstill
  imr_.tire_spring_initial_compression_m.front_left =
    (p_.m * G_CONST * p_dep_.l_r / (2 * p_.l)) / p_.c_t_f;
  imr_.tire_spring_initial_compression_m.rear_left =
    (p_.m * G_CONST * p_.l_f / (2 * p_.l)) / p_.c_t_r;
  imr_.tire_spring_initial_compression_m.front_right =
    imr_.tire_spring_initial_compression_m.front_left;
  imr_.tire_spring_initial_compression_m.rear_right =
    imr_.tire_spring_initial_compression_m.rear_left;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_tire_spring_force_N()
{  // Tire spring forces
  // Clamp to Fz = 0.001 in order to avoid numeric problems in the tire model.
  imr_.tire_spring_force_N.front_left = std::clamp(
    p_.c_t_f * (x_vec_[x::z_w_FL_m] - imr_.tire_spring_initial_compression_m.front_left),
    std::numeric_limits<double>::lowest(), -0.001);
  imr_.tire_spring_force_N.front_right = std::clamp(
    p_.c_t_f * (x_vec_[x::z_w_FR_m] - imr_.tire_spring_initial_compression_m.front_right),
    std::numeric_limits<double>::lowest(), -0.001);
  imr_.tire_spring_force_N.rear_left = std::clamp(
    p_.c_t_r * (x_vec_[x::z_w_RL_m] - imr_.tire_spring_initial_compression_m.rear_left),
    std::numeric_limits<double>::lowest(), -0.001);
  imr_.tire_spring_force_N.rear_right = std::clamp(
    p_.c_t_r * (x_vec_[x::z_w_RR_m] - imr_.tire_spring_initial_compression_m.rear_right),
    std::numeric_limits<double>::lowest(), -0.001);
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_vertical_tire_force_N()
{
  imr_.vertical_tire_force_N.front_left = -imr_.tire_spring_force_N.front_left;
  imr_.vertical_tire_force_N.front_right = -imr_.tire_spring_force_N.front_right;
  imr_.vertical_tire_force_N.rear_left = -imr_.tire_spring_force_N.rear_left;
  imr_.vertical_tire_force_N.rear_right = -imr_.tire_spring_force_N.rear_right;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<
  TireModelT, AeroModelT>::calculate_velocity_wheel_over_ground_mps()
{  // Tire Velocity Vector @ x - y axis
  imr_.velocity_wheel_over_ground_mps.front_left = {
    x_vec_[x::v_x_mps] - p_.b_f * x_vec_[x::psi_dot_radps] / 2,
    x_vec_[x::v_y_mps] + p_.l_f * x_vec_[x::psi_dot_radps]};
  imr_.velocity_wheel_over_ground_mps.front_right = {
    x_vec_[x::v_x_mps] + p_.b_f * x_vec_[x::psi_dot_radps] / 2,
    x_vec_[x::v_y_mps] + p_.l_f * x_vec_[x::psi_dot_radps]};
  imr_.velocity_wheel_over_ground_mps.rear_left = {
    x_vec_[x::v_x_mps] - p_.b_r * x_vec_[x::psi_dot_radps] / 2,
    x_vec_[x::v_y_mps] - p_dep_.l_r * x_vec_[x::psi_dot_radps]};
  imr_.velocity_wheel_over_ground_mps.rear_right = {
    x_vec_[x::v_x_mps] + p_.b_r * x_vec_[x::psi_dot_radps] / 2,
    x_vec_[x::v_y_mps] - p_dep_.l_r * x_vec_[x::psi_dot_radps]};
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<
  TireModelT, AeroModelT>::calculate_velocity_wheel_over_ground_tire_frame_mps()
{
  // Rotate the wheel velocity over ground into the vehicle coordinate frame (passive rotation,
  // i.e., reverse rotation)

  double_per_wheel_t toe_addition = p_.toe_out_rad;
  // Toe out means steering to right on the right side -> Therefore negate the sign.
  toe_addition.front_right *= -1;
  toe_addition.rear_right *= -1;

  // Required for macro below
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
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_velocity_tire_rotation_mps()
{
  // Velocity due to tire roll @ x - y axis:
  imr_.velocity_tire_rotation_mps.front_left =  // p_.rr_w_f * wheel_speeds_radps_.front_left;
    imr_.dynamic_tire_radius_m.front_left * wheel_speeds_radps_.front_left;
  imr_.velocity_tire_rotation_mps.front_right =  // p_.rr_w_f * wheel_speeds_radps_.front_right;
    imr_.dynamic_tire_radius_m.front_right * wheel_speeds_radps_.front_right;
  imr_.velocity_tire_rotation_mps.rear_left =  // p_.rr_w_r * wheel_speeds_radps_.rear_left;
    imr_.dynamic_tire_radius_m.rear_left * wheel_speeds_radps_.rear_left;
  imr_.velocity_tire_rotation_mps.rear_right =  // p_.rr_w_r * wheel_speeds_radps_.rear_right;
    imr_.dynamic_tire_radius_m.rear_right * wheel_speeds_radps_.rear_right;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_tire_longitudinal_slip()
{
  // TODO(Simon S) verify slip definition

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
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_tire_slip_angle_rad()
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
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_tire_forces_tire_frame_N()
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
                           double external_scale, double camber_out_rad) {
    // Mirror the tire model to account for asymmetry
    int mirror_factor = (mirror) ? -1 : 1;
    auto out = model.evaluate({long_slip, mirror_factor * slip_angle_rad, F_z_N, -camber_out_rad});
    target[0] = out.longitudinal_force_N * external_scale;
    target[1] = mirror_factor * out.lateral_force_N * external_scale;
  };

  // Use the delayed kappa and alpha
  double_per_wheel_t kappa, alpha;
  kappa.front_left = x_vec_[x::kappa_tire_FL];
  kappa.front_right = x_vec_[x::kappa_tire_FR];
  kappa.rear_left = x_vec_[x::kappa_tire_RL];
  kappa.rear_right = x_vec_[x::kappa_tire_RR];
  alpha.front_left = x_vec_[x::alpha_tire_FL];
  alpha.front_right = x_vec_[x::alpha_tire_FR];
  alpha.rear_left = x_vec_[x::alpha_tire_RL];
  alpha.rear_right = x_vec_[x::alpha_tire_RR];

  // clang-format off
  #define EVAL_TIRE_MODEL(x, mirror)   \
  tire_model_eval( \
    imr_.vertical_tire_force_N.x, \
    kappa.x, \
    alpha.x, \
    tire_models_.x, \
    imr_.tire_forces_tire_frame_N.x, \
    mirror, \
    external_influences_.lambda_mue.x, \
    p_.camber_out_rad.x \
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
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_tire_forces_N()
{
  // Tire Forces @ x - y axis of the vehicle
  // Rotate the tire frame forces into the vehicle frame (passive rotation, i.e., reverse rotation)
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
void VehicleDynamicsDoubleTrackEqns<
  TireModelT, AeroModelT>::calculate_suspension_spring_initial_compression_m()
{
  // Compression of suspension coils @ standstill
  imr_.suspension_spring_initial_compression_m.front_left =
    p_dep_.m_sprung * G_CONST * p_dep_.l_r_sprung / (2 * p_.c_f * p_.l);
  imr_.suspension_spring_initial_compression_m.rear_left =
    p_dep_.m_sprung * G_CONST * p_dep_.l_f_sprung / (2 * p_.c_r * p_.l);
  imr_.suspension_spring_initial_compression_m.front_right =
    imr_.suspension_spring_initial_compression_m.front_left;
  imr_.suspension_spring_initial_compression_m.rear_right =
    imr_.suspension_spring_initial_compression_m.rear_left;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<
  TireModelT, AeroModelT>::calculate_suspension_spring_compression_m()
{
  imr_.suspension_spring_compression_m.front_left =
    -imr_.suspension_spring_initial_compression_m.front_left + x_vec_[x::z_m] +
    x_vec_[x::phi_rad] * p_.b_f / 2 - x_vec_[x::theta_rad] * p_dep_.l_f_sprung -
    x_vec_[x::z_w_FL_m];
  imr_.suspension_spring_compression_m.front_right =
    -imr_.suspension_spring_initial_compression_m.front_right + x_vec_[x::z_m] -
    x_vec_[x::phi_rad] * p_.b_f / 2 - x_vec_[x::theta_rad] * p_dep_.l_f_sprung -
    x_vec_[x::z_w_FR_m];
  imr_.suspension_spring_compression_m.rear_left =
    -imr_.suspension_spring_initial_compression_m.rear_left + x_vec_[x::z_m] +
    x_vec_[x::phi_rad] * p_.b_r / 2 + x_vec_[x::theta_rad] * p_dep_.l_r_sprung -
    x_vec_[x::z_w_RL_m];
  imr_.suspension_spring_compression_m.rear_right =
    -imr_.suspension_spring_initial_compression_m.rear_right + x_vec_[x::z_m] -
    x_vec_[x::phi_rad] * p_.b_r / 2 + x_vec_[x::theta_rad] * p_dep_.l_r_sprung -
    x_vec_[x::z_w_RR_m];
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<
  TireModelT, AeroModelT>::calculate_suspension_damper_compression_speed_mps()
{
  // Push/ Pull of suspension dampers
  imr_.suspension_damper_compression_speed_mps.front_left =
    x_vec_[x::v_z_mps] + x_vec_[x::phi_dot_radps] * p_.b_f / 2 -
    x_vec_[x::theta_dot_radps] * p_dep_.l_f_sprung - x_vec_[x::v_z_w_FL_mps];
  imr_.suspension_damper_compression_speed_mps.front_right =
    x_vec_[x::v_z_mps] - x_vec_[x::phi_dot_radps] * p_.b_f / 2 -
    x_vec_[x::theta_dot_radps] * p_dep_.l_f_sprung - x_vec_[x::v_z_w_FR_mps];
  imr_.suspension_damper_compression_speed_mps.rear_left =
    x_vec_[x::v_z_mps] + x_vec_[x::phi_dot_radps] * p_.b_r / 2 +
    x_vec_[x::theta_dot_radps] * p_dep_.l_r_sprung - x_vec_[x::v_z_w_RL_mps];
  imr_.suspension_damper_compression_speed_mps.rear_right =
    x_vec_[x::v_z_mps] - x_vec_[x::phi_dot_radps] * p_.b_r / 2 +
    x_vec_[x::theta_dot_radps] * p_dep_.l_r_sprung - x_vec_[x::v_z_w_RR_mps];
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_suspension_spring_force_N()
{
  // Suspension spring forces
  imr_.suspension_spring_force_N.front_left =
    p_.c_f * imr_.suspension_spring_compression_m.front_left;
  imr_.suspension_spring_force_N.front_right =
    p_.c_f * imr_.suspension_spring_compression_m.front_right;
  imr_.suspension_spring_force_N.rear_left =
    p_.c_r * imr_.suspension_spring_compression_m.rear_left;
  imr_.suspension_spring_force_N.rear_right =
    p_.c_r * imr_.suspension_spring_compression_m.rear_right;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_suspension_damper_force_N()
{
  // Suspension damper forces
  imr_.suspension_damper_force_N.front_left =
    p_.d_f * imr_.suspension_damper_compression_speed_mps.front_left;
  imr_.suspension_damper_force_N.front_right =
    p_.d_f * imr_.suspension_damper_compression_speed_mps.front_right;
  imr_.suspension_damper_force_N.rear_left =
    p_.d_r * imr_.suspension_damper_compression_speed_mps.rear_left;
  imr_.suspension_damper_force_N.rear_right =
    p_.d_r * imr_.suspension_damper_compression_speed_mps.rear_right;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_antiroll_bar_force_N()
{
  // Anti-Roll bar forces
  imr_.antiroll_bar_force_N.front_left = 0.5 * p_.c_ar_f *
                                         (imr_.suspension_spring_compression_m.front_left -
                                          imr_.suspension_spring_compression_m.front_right);
  imr_.antiroll_bar_force_N.rear_left = 0.5 * p_.c_ar_r *
                                        (imr_.suspension_spring_compression_m.rear_left -
                                         imr_.suspension_spring_compression_m.rear_right);
  imr_.antiroll_bar_force_N.front_right = -imr_.antiroll_bar_force_N.front_left;
  imr_.antiroll_bar_force_N.rear_right = -imr_.antiroll_bar_force_N.rear_left;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_tire_rolling_resistance_N()
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
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_axle_vertical_force_N()
{  // Total Axle Forces
  // See page A-30
  // https://babel.hathitrust.org/cgi/pt?id=mdp.39015075298698&view=1up&seq=147&skin=2021
  auto calc_axle_force = [this](
                           double F_x, double F_y, double K_sli, double K_sadi, double K_sadi2) {
    double k_sadi_used = (F_x > 0.0) ? K_sadi2 : K_sadi;
    return (K_sli * F_y - k_sadi_used * F_x);
  };

  imr_.axle_vertical_force_N.front_left = calc_axle_force(
    -imr_.tire_forces_N.front_left[0], imr_.tire_forces_N.front_left[1], p_dep_.slf,
    p_dep_.sadf_deceleration, p_dep_.sadf_acceleration);
  imr_.axle_vertical_force_N.front_right = calc_axle_force(
    -imr_.tire_forces_N.front_right[0], -imr_.tire_forces_N.front_right[1], p_dep_.slf,
    p_dep_.sadf_deceleration, p_dep_.sadf_acceleration);
  imr_.axle_vertical_force_N.rear_left = calc_axle_force(
    imr_.tire_forces_N.rear_left[0], imr_.tire_forces_N.rear_left[1], p_dep_.slr,
    p_dep_.sadr_deceleration, p_dep_.sadr_acceleration);
  imr_.axle_vertical_force_N.rear_right = calc_axle_force(
    imr_.tire_forces_N.rear_right[0], -imr_.tire_forces_N.rear_right[1], p_dep_.slr,
    p_dep_.sadr_deceleration, p_dep_.sadr_acceleration);
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<
  TireModelT, AeroModelT>::calculate_resulting_suspension_force_N()
{
  // clang-format off
  #define VERTICAL_FORCES_SUSPENSION(tire_in)                                        \
    imr_.resulting_suspension_force_N.tire_in = \
    (  imr_.suspension_spring_force_N.tire_in \
    + imr_.suspension_damper_force_N.tire_in \
    + imr_.antiroll_bar_force_N.tire_in \
    + imr_.axle_vertical_force_N.tire_in \
    )

  EVAL_MACRO_PER_WHEEL(VERTICAL_FORCES_SUSPENSION);
  #undef VERTICAL_FORCES_SUSPENSION
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<
  TireModelT, AeroModelT>::calculate_resulting_vertical_force_on_wheel_N()
{
// Heave of Tire
// clang-format off
  #define CALC_HEAVE_TIRE(tire_name, tire_mass) \
  imr_.resulting_vertical_force_on_wheel_N.tire_name = \
    + imr_.resulting_suspension_force_N.tire_name \
    - imr_.tire_spring_force_N.tire_name \
    - tire_mass * G_CONST

  CALC_HEAVE_TIRE(front_left, p_.m_t_f);
  CALC_HEAVE_TIRE(front_right, p_.m_t_f);
  CALC_HEAVE_TIRE(rear_left, p_.m_t_r);
  CALC_HEAVE_TIRE(rear_right, p_.m_t_r);
  #undef CALC_HEAVE_TIRE
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_aerodynamics()
{
  // Aerodynamic forces
  types::AeroModelInput input;
  input.vx_mps = x_vec_[x::v_x_mps];
  input.vy_mps = x_vec_[x::v_y_mps];
  input.pitch_angle_rad = x_vec_[x::theta_rad];
  input.z_m = x_vec_[x::z_m];

  types::AeroModelOutput aero_model_out = aero_model_.evaluate(input);
  imr_.aero_force_N = aero_model_out.force_cog_N;
  imr_.aero_torque_Nm = aero_model_out.torque_Nm;
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_resulting_force_N()
{
  // Scale external influences at low speed
  double external_influence_scale_factor_lowspeed =
    std::clamp((0.2 * (x_vec_[x::v_x_mps] - 2.0)), 0.0, 1.0);

  imr_.resulting_force_N.x =
    imr_.tire_forces_N.front_left[0] + imr_.tire_forces_N.front_right[0] +
    imr_.tire_forces_N.rear_left[0] + imr_.tire_forces_N.rear_right[0] + imr_.aero_force_N.x +
    external_influences_.external_force_N.x * external_influence_scale_factor_lowspeed;

  // Sum of tire forces @ y axis
  imr_.resulting_force_N.y =
    imr_.tire_forces_N.front_left[1] + imr_.tire_forces_N.front_right[1] +
    imr_.tire_forces_N.rear_left[1] + imr_.tire_forces_N.rear_right[1] + imr_.aero_force_N.y +
    external_influences_.external_force_N.y * external_influence_scale_factor_lowspeed;
  // Heave of Car
  // clang-format off
  imr_.resulting_force_N.z =
    - imr_.resulting_suspension_force_N.front_left
    - imr_.resulting_suspension_force_N.front_right
    - imr_.resulting_suspension_force_N.rear_left
    - imr_.resulting_suspension_force_N.rear_right
    - p_dep_.m_sprung * G_CONST
    + imr_.aero_force_N.z
    + external_influences_.external_force_N.z;
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_resulting_torque_Nm()
{
  // Sum of tire moments @ z axis
  // clang-format off
  imr_.resulting_torque_Nm.z =
      (imr_.tire_forces_N.front_left[1] + imr_.tire_forces_N.front_right[1]) * p_.l_f
    + (imr_.tire_forces_N.front_right[0] - imr_.tire_forces_N.front_left[0]) * p_.b_f / 2
    + (imr_.tire_forces_N.rear_right[0] - imr_.tire_forces_N.rear_left[0]) * p_.b_r / 2
    - (imr_.tire_forces_N.rear_left[1] + imr_.tire_forces_N.rear_right[1]) * p_dep_.l_r
    + imr_.aero_torque_Nm.z
    + external_influences_.external_torque_Nm.z;


  // Macro to sum of the forces of the spring, damper and antirollbar
  imr_.resulting_torque_Nm.y =
    - (imr_.tire_forces_N.front_left[0] + imr_.tire_forces_N.front_right[0] +
       imr_.tire_forces_N.rear_left[0] + imr_.tire_forces_N.rear_right[0]
      ) * p_.h_cg
    + imr_.resulting_suspension_force_N.front_left * p_.l_f
    + imr_.resulting_suspension_force_N.front_right * p_.l_f
    - imr_.resulting_suspension_force_N.rear_left * p_dep_.l_r
    - imr_.resulting_suspension_force_N.rear_right* p_dep_.l_r
    + p_dep_.m_sprung* G_CONST * (p_.l_f - p_dep_.l_f_sprung)
    + imr_.aero_torque_Nm.y
    + external_influences_.external_torque_Nm.y;

  // Sum of sprung mass moments @ x axis
  imr_.resulting_torque_Nm.x = (
       imr_.tire_forces_N.front_left[1] + imr_.tire_forces_N.front_right[1] +
       imr_.tire_forces_N.rear_left[1] + imr_.tire_forces_N.rear_right[1]
      ) * p_.h_cg
    - imr_.resulting_suspension_force_N.front_left * p_.b_f / 2
    + imr_.resulting_suspension_force_N.front_right * p_.b_f / 2
    - imr_.resulting_suspension_force_N.rear_left * p_.b_r / 2
    + imr_.resulting_suspension_force_N.rear_right * p_.b_r / 2
    + imr_.aero_torque_Nm.x
    + external_influences_.external_torque_Nm.x;
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_drivetrain_load_torque_Nm()
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
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_steering_load_torque_Nm()
{
  // clang-format off
  #define CALC_SA_LOAD_TRQ(in_tire)\
    steering_load_torque_per_wheel_Nm_.in_tire = 0
    // Some fancy equation or model for torque steer
    // possibly from here: https://link.springer.com/content/pdf/10.1007/978-3-8348-9026-9_4

  EVAL_MACRO_PER_WHEEL(CALC_SA_LOAD_TRQ);
  #undef CALC_SA_LOAD_TRQ
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::calculate_intermediate_results()
{
  calculate_dependent_parameters();
  calculate_effective_steering_angle();
  calc_dynamic_tire_radius();
  calculate_tire_spring_initial_compression_m();
  calculate_tire_spring_force_N();
  calculate_vertical_tire_force_N();
  calculate_velocity_wheel_over_ground_mps();
  calculate_velocity_wheel_over_ground_tire_frame_mps();
  calculate_velocity_tire_rotation_mps();
  calculate_tire_longitudinal_slip();
  calculate_tire_slip_angle_rad();
  calculate_tire_forces_tire_frame_N();
  calculate_tire_forces_N();
  calculate_suspension_spring_initial_compression_m();
  calculate_suspension_spring_compression_m();
  calculate_suspension_damper_compression_speed_mps();
  calculate_suspension_spring_force_N();
  calculate_suspension_damper_force_N();
  calculate_antiroll_bar_force_N();
  calculate_tire_rolling_resistance_N();
  calculate_axle_vertical_force_N();
  calculate_resulting_suspension_force_N();
  calculate_resulting_vertical_force_on_wheel_N();
  calculate_aerodynamics();
  calculate_resulting_force_N();
  calculate_resulting_torque_Nm();
  // clang-format on
}
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
void VehicleDynamicsDoubleTrackEqns<TireModelT, AeroModelT>::evaluate()
{
  // Calc the required forces and moments
  calculate_intermediate_results();
  calculate_drivetrain_load_torque_Nm();
  calculate_steering_load_torque_Nm();

  // According to Pacejka 2006 page 343, Eq. (7.18)
  auto pt1_T_relax_long = [](double vx) { return 0.3 / std::clamp(std::abs(vx), 10.0, 100.0); };
  auto pt1_T_relax_lat = [](double vx) { return 0.2 / std::clamp(std::abs(vx), 10.0, 100.0); };

  // State Equation
  // ============================================================================================
  // clang-format off
  // Vehicle Movement
  x_dot_vec_[x::x_m] = cos(x_vec_[x::psi_rad]) * x_vec_[x::v_x_mps] - sin(x_vec_[x::psi_rad]) * x_vec_[x::v_y_mps]; // NOLINT
  x_dot_vec_[x::y_m] = sin(x_vec_[x::psi_rad]) * x_vec_[x::v_x_mps] + cos(x_vec_[x::psi_rad]) * x_vec_[x::v_y_mps];  // NOLINT
  x_dot_vec_[x::psi_rad] = x_vec_[x::psi_dot_radps];  // psi_dot
  x_dot_vec_[x::v_x_mps] = imr_.resulting_force_N.x / p_.m + x_vec_[x::psi_dot_radps] * x_vec_[x::v_y_mps];  // NOLINT
  x_dot_vec_[x::v_y_mps] = imr_.resulting_force_N.y / p_.m - x_vec_[x::psi_dot_radps] * x_vec_[x::v_x_mps]; // NOLINT
  x_dot_vec_[x::psi_dot_radps] = imr_.resulting_torque_Nm.z / p_.I_z;
  // Vertical movement of vehicle body
  x_dot_vec_[x::z_m] = x_vec_[x::v_z_mps];
  x_dot_vec_[x::v_z_mps] = imr_.resulting_force_N.z / p_dep_.m_sprung;
  // Pitching an rolling
  x_dot_vec_[x::phi_rad] = x_vec_[x::phi_dot_radps];
  x_dot_vec_[x::phi_dot_radps] = imr_.resulting_torque_Nm.x / p_.I_x;
  x_dot_vec_[x::theta_rad] = x_vec_[x::theta_dot_radps];
  x_dot_vec_[x::theta_dot_radps] = imr_.resulting_torque_Nm.y / p_.I_y;
  // Wheel vertical dynamics
  x_dot_vec_[x::z_w_FL_m] = x_vec_[x::v_z_w_FL_mps];  // z_t_dot_LF
  x_dot_vec_[x::z_w_FR_m] = x_vec_[x::v_z_w_FR_mps];  // z_t_dot_RF
  x_dot_vec_[x::z_w_RL_m] = x_vec_[x::v_z_w_RL_mps];  // z_t_dot_LR
  x_dot_vec_[x::z_w_RR_m] = x_vec_[x::v_z_w_RR_mps];  // z_t_dot_RR
  x_dot_vec_[x::v_z_w_FL_mps] = imr_.resulting_vertical_force_on_wheel_N.front_left / p_.m_t_f;
  x_dot_vec_[x::v_z_w_FR_mps] = imr_.resulting_vertical_force_on_wheel_N.front_right / p_.m_t_f;
  x_dot_vec_[x::v_z_w_RL_mps] = imr_.resulting_vertical_force_on_wheel_N.rear_left / p_.m_t_r;
  x_dot_vec_[x::v_z_w_RR_mps] = imr_.resulting_vertical_force_on_wheel_N.rear_right / p_.m_t_r;
  // Tire delay
  x_dot_vec_[x::kappa_tire_FL] = (imr_.tire_longitudinal_slip.front_left  - x_vec_[x::kappa_tire_FL]) / pt1_T_relax_long(imr_.velocity_wheel_over_ground_tire_frame_mps.front_left[0]); // NOLINT
  x_dot_vec_[x::kappa_tire_FR] = (imr_.tire_longitudinal_slip.front_right - x_vec_[x::kappa_tire_FR]) / pt1_T_relax_long(imr_.velocity_wheel_over_ground_tire_frame_mps.front_right[0]); // NOLINT
  x_dot_vec_[x::kappa_tire_RL] = (imr_.tire_longitudinal_slip.rear_left   - x_vec_[x::kappa_tire_RL]) / pt1_T_relax_long(imr_.velocity_wheel_over_ground_tire_frame_mps.rear_left[0]); // NOLINT
  x_dot_vec_[x::kappa_tire_RR] = (imr_.tire_longitudinal_slip.rear_right  - x_vec_[x::kappa_tire_RR]) / pt1_T_relax_long(imr_.velocity_wheel_over_ground_tire_frame_mps.rear_right[0]); // NOLINT
  x_dot_vec_[x::alpha_tire_FL] = (imr_.tire_slip_angle_rad.front_left     - x_vec_[x::alpha_tire_FL]) / pt1_T_relax_lat(imr_.velocity_wheel_over_ground_tire_frame_mps.front_left[0]); // NOLINT
  x_dot_vec_[x::alpha_tire_FR] = (imr_.tire_slip_angle_rad.front_right    - x_vec_[x::alpha_tire_FR]) / pt1_T_relax_lat(imr_.velocity_wheel_over_ground_tire_frame_mps.front_right[0]); // NOLINT
  x_dot_vec_[x::alpha_tire_RL] = (imr_.tire_slip_angle_rad.rear_left      - x_vec_[x::alpha_tire_RL]) / pt1_T_relax_lat(imr_.velocity_wheel_over_ground_tire_frame_mps.rear_left[0]); // NOLINT
  x_dot_vec_[x::alpha_tire_RR] = (imr_.tire_slip_angle_rad.rear_right     - x_vec_[x::alpha_tire_RR]) / pt1_T_relax_lat(imr_.velocity_wheel_over_ground_tire_frame_mps.rear_right[0]); // NOLINT
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
  vd_output_.tire_longitudinal_slip_per_wheel.front_left = x_vec_[x::kappa_tire_FL];
  vd_output_.tire_longitudinal_slip_per_wheel.front_right = x_vec_[x::kappa_tire_FR];
  vd_output_.tire_longitudinal_slip_per_wheel.rear_left = x_vec_[x::kappa_tire_RL];
  vd_output_.tire_longitudinal_slip_per_wheel.rear_right = x_vec_[x::kappa_tire_RR];
  vd_output_.tire_slip_angle_per_wheel_rad.front_left = x_vec_[x::alpha_tire_FL];
  vd_output_.tire_slip_angle_per_wheel_rad.front_right = x_vec_[x::alpha_tire_FR];
  vd_output_.tire_slip_angle_per_wheel_rad.rear_left = x_vec_[x::alpha_tire_RL];
  vd_output_.tire_slip_angle_per_wheel_rad.rear_right = x_vec_[x::alpha_tire_RR];
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
