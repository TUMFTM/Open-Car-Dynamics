// Copyright 2023 Simon Sagmeister

#include "vehicle_dynamics_double_track_cpp/vehicle_dynamics_equations.hpp"

#define EVAL_MACRO_PER_TIRE(x) \
  x(front_left);               \
  x(front_right);              \
  x(rear_left);                \
  x(rear_right)
namespace tam::sim::vd_double_track
{
void VehicleDynamicsDoubleTrackEqns::calculate_dependent_parameters()
{
  p_dep.l_r = p.l - p.l_f;
  p_dep.m_sprung = p.m - 2 * p.m_t_f - 2 * p.m_t_r;
  // Calculate the cog for the unsprung mass
  p_dep.l_r_sprung = (p.m * p_dep.l_r - 2 * p.m_t_f * p.l) / p_dep.m_sprung;
  p_dep.l_f_sprung = p.l - p_dep.l_r_sprung;
  // Calculate the parameters used for squat force calc
  p_dep.slf = (p.h_rc / (0.5 * p.b_f));
  p_dep.slr = (p.h_rc / (0.5 * p.b_r));
  p_dep.sadf_deceleration = (p.h_pc_decel / (p.l_f));
  p_dep.sadr_deceleration = (p.h_pc_decel / (p_dep.l_r));
  p_dep.sadf_acceleration = (p.h_pc_accel / (p.l_f));
  p_dep.sadr_acceleration = (p.h_pc_accel / (p_dep.l_r));
}
void VehicleDynamicsDoubleTrackEqns::calculate_tire_spring_initial_compression_m()
{
  // Tire spring compression @ standstill
  imr.tire_spring_initial_compression_m.front_left =
    (p.m * G_CONST * p_dep.l_r / (2 * p.l)) / p.c_t_f;
  imr.tire_spring_initial_compression_m.rear_left = (p.m * G_CONST * p.l_f / (2 * p.l)) / p.c_t_r;
  imr.tire_spring_initial_compression_m.front_right =
    imr.tire_spring_initial_compression_m.front_left;
  imr.tire_spring_initial_compression_m.rear_right =
    imr.tire_spring_initial_compression_m.rear_left;
}
void VehicleDynamicsDoubleTrackEqns::calculate_tire_spring_force_N()
{  // Tire spring forces
  imr.tire_spring_force_N.front_left =
    p.c_t_f * (x_vec[x::z_w_FL_m] - imr.tire_spring_initial_compression_m.front_left);
  imr.tire_spring_force_N.front_right =
    p.c_t_f * (x_vec[x::z_w_FR_m] - imr.tire_spring_initial_compression_m.front_right);
  imr.tire_spring_force_N.rear_left =
    p.c_t_r * (x_vec[x::z_w_RL_m] - imr.tire_spring_initial_compression_m.rear_left);
  imr.tire_spring_force_N.rear_right =
    p.c_t_r * (x_vec[x::z_w_RR_m] - imr.tire_spring_initial_compression_m.rear_right);
}
void VehicleDynamicsDoubleTrackEqns::calculate_F_z_tire_N()
{
  // Input to tire model force
  imr.F_z_tire_N.front_left = -imr.tire_spring_force_N.front_left;
  imr.F_z_tire_N.front_right = -imr.tire_spring_force_N.front_right;
  imr.F_z_tire_N.rear_left = -imr.tire_spring_force_N.rear_left;
  imr.F_z_tire_N.rear_right = -imr.tire_spring_force_N.rear_right;
}
void VehicleDynamicsDoubleTrackEqns::calculate_antiroll_bar_force_N()
{
  // Anti-Roll bar forces
  imr.antiroll_bar_force_N.front_left = p.c_ar_f * (x_vec[x::z_w_FR_m] - x_vec[x::z_w_FL_m]);
  imr.antiroll_bar_force_N.rear_left = p.c_ar_r * (x_vec[x::z_w_RR_m] - x_vec[x::z_w_RL_m]);
  imr.antiroll_bar_force_N.front_right = -imr.antiroll_bar_force_N.front_left;
  imr.antiroll_bar_force_N.rear_right = -imr.antiroll_bar_force_N.rear_left;
}
void VehicleDynamicsDoubleTrackEqns::calculate_velocity_wheel_over_ground_mps()
{  // Tire Velocity Vector @ x - y axis
  imr.velocity_wheel_over_ground_mps.front_left = {
    x_vec[x::v_x_mps] - p.b_f * x_vec[x::psi_dot_radps] / 2,
    x_vec[x::v_y_mps] + p.l_f * x_vec[x::psi_dot_radps]};
  imr.velocity_wheel_over_ground_mps.front_right = {
    x_vec[x::v_x_mps] + p.b_f * x_vec[x::psi_dot_radps] / 2,
    x_vec[x::v_y_mps] + p.l_f * x_vec[x::psi_dot_radps]};
  imr.velocity_wheel_over_ground_mps.rear_left = {
    x_vec[x::v_x_mps] - p.b_r * x_vec[x::psi_dot_radps] / 2,
    x_vec[x::v_y_mps] - p_dep.l_r * x_vec[x::psi_dot_radps]};
  imr.velocity_wheel_over_ground_mps.rear_right = {
    x_vec[x::v_x_mps] + p.b_r * x_vec[x::psi_dot_radps] / 2,
    x_vec[x::v_y_mps] - p_dep.l_r * x_vec[x::psi_dot_radps]};
}
void VehicleDynamicsDoubleTrackEqns::calculate_velocity_wheel_over_ground_tire_frame_mps()
{
  // clang-format off
  Eigen::Matrix2d rot_matrix{
    {cos(u.steering_angle_tire_rad), sin(u.steering_angle_tire_rad)},
    {-sin(u.steering_angle_tire_rad), cos(u.steering_angle_tire_rad)}
  };
  // clang-format on

  // Rotate the wheel velocity over ground into the front vehicles coordinate frame
  imr.velocity_wheel_over_ground_tire_frame_mps.front_left =
    rot_matrix * imr.velocity_wheel_over_ground_mps.front_left;
  imr.velocity_wheel_over_ground_tire_frame_mps.front_right =
    rot_matrix * imr.velocity_wheel_over_ground_mps.front_right;
  // Nothing for the rea wheel, since they align with the vehicles coordinate system
  // regarding orientation
  imr.velocity_wheel_over_ground_tire_frame_mps.rear_left =
    imr.velocity_wheel_over_ground_mps.rear_left;
  imr.velocity_wheel_over_ground_tire_frame_mps.rear_right =
    imr.velocity_wheel_over_ground_mps.rear_right;
}
void VehicleDynamicsDoubleTrackEqns::calculate_velocity_tire_rotation_mps()
{
  // Velocity due to tire roll @ x - y axis:
  imr.velocity_tire_rotation_mps.front_left = p.rr_w_f * u.omega_wheel_radps.front_left;
  imr.velocity_tire_rotation_mps.front_right = p.rr_w_f * u.omega_wheel_radps.front_right;
  imr.velocity_tire_rotation_mps.rear_left = p.rr_w_r * u.omega_wheel_radps.rear_left;
  imr.velocity_tire_rotation_mps.rear_right = p.rr_w_r * u.omega_wheel_radps.rear_right;
}
void VehicleDynamicsDoubleTrackEqns::calculate_tire_longitudinal_slip()
{
  constexpr static auto long_slip_def =
    [](double rotational_velocity_mps, double vel_over_ground_mps) {
      return std::clamp(
        (rotational_velocity_mps - vel_over_ground_mps) / std::max(3.0, vel_over_ground_mps), -1.0,
        1.0);
    };

#define CALC_LONG_SLIP(x)                       \
  imr.tire_longitudinal_slip.x = long_slip_def( \
    imr.velocity_tire_rotation_mps.x, imr.velocity_wheel_over_ground_tire_frame_mps.x[0]);

  EVAL_MACRO_PER_TIRE(CALC_LONG_SLIP);
#undef CALC_LONG_SLIP
}
void VehicleDynamicsDoubleTrackEqns::calculate_tire_slip_angle_rad()
{
  auto slip_angle_def = [this](double vy, double vx) { return -atan2(vy, std::max(1.0, vx)); };

// clang-format off
  #define CALC_TIRE_SLIP(x)                               \
  imr.tire_slip_angle_rad.x = slip_angle_def(           \
    imr.velocity_wheel_over_ground_tire_frame_mps.x[1], \
    imr.velocity_wheel_over_ground_tire_frame_mps.x[0])

  EVAL_MACRO_PER_TIRE(CALC_TIRE_SLIP);

  #undef CALC_TIRE_SLIP
  // clang-format on
}
void VehicleDynamicsDoubleTrackEqns::calculate_tire_forces_tire_frame_N()
{
  // Zero out tire forces at stillstand
  if (
    std::pow(x_vec[x::v_x_mps], 2) < 0.001 && std::pow(x_vec[x::v_y_mps], 2) < 0.001 &&
    std::pow(u.omega_wheel_radps.front_left, 2) < 0.001 &&
    std::pow(u.omega_wheel_radps.front_right, 2) < 0.001 &&
    std::pow(u.omega_wheel_radps.rear_left, 2) < 0.001 &&
    std::pow(u.omega_wheel_radps.rear_right, 2) < 0.001) {
    imr.tire_forces_tire_frame_N.front_right = {0, 0};
    imr.tire_forces_tire_frame_N.front_left = {0, 0};
    imr.tire_forces_tire_frame_N.rear_left = {0, 0};
    imr.tire_forces_tire_frame_N.rear_right = {0, 0};
    return;
  }

  auto tire_model_eval = [this](
                           double F_z_N, double long_slip, double slip_angle_rad,
                           decltype(p.tire.front_left) const & params, Eigen::Vector2d & target) {
    auto out = tire_model({long_slip, slip_angle_rad, F_z_N}, params);
    target[0] = out.longitudinal_force_N;
    target[1] = out.lateral_force_N;
  };

  // Use the delayed kappa and alpha
  double_per_wheel_t kappa, alpha;
  kappa.front_left = x_vec[x::kappa_tire_FL];
  kappa.front_right = x_vec[x::kappa_tire_FR];
  kappa.rear_left = x_vec[x::kappa_tire_RL];
  kappa.rear_right = x_vec[x::kappa_tire_RR];
  alpha.front_left = x_vec[x::alpha_tire_FL];
  alpha.front_right = x_vec[x::alpha_tire_FR];
  alpha.rear_left = x_vec[x::alpha_tire_RL];
  alpha.rear_right = x_vec[x::alpha_tire_RR];
  // clang-format off
  #define EVAL_TIRE_MODEL(x)   \
  tire_model_eval( \
    imr.F_z_tire_N.x, \
    kappa.x, \
    alpha.x, \
    p.tire.x, \
    imr.tire_forces_tire_frame_N.x \
  )
  EVAL_MACRO_PER_TIRE(EVAL_TIRE_MODEL);
  #undef EVAL_TIRE_MODEL
  // clang-format on
}
void VehicleDynamicsDoubleTrackEqns::calculate_tire_forces_N()
{
  // Tire Forces @ x - y axis of the vehicle
  // clang-format off
  Eigen::Matrix2d rot_matrix{
    {cos(-u.steering_angle_tire_rad), sin(-u.steering_angle_tire_rad)},
    {-sin(-u.steering_angle_tire_rad), cos(-u.steering_angle_tire_rad)}
  };
  // clang-format on
  imr.tire_forces_N.front_left = rot_matrix * imr.tire_forces_tire_frame_N.front_left;
  imr.tire_forces_N.front_right = rot_matrix * imr.tire_forces_tire_frame_N.front_right;
  // The rear tire can be used as is
  imr.tire_forces_N.rear_left = imr.tire_forces_tire_frame_N.rear_left;
  imr.tire_forces_N.rear_right = imr.tire_forces_tire_frame_N.rear_right;
}
void VehicleDynamicsDoubleTrackEqns::calculate_suspension_spring_initial_compression_m()
{
  // Compression of suspension coils @ standstill
  imr.suspension_spring_initial_compression_m.front_left =
    p_dep.m_sprung * G_CONST * p_dep.l_r_sprung / (2 * p.c_f * p.l);
  imr.suspension_spring_initial_compression_m.rear_left =
    p_dep.m_sprung * G_CONST * p_dep.l_f_sprung / (2 * p.c_r * p.l);
  imr.suspension_spring_initial_compression_m.front_right =
    imr.suspension_spring_initial_compression_m.front_left;
  imr.suspension_spring_initial_compression_m.rear_right =
    imr.suspension_spring_initial_compression_m.rear_left;
}
void VehicleDynamicsDoubleTrackEqns::calculate_suspension_spring_compression_m()
{
  imr.suspension_spring_compression_m.front_left =
    -imr.suspension_spring_initial_compression_m.front_left + x_vec[x::z_m] +
    x_vec[x::phi_rad] * p.b_f / 2 - x_vec[x::theta_rad] * p_dep.l_f_sprung - x_vec[x::z_w_FL_m];
  imr.suspension_spring_compression_m.front_right =
    -imr.suspension_spring_initial_compression_m.front_right + x_vec[x::z_m] -
    x_vec[x::phi_rad] * p.b_f / 2 - x_vec[x::theta_rad] * p_dep.l_f_sprung - x_vec[x::z_w_FR_m];
  imr.suspension_spring_compression_m.rear_left =
    -imr.suspension_spring_initial_compression_m.rear_left + x_vec[x::z_m] +
    x_vec[x::phi_rad] * p.b_r / 2 + x_vec[x::theta_rad] * p_dep.l_r_sprung - x_vec[x::z_w_RL_m];
  imr.suspension_spring_compression_m.rear_right =
    -imr.suspension_spring_initial_compression_m.rear_right + x_vec[x::z_m] -
    x_vec[x::phi_rad] * p.b_r / 2 + x_vec[x::theta_rad] * p_dep.l_r_sprung - x_vec[x::z_w_RR_m];
}
void VehicleDynamicsDoubleTrackEqns::calculate_suspension_damper_compression_speed_mps()
{
  // Push/ Pull of suspension dampers
  imr.suspension_damper_compression_speed_mps.front_left =
    x_vec[x::v_z_mps] + x_vec[x::phi_dot_radps] * p.b_f / 2 -
    x_vec[x::theta_dot_radps] * p_dep.l_f_sprung - x_vec[x::v_z_w_FL_mps];
  imr.suspension_damper_compression_speed_mps.front_right =
    x_vec[x::v_z_mps] - x_vec[x::phi_dot_radps] * p.b_f / 2 -
    x_vec[x::theta_dot_radps] * p_dep.l_f_sprung - x_vec[x::v_z_w_FR_mps];
  imr.suspension_damper_compression_speed_mps.rear_left =
    x_vec[x::v_z_mps] + x_vec[x::phi_dot_radps] * p.b_r / 2 +
    x_vec[x::theta_dot_radps] * p_dep.l_r_sprung - x_vec[x::v_z_w_RL_mps];
  imr.suspension_damper_compression_speed_mps.rear_right =
    x_vec[x::v_z_mps] - x_vec[x::phi_dot_radps] * p.b_r / 2 +
    x_vec[x::theta_dot_radps] * p_dep.l_r_sprung - x_vec[x::v_z_w_RR_mps];
}
void VehicleDynamicsDoubleTrackEqns::calculate_suspension_spring_force_N()
{
  // Suspension spring forces
  imr.suspension_spring_force_N.front_left = p.c_f * imr.suspension_spring_compression_m.front_left;
  imr.suspension_spring_force_N.front_right =
    p.c_f * imr.suspension_spring_compression_m.front_right;
  imr.suspension_spring_force_N.rear_left = p.c_r * imr.suspension_spring_compression_m.rear_left;
  imr.suspension_spring_force_N.rear_right = p.c_r * imr.suspension_spring_compression_m.rear_right;
}
void VehicleDynamicsDoubleTrackEqns::calculate_suspension_damper_force_N()
{
  // Suspension damper forces
  imr.suspension_damper_force_N.front_left =
    p.d_f * imr.suspension_damper_compression_speed_mps.front_left;
  imr.suspension_damper_force_N.front_right =
    p.d_f * imr.suspension_damper_compression_speed_mps.front_right;
  imr.suspension_damper_force_N.rear_left =
    p.d_r * imr.suspension_damper_compression_speed_mps.rear_left;
  imr.suspension_damper_force_N.rear_right =
    p.d_r * imr.suspension_damper_compression_speed_mps.rear_right;
}
void VehicleDynamicsDoubleTrackEqns::calculate_tire_rolling_resistance_N()
{
// Always positive since always going forward
#define CALC_TIRE_RR(x_in) imr.tire_rolling_resistance_N.x_in = imr.F_z_tire_N.x_in * p.c_rr;
  EVAL_MACRO_PER_TIRE(CALC_TIRE_RR);
#undef CALC_TIRE_RR
}
void VehicleDynamicsDoubleTrackEqns::calculate_axle_vertical_force_N()
{  // Total Axle Forces
  // See page A-30
  // https://babel.hathitrust.org/cgi/pt?id=mdp.39015075298698&view=1up&seq=147&skin=2021
  auto calc_axle_force_front =
    [this](double F_x, double F_y, double K_sli, double K_sadi, double K_sadi2) {
      double k_sadi_used = (F_x > 0.0) ? K_sadi2 : K_sadi;
      return (K_sli * F_y + k_sadi_used * F_x);
    };
  auto calc_axle_force_rear =
    [this](double F_x, double F_y, double K_sli, double K_sadi, double K_sadi2) {
      double k_sadi_used = (F_x > 0.0) ? K_sadi2 : K_sadi;
      return (K_sli * F_y - k_sadi_used * F_x);
    };

  imr.axle_vertical_force_N.front_left = calc_axle_force_front(
    imr.tire_forces_N.front_left[0], imr.tire_forces_N.front_left[1], p_dep.slf,
    p_dep.sadf_deceleration, p_dep.sadf_acceleration);
  imr.axle_vertical_force_N.front_right = calc_axle_force_front(
    imr.tire_forces_N.front_right[0], -imr.tire_forces_N.front_right[1], p_dep.slf,
    p_dep.sadf_deceleration, p_dep.sadf_acceleration);
  imr.axle_vertical_force_N.rear_left = calc_axle_force_rear(
    imr.tire_forces_N.rear_left[0], imr.tire_forces_N.rear_left[1], p_dep.slr,
    p_dep.sadr_deceleration, p_dep.sadr_acceleration);
  imr.axle_vertical_force_N.rear_right = calc_axle_force_rear(
    imr.tire_forces_N.rear_right[0], -imr.tire_forces_N.rear_right[1], p_dep.slr,
    p_dep.sadr_deceleration, p_dep.sadr_acceleration);
}
void VehicleDynamicsDoubleTrackEqns::calculate_resulting_suspension_force_N()
{
  // clang-format off
  #define VERTICAL_FORCES_SUSPENSION(tire_in)                                        \
    imr.resulting_suspension_force_N.tire_in = \
    (  imr.suspension_spring_force_N.tire_in \
    + imr.suspension_damper_force_N.tire_in \
    + imr.antiroll_bar_force_N.tire_in \
    + imr.axle_vertical_force_N.tire_in \
    )
  // Use macro
  EVAL_MACRO_PER_TIRE(VERTICAL_FORCES_SUSPENSION);
  // Undef used macro
  #undef VERTICAL_FORCES_SUSPENSION
  // clang-format on
}
void VehicleDynamicsDoubleTrackEqns::calculate_resulting_vertical_force_on_wheel_N()
{
  // Heave of Tire
  // clang-format off
  #define CALC_HEAVE_TIRE(tire_name, tire_mass) \
  imr.resulting_vertical_force_on_wheel_N.tire_name = \
    + imr.resulting_suspension_force_N.tire_name \
    - imr.tire_spring_force_N.tire_name \
    - tire_mass * G_CONST

  CALC_HEAVE_TIRE(front_left, p.m_t_f);
  CALC_HEAVE_TIRE(front_right, p.m_t_f);
  CALC_HEAVE_TIRE(rear_left, p.m_t_r);
  CALC_HEAVE_TIRE(rear_right, p.m_t_r);

  #undef CALC_HEAVE_TIRE
  // clang-format on
}
void VehicleDynamicsDoubleTrackEqns::calculate_resulting_force_N()
{
  imr.resulting_force_N.x = imr.tire_forces_N.front_left[0] + imr.tire_forces_N.front_right[0] +
                            imr.tire_forces_N.rear_left[0] + imr.tire_forces_N.rear_right[0] +
                            w.external_forces_cog_N.x;

  // Sum of tire forces @ y axis
  imr.resulting_force_N.y = imr.tire_forces_N.front_left[1] + imr.tire_forces_N.front_right[1] +
                            imr.tire_forces_N.rear_left[1] + imr.tire_forces_N.rear_right[1] +
                            w.external_forces_cog_N.y;
  // Heave of Car
  // clang-format off
  imr.resulting_force_N.z =
    - imr.resulting_suspension_force_N.front_left
    - imr.resulting_suspension_force_N.front_right
    - imr.resulting_suspension_force_N.rear_left
    - imr.resulting_suspension_force_N.rear_right
    - p_dep.m_sprung * G_CONST
    + w.external_forces_cog_N.z;
  // clang-format on
}
void VehicleDynamicsDoubleTrackEqns::calculate_resulting_torque_Nm()
{
  // Sum of tire moments @ z axis
  // clang-format off
  imr.resulting_torque_Nm.z =
      (imr.tire_forces_N.front_left[1] + imr.tire_forces_N.front_right[1]) * p.l_f
    + (imr.tire_forces_N.front_right[0] - imr.tire_forces_N.front_left[0]) * p.b_f / 2
    + (imr.tire_forces_N.rear_right[0] - imr.tire_forces_N.rear_left[0]) * p.b_r / 2
    - (imr.tire_forces_N.rear_left[1] + imr.tire_forces_N.rear_right[1]) * p_dep.l_r
    + w.external_moments_cog_Nm.z;


  // Macro to sum of the forces of the spring, damper and antirollbar
  imr.resulting_torque_Nm.y =
    - (imr.tire_forces_N.front_left[0] + imr.tire_forces_N.front_right[0] +
       imr.tire_forces_N.rear_left[0] + imr.tire_forces_N.rear_right[0]
      ) * p.h_cg
    + imr.resulting_suspension_force_N.front_left * p.l_f
    + imr.resulting_suspension_force_N.front_right * p.l_f
    - imr.resulting_suspension_force_N.rear_left * p_dep.l_r
    - imr.resulting_suspension_force_N.rear_right* p_dep.l_r
    + p_dep.m_sprung* G_CONST * (p.l_f - p_dep.l_f_sprung)
    + w.external_moments_cog_Nm.y;

  // Sum of sprung mass moments @ x axis
  imr.resulting_torque_Nm.x = (imr.tire_forces_N.front_left[1] + imr.tire_forces_N.front_right[1] +
       imr.tire_forces_N.rear_left[1] + imr.tire_forces_N.rear_right[1]
      ) * p.h_cg
    - imr.resulting_suspension_force_N.front_left * p.b_f / 2
    + imr.resulting_suspension_force_N.front_right * p.b_f / 2
    - imr.resulting_suspension_force_N.rear_left * p.b_r / 2
    + imr.resulting_suspension_force_N.rear_right * p.b_r / 2
    + w.external_moments_cog_Nm.x;
  // clang-format on
}
void VehicleDynamicsDoubleTrackEqns::calculate_intermediate_results()
{
  calculate_dependent_parameters();
  calculate_tire_spring_initial_compression_m();
  calculate_tire_spring_force_N();
  calculate_F_z_tire_N();
  calculate_antiroll_bar_force_N();
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
  calculate_tire_rolling_resistance_N();
  calculate_axle_vertical_force_N();
  calculate_resulting_suspension_force_N();
  calculate_resulting_vertical_force_on_wheel_N();
  calculate_resulting_force_N();
  calculate_resulting_torque_Nm();
  // clang-format on
}
void VehicleDynamicsDoubleTrackEqns::evaluate()
{
  // Calc the required forces and moments
  calculate_intermediate_results();

  // According to Pacejka 2006 page 343, Eq. (7.18)
  auto pt1_T_relax_long = [](double vx) { return 0.3 / std::clamp(std::abs(vx), 10.0, 100.0); };
  auto pt1_T_relax_lat = [](double vx) { return 0.2 / std::clamp(std::abs(vx), 10.0, 100.0); };

  // State Equation
  // ============================================================================================
  // clang-format off
  // Vehicle Movement
  x_dot_vec[x::x_m] = cos(x_vec[x::psi_rad]) * x_vec[x::v_x_mps] - sin(x_vec[x::psi_rad]) * x_vec[x::v_y_mps]; // NOLINT
  x_dot_vec[x::y_m] = sin(x_vec[x::psi_rad]) * x_vec[x::v_x_mps] + cos(x_vec[x::psi_rad]) * x_vec[x::v_y_mps];  // NOLINT
  x_dot_vec[x::psi_rad] = x_vec[x::psi_dot_radps];  // psi_dot
  x_dot_vec[x::v_x_mps] = imr.resulting_force_N.x / p.m + x_vec[x::psi_dot_radps] * x_vec[x::v_y_mps];  // NOLINT
  x_dot_vec[x::v_y_mps] = imr.resulting_force_N.y / p.m - x_vec[x::psi_dot_radps] * x_vec[x::v_x_mps]; // NOLINT
  x_dot_vec[x::psi_dot_radps] = imr.resulting_torque_Nm.z / p.I_z;
  // Vertical movement of vehicle body
  x_dot_vec[x::z_m] = x_vec[x::v_z_mps];
  x_dot_vec[x::v_z_mps] = imr.resulting_force_N.z / p_dep.m_sprung;
  // Pitching an rolling
  x_dot_vec[x::phi_rad] = x_vec[x::phi_dot_radps];
  x_dot_vec[x::phi_dot_radps] = imr.resulting_torque_Nm.x / p.I_x;
  x_dot_vec[x::theta_rad] = x_vec[x::theta_dot_radps];
  x_dot_vec[x::theta_dot_radps] = imr.resulting_torque_Nm.y / p.I_y;
  // Wheel vertical dynamics
  x_dot_vec[x::z_w_FL_m] = x_vec[x::v_z_w_FL_mps];  // z_t_dot_LF
  x_dot_vec[x::z_w_FR_m] = x_vec[x::v_z_w_FR_mps];  // z_t_dot_RF
  x_dot_vec[x::z_w_RL_m] = x_vec[x::v_z_w_RL_mps];  // z_t_dot_LR
  x_dot_vec[x::z_w_RR_m] = x_vec[x::v_z_w_RR_mps];  // z_t_dot_RR
  x_dot_vec[x::v_z_w_FL_mps] = imr.resulting_vertical_force_on_wheel_N.front_left / p.m_t_f;
  x_dot_vec[x::v_z_w_FR_mps] = imr.resulting_vertical_force_on_wheel_N.front_right / p.m_t_f;
  x_dot_vec[x::v_z_w_RL_mps] = imr.resulting_vertical_force_on_wheel_N.rear_left / p.m_t_r;
  x_dot_vec[x::v_z_w_RR_mps] = imr.resulting_vertical_force_on_wheel_N.rear_right / p.m_t_r;
  // Tire delay
  x_dot_vec[x::kappa_tire_FL] = (imr.tire_longitudinal_slip.front_left  - x_vec[x::kappa_tire_FL]) / pt1_T_relax_long(imr.velocity_wheel_over_ground_tire_frame_mps.front_left[0]); // NOLINT
  x_dot_vec[x::kappa_tire_FR] = (imr.tire_longitudinal_slip.front_right - x_vec[x::kappa_tire_FR]) / pt1_T_relax_long(imr.velocity_wheel_over_ground_tire_frame_mps.front_right[0]); // NOLINT
  x_dot_vec[x::kappa_tire_RL] = (imr.tire_longitudinal_slip.rear_left   - x_vec[x::kappa_tire_RL]) / pt1_T_relax_long(imr.velocity_wheel_over_ground_tire_frame_mps.rear_left[0]); // NOLINT
  x_dot_vec[x::kappa_tire_RR] = (imr.tire_longitudinal_slip.rear_right  - x_vec[x::kappa_tire_RR]) / pt1_T_relax_long(imr.velocity_wheel_over_ground_tire_frame_mps.rear_right[0]); // NOLINT
  x_dot_vec[x::alpha_tire_FL] = (imr.tire_slip_angle_rad.front_left     - x_vec[x::alpha_tire_FL]) / pt1_T_relax_lat(imr.velocity_wheel_over_ground_tire_frame_mps.front_left[0]); // NOLINT
  x_dot_vec[x::alpha_tire_FR] = (imr.tire_slip_angle_rad.front_right    - x_vec[x::alpha_tire_FR]) / pt1_T_relax_lat(imr.velocity_wheel_over_ground_tire_frame_mps.front_right[0]); // NOLINT
  x_dot_vec[x::alpha_tire_RL] = (imr.tire_slip_angle_rad.rear_left      - x_vec[x::alpha_tire_RL]) / pt1_T_relax_lat(imr.velocity_wheel_over_ground_tire_frame_mps.rear_left[0]); // NOLINT
  x_dot_vec[x::alpha_tire_RR] = (imr.tire_slip_angle_rad.rear_right     - x_vec[x::alpha_tire_RR]) / pt1_T_relax_lat(imr.velocity_wheel_over_ground_tire_frame_mps.rear_right[0]); // NOLINT
  // clang-format on

  // Output Equation
  // ============================================================================================
  // clang-format off
  #define CALC_TIRE_ROT_TORQUE(in_tire, tire_radius)                                      \
    y.tire_torque_rotational_Nm.in_tire =                                                 \
      (imr.tire_forces_tire_frame_N.in_tire[0] + imr.tire_rolling_resistance_N.in_tire) * \
      tire_radius

  CALC_TIRE_ROT_TORQUE(front_left, p.rr_w_f);
  CALC_TIRE_ROT_TORQUE(front_right, p.rr_w_f);
  CALC_TIRE_ROT_TORQUE(rear_left, p.rr_w_r);
  CALC_TIRE_ROT_TORQUE(rear_right, p.rr_w_r);

  #undef CALC_TIRE_ROT_TORQUE
  // clang-format on
};
}  // namespace tam::sim::vd_double_track
#undef EVAL_MACRO_PER_TIRE
