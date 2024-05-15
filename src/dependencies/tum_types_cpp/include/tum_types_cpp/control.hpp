// Copyright 2023 Simon Sagmeister
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tum_types_cpp/common.hpp"
namespace tam::types::control
{
// #region tracking_controller
// #region inputs
struct TrajectoryPoint
{
  tam::types::common::Vector3D<double> position_m;
  tam::types::common::Vector3D<double>
    orientation_rad;  // Euler Angles - therefore the assignement is (x:roll, y:pitch, z:yaw)
  tam::types::common::Vector3D<double>
    velocity_mps;  // In the "local" coordinate system speficied by position and orientation
  tam::types::common::Vector3D<double>
    angular_velocity_radps;  // In the "local" coordinate system speficied by
                             // position and orientation
  tam::types::common::Vector3D<double>
    acceleration_mps2;  // In the "local" coordinate system speficied by position and orientation
  tam::types::common::Vector3D<double>
    angular_acceleration_radps2;  // In the "local" coordinate system speficied by
                                  // position and orientation
  TrajectoryPoint() = default;
  TrajectoryPoint(
    tam::types::common::Vector3D<double> position_m_in,
    tam::types::common::Vector3D<double> orientation_rad_in,
    tam::types::common::Vector3D<double> velocity_mps_in,
    tam::types::common::Vector3D<double> angular_velocity_radps_in,
    tam::types::common::Vector3D<double> acceleration_mps2_in,
    tam::types::common::Vector3D<double> angular_acceleration_radps2_in)
  : position_m(position_m_in),
    orientation_rad(orientation_rad_in),
    velocity_mps(velocity_mps_in),
    angular_velocity_radps(angular_velocity_radps_in),
    acceleration_mps2(acceleration_mps2_in),
    angular_acceleration_radps2(angular_acceleration_radps2_in)
  {
  }
  TrajectoryPoint operator+(const TrajectoryPoint & other) const
  {
    return TrajectoryPoint(
      position_m + other.position_m, orientation_rad + other.orientation_rad,
      velocity_mps + other.velocity_mps, angular_velocity_radps + other.angular_velocity_radps,
      acceleration_mps2 + other.acceleration_mps2,
      angular_acceleration_radps2 + other.angular_acceleration_radps2);
  }
  TrajectoryPoint operator-(const TrajectoryPoint & other) const
  {
    return TrajectoryPoint(
      position_m - other.position_m, orientation_rad - other.orientation_rad,
      velocity_mps - other.velocity_mps, angular_velocity_radps - other.angular_velocity_radps,
      acceleration_mps2 - other.acceleration_mps2,
      angular_acceleration_radps2 - other.angular_acceleration_radps2);
  }
  // right multiplication with a factor
  friend TrajectoryPoint operator*(double factor, const TrajectoryPoint & obj)
  {
    return TrajectoryPoint(
      factor * obj.position_m, factor * obj.orientation_rad, factor * obj.velocity_mps,
      factor * obj.angular_velocity_radps, factor * obj.acceleration_mps2,
      factor * obj.angular_acceleration_radps2);
  }
  // left multiplication with a factor = right multiplication
  TrajectoryPoint operator*(const double & factor) const { return factor * (*this); }
};
struct Trajectory
{
  tam::types::common::Header header;
  std::vector<TrajectoryPoint> points;
};
struct Odometry
{
  tam::types::common::Vector3D<double> position_m;
  tam::types::common::Vector3D<double>
    orientation_rad;  // Euler Angles - therefore the assignement is (x:roll, y:pitch, z:yaw)
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
  std::array<double, 36> pose_covariance;  //  Row Major Matrix Representation

  tam::types::common::Vector3D<double>
    velocity_mps;  // In the "local" coordinate system speficied by position and orientation
  tam::types::common::Vector3D<double>
    angular_velocity_radps;  // In the "local" coordinate system speficied by
                             // position and orientation
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
  std::array<double, 36> velocity_covariance;  //  Row Major Matrix Representation
};
struct AccelerationwithCovariances
{
  tam::types::common::Vector3D<double>
    acceleration_mps2;  // In the "local" coordinate system speficied by position and orientation
  tam::types::common::Vector3D<double>
    angular_acceleration_radps2;  // In the "local" coordinate system speficied by
                                  // position and orientation
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
  std::array<double, 36> acceleration_covariance;  //  Row Major Matrix Representation
};
struct AutowareSteeringReport
{
  // Should probably not be used by a tracking controller but keep compatability
  uint64_t time_stamp_ns;
  double steering_angle_tire_rad;  // Desired angle of the steering tire in radians left (positive)
                                   // or right (negative) of center (0.0)
};

enum class AutowareOperationMode : uint8_t {
  // # constants for mode
  UNKNOWN = 0,
  STOP = 1,
  AUTONOMOUS = 2,
  LOCAL = 3,
  REMOTE = 4
};
struct AutowareOperationModeState
{
  // Is here but should probably not be used by other controllers that are not the autoware
  // controller
  uint64_t time_stamp_ns;
  AutowareOperationMode mode;
  bool is_autoware_control_enabled;
  bool is_in_transition;
  bool is_stop_mode_available;
  bool is_autonomous_mode_available;
  bool is_local_mode_available;
  bool is_remote_mode_available;
};
// #endregion

// #region outputs
// Inspired by
// https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Longitudinal.msg
struct LongitudinalControlCommand
{
  uint64_t time_stamp_ns;
  uint64_t control_time_ns;  // Time in which the requested value should be achieved - optional
  double velocity_mps;
  double acceleration_mps2;
  double jerk_mps3;
  bool is_defined_acceleration;  // Indicate whether the acceleration field is filled
  bool is_defined_jerk;          // Indicate whether the jerk field is filled.
};
// Inspired by
// https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Lateral.msg
struct LateralControlCommand
{
  uint64_t time_stamp_ns;
  uint64_t control_time_ns;  // Time in which the requested value should be achieved - optional
  double steering_angle_tire_rad;
  double steering_rotation_rate_tire_radps;
  bool is_defined_steering_rotation_rate;  // Indicate whether the rotation rate field is filled
};
// #endregion
// #region Additionally to the Autoware Interfaces
struct ControlConstraintPointAsPolygon
{
  std::vector<tam::types::common::Vector2D<double>> a_lim;
  double lateral_error_min_m;  // Tube Left
  double lateral_error_max_m;  // Tube right
};
template <typename T, size_t N>
std::array<T, N> operator+(const std::array<T, N> & ob1, const std::array<T, N> & ob2)
{
  std::array<T, N> res;
  for (int i = 0; i < N; ++i) res[i] = ob1[i] + ob2[i];
  return res;
}
template <typename T, size_t N>
std::array<T, N> operator*(double factor, const std::array<T, N> & obj)
{
  std::array<T, N> res;
  for (int i = 0; i < N; ++i) res[i] = factor * obj[i];
  return res;
}
struct ControlConstraintPoint
{
  tam::types::common::Vector2D<double> a_x_min_mps2;
  tam::types::common::Vector2D<double> a_x_max_mps2;
  tam::types::common::Vector2D<double> a_y_min_mps2;
  tam::types::common::Vector2D<double> a_y_max_mps2;
  double a_x_max_engine_mps2;  // Acceleration limited by the maximum available engine torque
  std::array<float, 4> shape_factor{1, 1, 1, 1};  // [acc_pos, dec_pos, dec_neg, acc_neg]
  double lateral_error_min_m;                     // Tube Left
  double lateral_error_max_m;
  ControlConstraintPoint() = default;
  ControlConstraintPoint(
    tam::types::common::Vector2D<double> a_x_min_mps2_in,
    tam::types::common::Vector2D<double> a_x_max_mps2_in,
    tam::types::common::Vector2D<double> a_y_min_mps2_in,
    tam::types::common::Vector2D<double> a_y_max_mps2_in, double a_x_max_engine_mps2_in,
    std::array<float, 4> shape_factor_in, double lateral_error_min_m_in,
    double lateral_error_max_m_in)
  : a_x_min_mps2{a_x_min_mps2_in},
    a_x_max_mps2{a_x_max_mps2_in},
    a_y_min_mps2{a_y_min_mps2_in},
    a_y_max_mps2{a_y_max_mps2_in},
    a_x_max_engine_mps2{a_x_max_engine_mps2_in},
    shape_factor{shape_factor_in},
    lateral_error_min_m{lateral_error_min_m_in},
    lateral_error_max_m{lateral_error_max_m_in}
  {
  }
  ControlConstraintPoint operator+(const ControlConstraintPoint & other) const
  {
    return ControlConstraintPoint(
      a_x_min_mps2 + other.a_x_min_mps2, a_x_max_mps2 + other.a_x_max_mps2,
      a_y_min_mps2 + other.a_y_min_mps2, a_y_max_mps2 + other.a_y_max_mps2,
      a_x_max_engine_mps2 + other.a_x_max_engine_mps2, shape_factor + other.shape_factor,
      lateral_error_min_m + other.lateral_error_min_m,
      lateral_error_max_m + other.lateral_error_max_m);
  }
  ControlConstraintPoint operator-(const ControlConstraintPoint & other) const
  {
    return ControlConstraintPoint(
      a_x_min_mps2 - other.a_x_min_mps2, a_x_max_mps2 - other.a_x_max_mps2,
      a_y_min_mps2 - other.a_y_min_mps2, a_y_max_mps2 - other.a_y_max_mps2,
      a_x_max_engine_mps2 - other.a_x_max_engine_mps2, shape_factor + (-1.0) * other.shape_factor,
      lateral_error_min_m - other.lateral_error_min_m,
      lateral_error_max_m - other.lateral_error_max_m);
  }
  // right multiplication with a factor
  friend ControlConstraintPoint operator*(double factor, const ControlConstraintPoint & obj)
  {
    return ControlConstraintPoint(
      factor * obj.a_x_min_mps2, factor * obj.a_x_max_mps2, factor * obj.a_y_min_mps2,
      factor * obj.a_y_max_mps2, factor * obj.a_x_max_engine_mps2, factor * obj.shape_factor,
      factor * obj.lateral_error_min_m, factor * obj.lateral_error_max_m);
  }
  // left multiplication with a factor = right multiplication
  ControlConstraintPoint operator*(const double & factor) const
  {
    return factor * (*this);
  }  // Tube right
};
struct ControlConstraints
{
  tam::types::common::Header header;
  std::vector<ControlConstraintPoint> points;
};
struct ControlConstraintsPolygon
{
  tam::types::common::Header header;
  std::vector<ControlConstraintPointAsPolygon> points;
};

struct AdditionalInfoPoint
{
  double s_global_m;
  double s_local_m;
  double kappa_1pm;
  double lap_cnt;
  AdditionalInfoPoint() = default;
  AdditionalInfoPoint(
    double s_global_m_in, double s_local_m_in, double kappa_1pm_in, double lap_cnt_in)
  : s_global_m(s_global_m_in), s_local_m(s_local_m_in), kappa_1pm(kappa_1pm_in), lap_cnt(lap_cnt_in)
  {
  }
  AdditionalInfoPoint operator+(const AdditionalInfoPoint & other) const
  {
    return AdditionalInfoPoint(
      s_global_m + other.s_global_m, s_local_m + other.s_local_m, kappa_1pm + other.kappa_1pm,
      lap_cnt + other.lap_cnt);
  }
  AdditionalInfoPoint operator-(const AdditionalInfoPoint & other) const
  {
    return AdditionalInfoPoint(
      s_global_m - other.s_global_m, s_local_m - other.s_local_m, kappa_1pm - other.kappa_1pm,
      lap_cnt - other.lap_cnt);
  }
  // right multiplication with a factor
  friend AdditionalInfoPoint operator*(double factor, const AdditionalInfoPoint & obj)
  {
    return AdditionalInfoPoint(
      factor * obj.s_global_m, factor * obj.s_local_m, factor * obj.kappa_1pm,
      factor * obj.lap_cnt);
  }
  // left multiplication with a factor = right multiplication
  AdditionalInfoPoint operator*(const double & factor) const { return factor * (*this); }
};
struct AdditionalTrajectoryInfos
{
  tam::types::common::Header header;
  std::vector<AdditionalInfoPoint> points;
};
// #endregion
// #endregion

// #region Longitudinal Control
struct ICECommand
{
  double throttle;  // [0,1]
  tam::types::common::DataPerWheel<double> brake_pressure_Pa;
  uint8_t gear;
};
struct DriveTrainFeedback
{
  double omega_engine_radps;  // Engine rpm
  int8_t gear_engaged;
};
struct EngineTorques
{
  double T_max_Nm;
  double T_min_Nm;
  double T_30percThrottle_Nm;
};
// #endregion
constexpr size_t mpc_horizon_length = 40;
constexpr size_t planner_traj_length = 50;
constexpr size_t limit_pts = 8;
constexpr size_t error_msg_size = 3;
// see also TUMControlAcadosDebug.msg
struct acados_mpc_debug
{
  double dot_d_analytical_mps;
  double dot_d_numerical_mps;
  double v_terminal_mps;
  double error_state[error_msg_size];
  double s_current_m;
  double d_current_m;
  double x_real_m;
  double y_real_m;
  double delta_psi_real_rad;
  double psi_dot_real_radps;
  double ax_real_mps2;
  double ay_real_mps2;
  double vx_real_mps;
  double vy_real_mps;
  double beta_real_rad;
  double throttle_real;
  double brake_real;
  double steering_angle_real_rad;

  // solver stats, timing
  double solver_status;
  unsigned int solver_qp_iter;
  unsigned int solver_sqp_iter;
  double res_stat;
  double res_eq;
  double res_ineq;
  double res_comp;
  double cost_value;
  double time_acados_solve_s;
  double time_total_controller_step_s;

  // resampled planner traj
  double x_traj_m[mpc_horizon_length + 1];
  double y_traj_m[mpc_horizon_length + 1];
  double psi_traj_rad[mpc_horizon_length + 1];
  double v_traj_mps[mpc_horizon_length + 1];
  double kappa_traj_radpm[mpc_horizon_length + 1];
  double ax_diff_traj_mps2m[mpc_horizon_length + 1];
  double ax_traj_mps2[mpc_horizon_length + 1];
  double ay_traj_mps2[mpc_horizon_length + 1];

  double flag_s_request;

  // planner target trajectory
  unsigned int tartraj_lapcnt;
  unsigned int tartraj_trajcnt;
  double tartraj_s_loc_m[planner_traj_length];
  double tartraj_s_glob_m[planner_traj_length];
  double tartraj_x_m[planner_traj_length];
  double tartraj_y_m[planner_traj_length];
  double tartraj_psi_rad[planner_traj_length];
  double tartraj_kappa_radpm[planner_traj_length];
  double tartraj_v_mps[planner_traj_length];
  double tartraj_ax_mps2[planner_traj_length];
  double tartraj_ay_mps2[planner_traj_length];
  double tartraj_banking_rad[planner_traj_length];
  double tartraj_ax_lim_max_x_mps2[planner_traj_length];
  double tartraj_ax_lim_max_y_mps2[planner_traj_length];
  double tartraj_ax_lim_min_x_mps2[planner_traj_length];
  double tartraj_ax_lim_min_y_mps2[planner_traj_length];
  double tartraj_ay_lim_max_x_mps2[planner_traj_length];
  double tartraj_ay_lim_max_y_mps2[planner_traj_length];
  double tartraj_ay_lim_min_x_mps2[planner_traj_length];
  double tartraj_ay_lim_min_y_mps2[planner_traj_length];

  // limits resampled
  double ax_lim_max_x_mps2[mpc_horizon_length + 1];
  double ax_lim_max_y_mps2[mpc_horizon_length + 1];
  double ax_lim_min_x_mps2[mpc_horizon_length + 1];
  double ax_lim_min_y_mps2[mpc_horizon_length + 1];
  double ay_lim_max_x_mps2[mpc_horizon_length + 1];
  double ay_lim_max_y_mps2[mpc_horizon_length + 1];
  double ay_lim_min_x_mps2[mpc_horizon_length + 1];
  double ay_lim_min_y_mps2[mpc_horizon_length + 1];
  double d_lim_ub_m[mpc_horizon_length + 1];
  double d_lim_lb_m[mpc_horizon_length + 1];

  // MPC prediction (Solver solution)
  double x_pred_m[mpc_horizon_length + 1];
  double y_pred_m[mpc_horizon_length + 1];
  double x_pred_left_m[mpc_horizon_length + 1];
  double y_pred_left_m[mpc_horizon_length + 1];
  double x_pred_right_m[mpc_horizon_length + 1];
  double y_pred_right_m[mpc_horizon_length + 1];
  double vx_pred_mps[mpc_horizon_length + 1];
  double vy_pred_mps[mpc_horizon_length + 1];
  double delta_psi_pred_rad[mpc_horizon_length + 1];
  double psi_dot_pred_radps[mpc_horizon_length + 1];
  double sigma_pred[mpc_horizon_length + 1];
  double d_pred_m[mpc_horizon_length + 1];
  double d_dot_pred_mps[mpc_horizon_length + 1];
  double s_dot_pred_mps[mpc_horizon_length + 1];
  double steering_angle_pred_rad[mpc_horizon_length + 1];
  double thottle_pred[mpc_horizon_length + 1];
  double brake_pred[mpc_horizon_length + 1];
  // inputs
  double ax_pred_mps2[mpc_horizon_length + 1];
  double ay_pred_mps2[mpc_horizon_length + 1];
  double steering_angle_rate_input_pred_radps[mpc_horizon_length + 1];
  double throttle_rate_input_pred[mpc_horizon_length + 1];
  double brake_rate_input_pred[mpc_horizon_length + 1];

  double tire_utilization_target;
  double s_dot_lin_mps[mpc_horizon_length + 1];
  double vx_lin_mps[mpc_horizon_length + 1];
  double kappa_lin_radpm[mpc_horizon_length + 1];
};
};  // namespace tam::types::control
