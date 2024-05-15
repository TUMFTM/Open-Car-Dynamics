// Copyright 2023 Simon Sagmeister

#pragma once
#include <math.h>

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>

#include "param_manager_cpp/param_manager.hpp"
#include "param_manager_cpp/param_manager_base.hpp"
#include "tum_types_cpp/common.hpp"
#include "vehicle_dynamics_helpers_cpp/tire_models.hpp"
namespace tam::sim::vd_double_track
{
namespace x
{
enum States {
  x_m,        // x position of the vehicle in m
  y_m,        // y position of the vehicle in m
  z_m,        // z position of the vehicle in m
  psi_rad,    //  Yaw angle of the vehicle in rad
  phi_rad,    // Roll angle of the vehicle body in rad
  theta_rad,  // Pitch angle of the vehicle in rad
  // Velocity of the vehicles COG in the longitudinal direction of the vehicle
  v_x_mps,
  // Velocity of the vehicles COG in the lateral direction of the vehicle
  v_y_mps,
  // Velocity of the vehicles COG in the vertical direction of the vehicle
  v_z_mps,
  psi_dot_radps,    // Yaw rate of the vehicle in radps
  phi_dot_radps,    // Roll rate of the vehicle in radps
  theta_dot_radps,  // Pitch rate of the vehicle in radps
  z_w_FL_m,         // Vertical position of the front left tire
  z_w_FR_m,         // Vertical position of the front right tire
  z_w_RL_m,         // Vertical position of the rear left tire
  z_w_RR_m,         // Vertical position of the rear right tire
  v_z_w_FL_mps,     // Vertical velocity of the front left tire
  v_z_w_FR_mps,     // Vertical velocity of the front right tire
  v_z_w_RL_mps,     // Vertical velocity of the rear left tire
  v_z_w_RR_mps,     // Vertical velocity of the rear right tire
  kappa_tire_FL,    // Tire forces
  kappa_tire_FR,    // Tire forces
  kappa_tire_RL,    // Tire forces
  kappa_tire_RR,    // Tire forces
  alpha_tire_FL,    // Tire forces
  alpha_tire_FR,    // Tire forces
  alpha_tire_RL,    // Tire forces
  alpha_tire_RR,    // Tire forces
  CNT_LENGTH_STATE_VECTOR
};
constexpr const char * TO_STRING(int enum_value)
{
  // clang-format off
  switch (enum_value) {
    case States::x_m: return "x_m";
    case States::y_m: return "y_m";
    case States::z_m: return "z_m";
    case States::psi_rad: return "psi_rad";
    case States::phi_rad: return "phi_rad";
    case States::theta_rad: return "theta_rad";
    case States::v_x_mps: return "v_x_mps";
    case States::v_y_mps: return "v_y_mps";
    case States::v_z_mps: return "v_z_mps";
    case States::psi_dot_radps: return "psi_dot_radps";
    case States::phi_dot_radps: return "phi_dot_radps";
    case States::theta_dot_radps: return "theta_dot_radps";
    case States::z_w_FL_m: return "z_w_FL_m";
    case States::z_w_FR_m: return "z_w_FR_m";
    case States::z_w_RL_m: return "z_w_RL_m";
    case States::z_w_RR_m: return "z_w_RR_m";
    case States::v_z_w_FL_mps: return "v_z_w_FL_mps";
    case States::v_z_w_FR_mps: return "v_z_w_FR_mps";
    case States::v_z_w_RL_mps: return "v_z_w_RL_mps";
    case States::v_z_w_RR_mps: return "v_z_w_RR_mps";
    case States::kappa_tire_FR: return "kappa_tire_FR";
    case States::kappa_tire_FL: return "kappa_tire_FL";
    case States::kappa_tire_RL: return "kappa_tire_RL";
    case States::kappa_tire_RR: return "kappa_tire_RR";
    case States::alpha_tire_FL: return "alpha_tire_FL";
    case States::alpha_tire_FR: return "alpha_tire_FR";
    case States::alpha_tire_RL: return "alpha_tire_RL";
    case States::alpha_tire_RR: return "alpha_tire_RR";
    // Just here so silence compiler warnings
    case States::CNT_LENGTH_STATE_VECTOR: return "NOT_DEFINED";
  }
  // Just here so silence compiler warnings
  return "NOT_DEFINED";
  // clang-format on
};
}  // namespace x
typedef Eigen::Matrix<double, x::CNT_LENGTH_STATE_VECTOR, 1> state_vector_t;
typedef tam::types::common::DataPerWheel<double> double_per_wheel_t;
typedef tam::types::common::DataPerWheel<Eigen::Vector2d> double_vector_per_wheel_t;
struct Input
{
  tam::types::common::DataPerWheel<double> omega_wheel_radps;
  double steering_angle_tire_rad;  // Left is positive
};
struct ExternalInfluences
{
  // External forces on the cog
  tam::types::common::Vector3D<double> external_forces_cog_N;
  tam::types::common::Vector3D<double> external_moments_cog_Nm;
};
struct Output
{
  tam::types::common::DataPerWheel<double> tire_torque_rotational_Nm;
};
template <typename T>
struct Parameters
{
  tam::types::common::DataPerWheel<T> tire;

  // Masses & Inertial moments
  double m;      // [kg]
  double m_t_f;  // [kg]
  double m_t_r;  // [kg]
  double I_z;    // [kg m^2]
  double I_y;    // [kg m^2]
  double I_x;    // [kg m^2]

  // Geometric parameters
  double l_f;     // [m]
  double l;       // [m]
  double b_f;     // [m]
  double b_r;     // [m]
  double h_cg;    // [m]
  double rr_w_f;  // [m]
  double rr_w_r;  // [m]
  // Height of the pitch center during acceleration
  double h_pc_accel;  // [m]
  // Height of the pitch center during acceleration
  double h_pc_decel;  // [m]
  // Height of the roll center
  double h_rc;  // [m]

  // Spring & Damping constants
  double c_f;    // [N/m]
  double c_r;    // [N/m]
  double d_f;    // [N*s/m]
  double d_r;    // [N*s/m]
  double c_t_f;  // [N/m]
  double c_t_r;  // [N/m]

  // Anti-Roll bar
  double c_ar_f;  // [N/m] *
  double c_ar_r;  // [N/m]

  // Rolling resistance
  double c_rr;  // https://en.wikipedia.org/wiki/Rolling_resistance
};
// Variables that are directly calculated from the given parameters
struct DependentParameters
{
  double l_r;
  double m_sprung;    // Sprung mass
  double l_r_sprung;  // Describing COG of the sprung mass
  double l_f_sprung;  // Describing COG of the sprung mass
  // Coefficient for calculating anti squat and anti dive forces from the axle
  // See page A-30 of
  // https://babel.hathitrust.org/cgi/pt?id=mdp.39015075298698&view=1up&seq=146&skin=2021
  double slf;
  double slr;
  double sadf_deceleration;
  double sadr_deceleration;
  double sadf_acceleration;
  double sadr_acceleration;
};
struct IntermediateResults
{
  // Tire spring compression at standstill
  double_per_wheel_t tire_spring_initial_compression_m;
  double_per_wheel_t tire_spring_force_N;  // Tire spring forces
  double_per_wheel_t F_z_tire_N;  // Tire load in the contact patch. Used for the tire model.
  double_per_wheel_t antiroll_bar_force_N;  // Antiroll bar forces
  // The actual velocity vector of the wheel in the contact patchoriginated in the vehicles
  // movement "Velocity over ground"
  double_vector_per_wheel_t velocity_wheel_over_ground_mps;
  // The velocity vector of each tire rotated in each tires coordinate system
  double_vector_per_wheel_t velocity_wheel_over_ground_tire_frame_mps;
  double_per_wheel_t velocity_tire_rotation_mps;
  // Long slip and slip angle for each tire
  double_per_wheel_t tire_longitudinal_slip;
  double_per_wheel_t tire_slip_angle_rad;
  // Tire forces in longitudinal and lateral direction of each tire
  double_vector_per_wheel_t tire_forces_tire_frame_N;
  // Tire forces in longitudinal and lateral direction of the vehicle
  double_vector_per_wheel_t tire_forces_N;
  // Suspension spring compression at standstill
  double_per_wheel_t suspension_spring_initial_compression_m;
  double_per_wheel_t suspension_spring_compression_m;
  // Compression speed of the damper in mps
  double_per_wheel_t suspension_damper_compression_speed_mps;
  // Forces in the suspension springs
  double_per_wheel_t suspension_spring_force_N;
  // Forces in the dampers
  double_per_wheel_t suspension_damper_force_N;
  // Torque on the tire because of rolling resistance
  // They won't be explicitly considered for calculating the lateral acceleration
  // since they will be considered as additional torque
  // keeping the tire from spinning faster
  double_per_wheel_t tire_rolling_resistance_N;
  // Resulting vertical axle force
  double_per_wheel_t axle_vertical_force_N;
  // Resulting force from suspension
  double_per_wheel_t resulting_suspension_force_N;
  // Resulting veritcal force on the wheel
  double_per_wheel_t resulting_vertical_force_on_wheel_N;
  // Resulting forces
  tam::types::common::Vector3D<double> resulting_force_N;    // On the vehicle
  tam::types::common::Vector3D<double> resulting_torque_Nm;  // On the vehicle body
};
class VehicleDynamicsDoubleTrackEqns
{
  // Types
  using TireModel_t = tam::sim::helpers::tire_models::MF52;

public:
  // Inputs - Write this variables
  Input u;
  state_vector_t x_vec;
  ExternalInfluences w;
  Parameters<TireModel_t::Parameters> p;
  DependentParameters p_dep;

  // Outputs - Read this variables
  Output y;
  state_vector_t x_dot_vec;
  IntermediateResults imr;

  // Evaluate the ODE
  void evaluate();

private:
  // Tire Model
  TireModel_t tire_model;

  // Constants
  static constexpr double G_CONST = 9.81;  // mps2
  double_per_wheel_t tire_spring_initial_compression;

  void calculate_intermediate_results();
  // Function to calculate pars of the intermediate results.
  void calculate_dependent_parameters();
  void calculate_tire_spring_initial_compression_m();
  void calculate_tire_spring_force_N();
  void calculate_F_z_tire_N();
  void calculate_antiroll_bar_force_N();
  void calculate_velocity_wheel_over_ground_mps();
  void calculate_velocity_wheel_over_ground_tire_frame_mps();
  void calculate_velocity_tire_rotation_mps();
  void calculate_tire_longitudinal_slip();
  void calculate_tire_slip_angle_rad();
  void calculate_tire_forces_tire_frame_N();
  void calculate_tire_forces_N();
  void calculate_suspension_spring_initial_compression_m();
  void calculate_suspension_spring_compression_m();
  void calculate_suspension_damper_compression_speed_mps();
  void calculate_suspension_spring_force_N();
  void calculate_suspension_damper_force_N();
  void calculate_tire_rolling_resistance_N();
  void calculate_axle_vertical_force_N();
  void calculate_resulting_suspension_force_N();
  void calculate_resulting_vertical_force_on_wheel_N();
  void calculate_resulting_force_N();
  void calculate_resulting_torque_Nm();
};
}  // namespace tam::sim::vd_double_track
