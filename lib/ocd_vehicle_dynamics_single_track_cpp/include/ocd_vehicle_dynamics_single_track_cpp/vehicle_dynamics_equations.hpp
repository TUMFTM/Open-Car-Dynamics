// Copyright 2026 Simon Sagmeister

#pragma once
#include <math.h>

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ocd_aerodynamics_model_base_cpp/concept.hpp"
#include "ocd_tire_model_base_cpp/concept.hpp"
#include "ocd_types_cpp/types.hpp"
#include "ocd_vehicle_dynamics_model_base_cpp/base_class.hpp"
#include "ocd_vehicle_dynamics_single_track_cpp/states.hpp"
#include "tum_helpers_cpp/constants.hpp"
#include "tum_helpers_cpp/numerical.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::ocd::vehicle_dynamics
{
// Forward declaration of the friend class
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
class VehicleDynamicsSingleTrackModel;
template <
  tam::ocd::interfaces::concepts::TireModel TireModelT,
  tam::ocd::interfaces::concepts::AerodynamicsModel AeroModelT>
class VehicleDynamicsSingleTrackEqns
{
  friend class VehicleDynamicsSingleTrackModel<TireModelT, AeroModelT>;
  // Types
  using x = tam::ocd::vehicle_dynamics::single_track::States::StateEnum;
  using StateVectorType = Eigen::Matrix<double, x::CNT_LENGTH_STATE_VECTOR, 1>;
  using double_per_wheel_t = tam::types::common::DataPerWheel<double>;
  using vector2d_per_wheel_t = tam::types::common::DataPerWheel<Eigen::Vector2d>;
  using vector3d_t = tam::types::common::Vector3D<double>;
  //
  struct Input
  {
    double_per_wheel_t wheel_speeds_radps{0, 0, 0, 0};       // Wheel speeds in rad/s
    double_per_wheel_t steering_angle_tire_rad{0, 0, 0, 0};  // Left is positive
  };
  struct Parameters
  {
    // Masses & Inertial moments
    double m;    // [kg]
    double I_z;  // [kg m^2]
    // Geometric parameters
    double l_f;     // [m]
    double l;       // [m]
    double rr_w_f;  // [m]
    double rr_w_r;  // [m]
    // Rolling resistance
    double c_rr;  // https://en.wikipedia.org/wiki/Rolling_resistance
    // Tire Radii velocity scaling
    std::vector<double> rr_vel_scale_vel_points_mps;
    std::vector<double> rr_vel_scale_scale_factors;
  };
  // Variables that are directly calculated from the given parameters
  struct DependentParameters
  {
    double l_r = 0.0;
  };
  struct IntermediateResults
  {
    // Effective steering angles (no toe in single track)
    double_per_wheel_t effective_steering_angle_per_wheel_rad{0, 0, 0, 0};
    // Static vertical tire force
    double_per_wheel_t vertical_tire_force_N{0, 0, 0, 0};
    // The actual velocity vector of the wheel in the contact patch
    // originated in the vehicles movement "Velocity over ground"
    vector2d_per_wheel_t velocity_wheel_over_ground_mps{
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero()};
    // The velocity vector of each tire rotated in each tires coordinate system
    vector2d_per_wheel_t velocity_wheel_over_ground_tire_frame_mps{
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero()};
    double_per_wheel_t velocity_tire_rotation_mps{0, 0, 0, 0};
    // Long slip and slip angle for each tire
    double_per_wheel_t tire_longitudinal_slip{0, 0, 0, 0};
    double_per_wheel_t tire_slip_angle_rad{0, 0, 0, 0};
    // Tire forces in longitudinal and lateral direction of each tire
    vector2d_per_wheel_t tire_forces_tire_frame_N{
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero()};
    vector2d_per_wheel_t tire_forces_N{
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero()};
    // Torque on the tire because of rolling resistance
    double_per_wheel_t tire_rolling_resistance_N{0, 0, 0, 0};
    // Aerodynamics
    vector3d_t aero_force_N{0, 0, 0};    // On the vehicle
    vector3d_t aero_torque_Nm{0, 0, 0};  // On the vehicle body
    // Resulting forces
    vector3d_t resulting_force_N{0, 0, 0};    // On the vehicle
    vector3d_t resulting_torque_Nm{0, 0, 0};  // On the vehicle body
    // Dynamic Tire Radius
    double_per_wheel_t dynamic_tire_radius_m{0, 0, 0, 0};
  };
  //
  double_per_wheel_t wheel_speeds_radps_{0, 0, 0, 0};
  double_per_wheel_t steering_angle_per_wheel_rad_{0, 0, 0, 0};
  //
  StateVectorType x_vec_ = StateVectorType::Zero();
  StateVectorType x_dot_vec_ = StateVectorType::Zero();
  //
  double_per_wheel_t drivetrain_load_torque_per_wheel_Nm_{0, 0, 0, 0};
  double_per_wheel_t steering_load_torque_per_wheel_Nm_{0, 0, 0, 0};
  types::VehicleDynamicsModelOutput vd_output_{};
  //
  IntermediateResults imr_{};
  //
  types::ExternalInfluences external_influences_{};
  tam::types::common::DataPerWheel<TireModelT> tire_models_;
  AeroModelT aero_model_;
  Parameters p_;
  DependentParameters p_dep_{};
  //
  // Constants
  static constexpr double G_CONST = 9.81;  // mps2
  //
  void calculate_intermediate_results();
  // Function to calculate parts of the intermediate results.
  void calc_dynamic_tire_radius();
  void calculate_effective_steering_angle();
  void calculate_dependent_parameters();
  void calculate_vertical_tire_force_N();
  void calculate_velocity_wheel_over_ground_mps();
  void calculate_velocity_wheel_over_ground_tire_frame_mps();
  void calculate_velocity_tire_rotation_mps();
  void calculate_tire_longitudinal_slip();
  void calculate_tire_slip_angle_rad();
  void calculate_tire_forces_tire_frame_N();
  void calculate_tire_forces_N();
  void calculate_tire_rolling_resistance_N();
  void calculate_aerodynamics();
  void calculate_resulting_force_N();
  void calculate_resulting_torque_Nm();
  void calculate_drivetrain_load_torque_Nm();
  void calculate_steering_load_torque_Nm();
  //
public:
  void set_wheel_speeds(const double_per_wheel_t & wheel_speeds_radps)
  {
    wheel_speeds_radps_ = wheel_speeds_radps;
  }
  void set_steering_angles(const double_per_wheel_t & steering_angle_per_wheel_rad)
  {
    steering_angle_per_wheel_rad_ = steering_angle_per_wheel_rad;
  }
  void set_external_influences(const types::ExternalInfluences & external_influences)
  {
    external_influences_ = external_influences;
  }
  void set_x_vec(const StateVectorType & x_vec) { x_vec_ = x_vec; }
  //
  // Evaluate the ODE
  void evaluate();
};
}  // namespace tam::ocd::vehicle_dynamics
#include "ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_equations_impl.hpp"
