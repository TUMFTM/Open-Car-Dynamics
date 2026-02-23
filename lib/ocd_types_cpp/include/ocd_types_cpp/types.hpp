// Copyright 2026 Simon Sagmeister
#pragma once
#include <vector>

#include "tum_types_cpp/common.hpp"
namespace tam::ocd::types
{
struct ExternalInfluences
{
  tam::types::common::Vector3D<double> external_force_N = {0, 0, 0};    // On the vehicle body
  tam::types::common::Vector3D<double> external_torque_Nm = {0, 0, 0};  // On the vehicle body
  tam::types::common::DataPerWheel<double> z_height_road_m = {0, 0, 0, 0};
  tam::types::common::DataPerWheel<double> lambda_mue = {1, 1, 1, 1};
};
struct VehicleDynamicsModelOutput
{
  // Odometry
  tam::types::common::Vector3D<double> position_m = {0, 0, 0};
  tam::types::common::Vector3D<double> velocity_mps = {0, 0, 0};
  tam::types::common::Vector3D<double> acceleration_mps2 = {0, 0, 0};
  tam::types::common::Vector3D<double> orientation_rad = {0, 0, 0};
  tam::types::common::Vector3D<double> angular_velocity_radps = {0, 0, 0};
  tam::types::common::Vector3D<double> angular_acceleration_radps2 = {0, 0, 0};
  tam::types::common::DataPerWheel<double> tire_longitudinal_slip_per_wheel = {0, 0, 0, 0};
  tam::types::common::DataPerWheel<double> tire_slip_angle_per_wheel_rad = {0, 0, 0, 0};
  tam::types::common::DataPerWheel<double> longitudinal_tire_force_tire_frame_per_wheel_N = {
    0, 0, 0, 0};
  tam::types::common::DataPerWheel<double> lateral_tire_force_tire_frame_per_wheel_N = {0, 0, 0, 0};
  tam::types::common::DataPerWheel<double> vertical_tire_force_per_wheel_N = {0, 0, 0, 0};
};
struct VehicleModelOutput
{
  VehicleDynamicsModelOutput vehicle_dynamics_output;
  tam::types::common::DataPerWheel<double> wheel_speeds_radps;
  tam::types::common::DataPerWheel<double> drivetrain_load_torque_per_wheel_Nm;
  tam::types::common::DataPerWheel<double> steering_angle_per_wheel_rad;
  tam::types::common::DataPerWheel<double> steering_load_torque_per_wheel_Nm;
};
struct AeroModelOutput
{
  tam::types::common::Vector3D<double> force_cog_N;
  tam::types::common::Vector3D<double> torque_Nm;
};
struct AeroModelInput
{
  double vx_mps;           // Velocity in x direction
  double vy_mps;           // Velocity in y direction
  double pitch_angle_rad;  // Pitch angle of the vehicle
  double z_m;              // Heave from neutral position
};
struct TireModelOutput
{
  double longitudinal_force_N;     // Tire force in longitudinal direction
  double lateral_force_N;          // Tire force in lateral direction
  double self_aligning_moment_Nm;  // Self aligning moment
};
struct TireModelInput
{
  double longitudinal_slip;  // Slip ratio in longitudinal direction
  double slip_angle_rad;     // Slip angle at the respective tire
  double F_z_N;              // Normal force in the tire road contact patch
  double gamma_rad;  // Positive angle means clockwise rotation locking from the rear of the car.
};
}  // namespace tam::ocd::types
