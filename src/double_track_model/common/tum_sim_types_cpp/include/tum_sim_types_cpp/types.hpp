// Copyright 2023 Simon Sagmeister
#pragma once
#include <vector>

#include "tum_types_cpp/common.hpp"
namespace tam::types
{
struct DriverInput
{
  double steering_angle_rad;  // Left is positive
  double ax_target;
};
struct ExternalInfluences
{
  common::Vector3D<double> external_force_N;    // On the vehicle body
  common::Vector3D<double> external_torque_Nm;  // On the vehicle body
  common::DataPerWheel<double> z_height_road_m;
  common::DataPerWheel<double> lambda_mue;
};
struct ActuatorInput
{
  double steering_angle_request;
  double brake_pressure_request;
};
struct ActuatorOutput
{
  double steering_angle;
  double brake_pressure;
};
struct DriveTrainInput
{
  double throttle;  // 0 to 1
  double brake_pressure_bar;
  common::DataPerWheel<double>
    long_tire_force_N;  // Necessary to calc the angular acceleration of the indivial wheels
  uint8_t gear;
};
struct DriveTrainOutput
{
  common::DataPerWheel<double> drive_torque_Nm;
  common::DataPerWheel<double> wheel_speed_radps;
};
struct VehicleDynamisModelInput
{
  common::DataPerWheel<double> wheel_speed_radps;
  double steering_angle;
};
struct VehicleDynamicsModelOutput
{
  // Odometry
  common::Vector3D<double> position_m;
  common::Vector3D<double> velocity_mps;
  common::Vector3D<double> acceleration_mps2;
  common::Vector3D<double> orientation_rad;
  common::Vector3D<double> angular_velocity_radps;
  common::Vector3D<double> angular_accelaration_radps2;
  // Long Tire Forces
  common::DataPerWheel<double> long_tire_force_N;
};
struct VehicleModelOutput
{
  double steering_angle;                                 // From actuation model
  common::DataPerWheel<double> wheel_speed_radps;        // From drivetrain model
  common::Vector3D<double> position_m;                   // From vehicle dynamics model
  common::Vector3D<double> velocity_mps;                 // From vehicle dynamics model
  common::Vector3D<double> acceleration_mps2;            // From vehicle dynamics model
  common::Vector3D<double> orientation_rad;              // From vehicle dynamics model
  common::Vector3D<double> angular_velocity_radps;       // From vehicle dynamics model
  common::Vector3D<double> angular_accelaration_radps2;  // From vehicle dynamics model
};
}  // namespace tam::types
