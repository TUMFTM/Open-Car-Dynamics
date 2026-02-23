// Copyright 2026 Simon Sagmeister

#pragma once
#include <math.h>

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ocd_drivetrain_rwd_lsd_cpp/states.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::ocd::drivetrain
{
//
class DrivetrainEquations_RWD_LSD
{
  friend class DrivetrainModel_RWD_LSD;
  using x = tam::ocd::drivetrain::rwd_lsd::States::StateEnum;
  //
  using state_vector_t = Eigen::Matrix<double, x::CNT_LENGTH_STATE_VECTOR, 1>;
  using double_per_wheel_t = tam::types::common::DataPerWheel<double>;
  //
  struct DriverInput
  {
    double transmission_output_torque_Nm{0};          // Torque on the rear axle
    double current_engine_inertia_at_wheels_kgm2{0};  // Intertia of the engine
    double_per_wheel_t brake_torque_per_wheel_Nm{0};  // Unit: Nm
  };
  struct Parameters
  {
    double I_wheel_front_kgm2;
    double I_wheel_rear_kgm2;
    double final_drive_ratio;
    double LSD_preload;
    double LSD_lock_drive;
    double LSD_lock_coast;
  };
  //
  Parameters p;
  state_vector_t x_vec_;
  DriverInput drivetrain_input_;
  double_per_wheel_t drivetrain_load_torque_per_wheel_Nm_;

  // Calculated by evaluate function
  state_vector_t x_dot_vec_;
  // The torque output of the transmission on the rear axle
  double M_LSD_Nm;
  double I_wheel_low_speed_kgm2;  // Add additional inertia at low speeds to avoid
                                  // oscillations due to integration
  //
  // Function to calc intermediate results.
  void calc_I_wheel_low_speed_kgm2();
  void calc_LSD_torque_Nm();
  //
public:
  DrivetrainEquations_RWD_LSD() = default;
  //
  // Setters
  void set_x_vec(const state_vector_t x_vec) { x_vec_ = x_vec; }
  void set_driver_input(const DriverInput & input) { drivetrain_input_ = input; }
  void set_load(const double_per_wheel_t & drivetrain_load_torque_per_wheel_Nm)
  {
    drivetrain_load_torque_per_wheel_Nm_ = drivetrain_load_torque_per_wheel_Nm;
  };
  // Evaluate the ODE
  void evaluate();
  //
};
}  // namespace tam::ocd::drivetrain
