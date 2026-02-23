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

#include "ocd_drivetrain_wheel_torque_cpp/states.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::ocd::drivetrain
{
class DrivetrainEquationsDirectTorque
{
  friend class DrivetrainWheelTorqueModel;
  //
  using x = tam::ocd::drivetrain::wheel_torque::States::StateEnum;
  using state_vector_t = Eigen::Matrix<double, x::CNT_LENGTH_STATE_VECTOR, 1>;
  using double_per_wheel_t = tam::types::common::DataPerWheel<double>;
  //
  struct DriverInput
  {
    tam::types::common::DataPerWheel<double> drivetrain_input_torque_per_wheel_Nm{0, 0, 0, 0};
  };
  struct Feedback
  {
    tam::types::common::DataPerWheel<double> drivetrain_input_torque_per_wheel_Nm{0, 0, 0, 0};
  };
  struct Parameters
  {
    double I_engine_kgm2;
    double I_wheel_front_kgm2;
    double I_wheel_rear_kgm2;
  };
  //
  Parameters p;
  state_vector_t x_vec_;
  DriverInput driver_input_;
  double_per_wheel_t drivetrain_load_torque_per_wheel_Nm_;
  //
  // Calculated by evaluate
  state_vector_t x_dot_vec_;
  // Intermediate results - Also calculated during the evaluate
  double I_wheel_low_speed_kgm2;  // Add additional inertia at low speeds to avoid
                                  // oscillations due to integration
  bool bool_clamp_xdot_to_avoid_reverse;
  //
  // Function to calc intermediate results.
  void calc_I_wheel_low_speed_kgm2();
  //
public:
  DrivetrainEquationsDirectTorque() = default;
  //
  // Setters
  void set_x_vec(const state_vector_t x_vec) { x_vec_ = x_vec; }
  void set_driver_input(const DriverInput & input) { driver_input_ = input; }
  void set_load(const double_per_wheel_t & drivetrain_load_torque_per_wheel_Nm)
  {
    drivetrain_load_torque_per_wheel_Nm_ = drivetrain_load_torque_per_wheel_Nm;
  };
  //
  // Evaluate the ODE
  void evaluate();
  //
};
}  // namespace tam::ocd::drivetrain
