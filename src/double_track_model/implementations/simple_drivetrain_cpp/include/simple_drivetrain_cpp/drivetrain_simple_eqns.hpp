// Copyright 2023 Simon Sagmeister

#pragma once
#include <math.h>

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "param_manager_cpp/param_manager.hpp"
#include "param_manager_cpp/param_manager_base.hpp"
#include "tum_sim_types_cpp/types.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::sim::drivetrain
{
namespace x
{
enum States {
  omega_FL_radps,
  omega_FR_radps,
  omega_RL_radps,
  omega_RR_radps,
  a_x,
  CNT_LENGTH_STATE_VECTOR
};
constexpr const char * TO_STRING(uint8_t enum_value)
{
  // clang-format off
  switch (enum_value) {
    case States::omega_FL_radps: return "omega_FL_radps";
    case States::omega_FR_radps: return "omega_FR_radps";
    case States::omega_RL_radps: return "omega_RL_radps";
    case States::omega_RR_radps: return "omega_RR_radps";
    case States::a_x: return "a_x";
    // Just here so silence compiler warnings
    case States::CNT_LENGTH_STATE_VECTOR: return "NOT_DEFINED";
  }
  // clang-format on
  throw std::runtime_error("ERROR: Invalid enum value");
};
}  // namespace x
typedef Eigen::Matrix<double, x::CNT_LENGTH_STATE_VECTOR, 1> state_vector_t;
struct Input
{
  double ax_target;
  tam::types::common::DataPerWheel<double> tire_torque_rotational_Nm;  // Unit: Newtonmeter
};
struct Output
{
  tam::types::common::DataPerWheel<double> omega_wheel_radps;
};
class DriveTrainEquationsSimple
{
  friend class DriveTrainModelSimple;
  // Definitions for the class namespace
public:
  typedef tam::types::common::DataPerWheel<double> double_per_wheel_t;
  struct Parameters
  {
    double m_vehicle_kg;
    double rr_tire_front;  // Tire rolling radius at the front
    double rr_tire_rear;   // Tire rolling radius at the front
    double I_wheel_front_kgm2;
    double I_wheel_rear_kgm2;
    // "brake balance" percent goes to the front [1-BrakeBias_Front] to the rear. [0,1]
    double BrakeBias_Front;
    // "acceleration" percent goes to the front [1-AccelBias_Front] to the rear. [0,1]
    double AccelBias_Front;
    double T_PT1_accel;
  };

private:
  // Set from outside
  Input u;
  Parameters p;
  state_vector_t x_vec;

  // Calculated by evaluate
  state_vector_t x_dot_vec;
  // Add additional inertia at low speeds to avoid oscilattions due to integration
  double_per_wheel_t I_wheel_low_speed_kgm2;
  double_per_wheel_t dt_torque_Nm;
  double external_fx;

  // Function to calc intermediate results.
  void calc_I_wheel_low_speed_kgm2();
  double calc_I_wheel_low_speed_per_tire_kgm2(double omega_tire_radps);

public:
  DriveTrainEquationsSimple() = default;
  void set_u(Input u_) { u = u_; }
  void set_x_vec(state_vector_t x_vec_) { x_vec = x_vec_; }
  void set_external_influences(tam::types::ExternalInfluences const & external_influences)
  {
    external_fx = external_influences.external_force_N.x;
  }
  // Evaluate the ODE
  void evaluate();
  // Getters - Call after evaluate
};
}  // namespace tam::sim::drivetrain
