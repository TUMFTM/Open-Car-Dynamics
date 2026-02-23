// Copyright 2026 Simon Sagmeister
#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
namespace tam::ocd::vehicle_dynamics::double_track::States
{
// clang-format off
#define STATE_LIST(X) \
  X(x_m) \
  X(y_m) \
  X(z_m) \
  X(psi_rad) \
  X(phi_rad) \
  X(theta_rad) \
  X(v_x_mps) \
  X(v_y_mps) \
  X(v_z_mps) \
  X(psi_dot_radps) \
  X(phi_dot_radps) \
  X(theta_dot_radps) \
  X(z_w_FL_m) \
  X(z_w_FR_m) \
  X(z_w_RL_m) \
  X(z_w_RR_m) \
  X(v_z_w_FL_mps) \
  X(v_z_w_FR_mps) \
  X(v_z_w_RL_mps) \
  X(v_z_w_RR_mps) \
  X(kappa_tire_FL) \
  X(kappa_tire_FR) \
  X(kappa_tire_RL) \
  X(kappa_tire_RR) \
  X(alpha_tire_FL) \
  X(alpha_tire_FR) \
  X(alpha_tire_RL) \
  X(alpha_tire_RR) \
// clang-format on

/*
  x_m              ->  x position of the vehicle in m
  y_m              ->  y position of the vehicle in m
  z_m              ->  z position of the vehicle in m
  psi_rad          ->  Yaw angle of the vehicle in rad
  phi_rad          ->  Roll angle of the vehicle body in rad
  theta_rad        ->  Pitch angle of the vehicle in rad
  v_x_mps          ->  Velocity of the vehicles COG in the longitudinal direction of the vehicle
  v_y_mps          ->  Velocity of the vehicles COG in the lateral direction of the vehicle
  v_z_mps          ->  Velocity of the vehicles COG in the vertical direction of the vehicle
  psi_dot_radps    ->  Yaw rate of the vehicle in radps
  phi_dot_radps    ->  Roll rate of the vehicle in radps
  theta_dot_radps  ->  Pitch rate of the vehicle in radps
  z_w_FL_m         ->  Vertical position of the front left tire
  z_w_FR_m         ->  Vertical position of the front right tire
  z_w_RL_m         ->  Vertical position of the rear left tire
  z_w_RR_m         ->  Vertical position of the rear right tire
  v_z_w_FL_mps     ->  Vertical velocity of the front left tire
  v_z_w_FR_mps     ->  Vertical velocity of the front right tire
  v_z_w_RL_mps     ->  Vertical velocity of the rear left tire
  v_z_w_RR_mps     ->  Vertical velocity of the rear right tire
  kappa_tire_FL    ->  Tire forces
  kappa_tire_FR    ->  Tire forces
  kappa_tire_RL    ->  Tire forces
  kappa_tire_RR    ->  Tire forces
  alpha_tire_FL    ->  Tire forces
  alpha_tire_FR    ->  Tire forces
  alpha_tire_RL    ->  Tire forces
  alpha_tire_RR    ->  Tire forces 
*/


// Generate enum and names array
enum StateEnum {
#define X(name) name,
  STATE_LIST(X)
#undef X
    CNT_LENGTH_STATE_VECTOR
};
struct StateNames
{
  static constexpr std::array<
    std::string_view, static_cast<std::size_t>(StateEnum::CNT_LENGTH_STATE_VECTOR)>
    value = {
#define X(name) #name,
      STATE_LIST(X)
#undef X
  };
};
#undef STATE_LIST
}  // namespace tam::ocd::vehicle_dynamics::double_track::States
