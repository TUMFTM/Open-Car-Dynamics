// Copyright 2026 Simon Sagmeister
#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
namespace tam::ocd::vehicle_dynamics::single_track::States
{
// clang-format off
#define STATE_LIST(X) \
  X(x_m) \
  X(y_m) \
  X(psi_rad) \
  X(v_x_mps) \
  X(v_y_mps) \
  X(psi_dot_radps) \
// clang-format on

/*
  x_m              ->  x position of the vehicle in m
  y_m              ->  y position of the vehicle in m
  psi_rad          ->  Yaw angle of the vehicle in rad
  v_x_mps          ->  Velocity of the vehicles COG in the longitudinal direction of the vehicle
  v_y_mps          ->  Velocity of the vehicles COG in the lateral direction of the vehicle
  psi_dot_radps    ->  Yaw rate of the vehicle in radps
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
}  // namespace tam::ocd::vehicle_dynamics::single_track::States
