
// Copyright 2026 Simon Sagmeister
#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "tum_types_cpp/common.hpp"
namespace tam::ocd::drivetrain::wheel_torque::States
{
// Input to the steering actuator model
#define STATE_LIST(X) \
  X(omega_FL_radps)   \
  X(omega_FR_radps)   \
  X(omega_RL_radps)   \
  X(omega_RR_radps)
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
}  // namespace tam::ocd::drivetrain::wheel_torque::States
