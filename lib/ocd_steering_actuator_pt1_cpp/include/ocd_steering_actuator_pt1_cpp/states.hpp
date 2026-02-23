// Copyright 2026 Simon Sagmeister
#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
namespace tam::ocd::steering_actuator::PT1
{
// Input to the steering actuator model
struct DriverInput
{
  double steering_angle_rad{0.0};
};
// Output of the steering actuator model
struct Feedback
{
  double steering_angle_rad{0.0};
};
namespace States
{
#define STATE_LIST(X) X(position_rad)
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
}  // namespace States
}  // namespace tam::ocd::steering_actuator::PT1