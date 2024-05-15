// Copyright 2024 Simon Sagmeister
#pragma once
#include <cmath>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
namespace tam::types::common
{
template <typename T>
struct DataPerWheel
{
  T front_left, front_right, rear_left, rear_right;
  enum class SignalOrder { front_left, front_right, rear_left, rear_right };
  DataPerWheel() = default;
  DataPerWheel(T FL, T FR, T RL, T RR)
  : front_left(FL), front_right(FR), rear_left(RL), rear_right(RR)
  {
  }
  explicit DataPerWheel(std::array<T, 4> const & array)
  : front_left(array[SignalOrder::front_left]),
    front_right(array[SignalOrder::front_right]),
    rear_left(array[SignalOrder::rear_left]),
    rear_right(array[SignalOrder::rear_right])
  {
  }
  // Use the same value for all 4 elements
  explicit DataPerWheel(T value)
  : front_left(value), front_right(value), rear_left(value), rear_right(value)
  {
  }
  static DataPerWheel<T> from_front_and_rear(T front, T rear) { return {front, front, rear, rear}; }
  static DataPerWheel<T> from_left_and_right(T left, T right) { return {left, right, left, right}; }
  std::array<T, 4> to_array() const { return {front_left, front_right, rear_left, rear_right}; }
// Component wise math operations
// If the type with is calculated with is a per wheel type
// -> Use component wise math
// =========================================================================
#define OVERLOAD_OPERATOR_COMPONENT_WISE_MATH(operator_name, _operator_)                     \
  template <typename T_math>                                                                 \
  DataPerWheel<T> operator_name(DataPerWheel<T_math> const & value) const                    \
  {                                                                                          \
    return {                                                                                 \
      front_left _operator_ value.front_left, front_right _operator_ value.front_right,      \
      rear_left _operator_ value.rear_left, rear_right _operator_ value.rear_right};         \
  }                                                                                          \
  template <typename T_math>                                                                 \
  DataPerWheel<T> operator_name(T_math const & value) const                                  \
  {                                                                                          \
    return {                                                                                 \
      front_left _operator_ value, front_right _operator_ value, rear_left _operator_ value, \
      rear_right _operator_ value};                                                          \
  }
  // Instanciate operators
  OVERLOAD_OPERATOR_COMPONENT_WISE_MATH(operator*, *)
  OVERLOAD_OPERATOR_COMPONENT_WISE_MATH(operator+, +)
  OVERLOAD_OPERATOR_COMPONENT_WISE_MATH(operator-, -)
  OVERLOAD_OPERATOR_COMPONENT_WISE_MATH(operator/, /)

#undef OVERLOAD_OPERATOR_COMPONENT_WISE_MATH

#define OVERLOAD_OPERATOR_COMPONENT_WISE_MATH_ASSIGNMENT(operator_name, _operator_) \
  template <typename T_math>                                                        \
  DataPerWheel<T> & operator_name(DataPerWheel<T_math> const & value)               \
  {                                                                                 \
    front_left _operator_ value.front_left;                                         \
    front_right _operator_ value.front_right;                                       \
    rear_left _operator_ value.rear_left;                                           \
    rear_right _operator_ value.rear_right;                                         \
    return *(this);                                                                 \
  }                                                                                 \
  template <typename T_math>                                                        \
  DataPerWheel<T> & operator_name(T_math const & value)                             \
  {                                                                                 \
    front_left _operator_ value;                                                    \
    front_right _operator_ value;                                                   \
    rear_left _operator_ value;                                                     \
    rear_right _operator_ value;                                                    \
    return *(this);                                                                 \
  }
  // Instanciate operators
  OVERLOAD_OPERATOR_COMPONENT_WISE_MATH_ASSIGNMENT(operator*=, *=)
  OVERLOAD_OPERATOR_COMPONENT_WISE_MATH_ASSIGNMENT(operator+=, +=)
  OVERLOAD_OPERATOR_COMPONENT_WISE_MATH_ASSIGNMENT(operator-=, -=)
  OVERLOAD_OPERATOR_COMPONENT_WISE_MATH_ASSIGNMENT(operator/=, /=)

#undef OVERLOAD_OPERATOR_COMPONENT_WISE_MATH_ASSIGNMENT
  DataPerWheel<T> operator-() const { return {-front_left, -front_right, -rear_left, -rear_right}; }
#define OVERLOAD_OPERATOR_COMPONENT_WISE_COMPARISON(operator_name, _operator_)               \
  DataPerWheel<bool> operator_name(const DataPerWheel<T> & other) const                      \
  {                                                                                          \
    return {                                                                                 \
      front_left _operator_ other.front_left, front_right _operator_ other.front_right,      \
      rear_left _operator_ other.rear_left, rear_right _operator_ other.rear_right};         \
  }                                                                                          \
  DataPerWheel<bool> operator_name(const T & other) const                                    \
  {                                                                                          \
    return {                                                                                 \
      front_left _operator_ other, front_right _operator_ other, rear_left _operator_ other, \
      rear_right _operator_ other};                                                          \
  }

  OVERLOAD_OPERATOR_COMPONENT_WISE_COMPARISON(operator==, ==)
  OVERLOAD_OPERATOR_COMPONENT_WISE_COMPARISON(operator!=, !=)
  OVERLOAD_OPERATOR_COMPONENT_WISE_COMPARISON(operator<, <)
  OVERLOAD_OPERATOR_COMPONENT_WISE_COMPARISON(operator>, >)
  OVERLOAD_OPERATOR_COMPONENT_WISE_COMPARISON(operator<=, <=)
  OVERLOAD_OPERATOR_COMPONENT_WISE_COMPARISON(operator>=, >=)
  OVERLOAD_OPERATOR_COMPONENT_WISE_COMPARISON(operator<=>, <=>)
#undef OVERLOAD_OPERATOR_COMPONENT_WISE_COMPARISON
  // Make type iterable
  // ================================================================================
  constexpr std::size_t size() const { return 4; }
  T & operator[](std::size_t i)
  {
    switch (i) {
      // clang-format off
      case static_cast<std::size_t>(SignalOrder::front_left): return front_left;
      case static_cast<std::size_t>(SignalOrder::front_right): return front_right;
      case static_cast<std::size_t>(SignalOrder::rear_left): return rear_left;
      case static_cast<std::size_t>(SignalOrder::rear_right): return rear_right;
      default: throw std::out_of_range(std::string("Range Error: Maximum index is 3. Tried accessing element number: ") + std::to_string(i)); // NOLINT
    }
    // clang-format on
  }
  constexpr std::string name_at(std::size_t i) const
  {
    switch (i) {
      // clang-format off
      case static_cast<std::size_t>(SignalOrder::front_left): return "front_left";
      case static_cast<std::size_t>(SignalOrder::front_right): return "front_right";
      case static_cast<std::size_t>(SignalOrder::rear_left): return "rear_left";
      case static_cast<std::size_t>(SignalOrder::rear_right): return "rear_right";
      default: throw std::out_of_range(std::string("Range Error: Maximum index is 3. Tried accessing element number: ") + std::to_string(i)); // NOLINT
        // clang-format on
    }
  }
  // Addition function for bool overload
  // ======================================================
  DataPerWheel<bool> operator!() const requires(std::is_same_v<T, bool>)
  {
    return {!front_left, !front_right, !rear_left, !rear_right};
  }
  DataPerWheel<bool> operator&&(const DataPerWheel<bool> & other) const
    requires(std::is_same_v<T, bool>)
  {
    return {
      front_left && other.front_left, front_right && other.front_right,
      rear_left && other.rear_left, rear_right && other.rear_right};
  }
  DataPerWheel<bool> operator||(const DataPerWheel<bool> & other) const
    requires(std::is_same_v<T, bool>)
  {
    return {
      front_left || other.front_left, front_right || other.front_right,
      rear_left || other.rear_left, rear_right || other.rear_right};
  }
  bool all() const requires(std::is_same_v<T, bool>)
  {
    return (front_left && front_right && rear_left && rear_right);
  }
  bool any() const requires(std::is_same_v<T, bool>)
  {
    return (front_left || front_right || rear_left || rear_right);
  }
};
}  // namespace tam::types::common
namespace std
{
#define TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(function)                                     \
  template <typename T>                                                                   \
  inline tam::types::common::DataPerWheel<T> function(                                    \
    tam::types::common::DataPerWheel<T> const & input)                                    \
  {                                                                                       \
    return {                                                                              \
      function(input.front_left), function(input.front_right), function(input.rear_left), \
      function(input.rear_right)};                                                        \
  }

TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(abs)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(exp)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(sqrt)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(log)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(log10)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(sin)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(cos)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(tan)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(asin)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(acos)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(atan)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(sinh)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(cosh)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(tanh)

#undef TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL
// Special functions

#define TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(function)                                         \
  template <typename T>                                                                       \
  inline tam::types::common::DataPerWheel<T> function(                                        \
    const tam::types::common::DataPerWheel<T> & base,                                         \
    const tam::types::common::DataPerWheel<T> & exp)                                          \
  {                                                                                           \
    return {                                                                                  \
      function(base.front_left, exp.front_left), function(base.front_right, exp.front_right), \
      function(base.rear_left, exp.rear_left), function(base.rear_right, exp.rear_right)};    \
  }                                                                                           \
  template <typename T>                                                                       \
  inline tam::types::common::DataPerWheel<T> function(                                        \
    const tam::types::common::DataPerWheel<T> & base, const T & exp_)                         \
  {                                                                                           \
    return {                                                                                  \
      function(base.front_left, exp_), function(base.front_right, exp_),                      \
      function(base.rear_left, exp_), function(base.rear_right, exp_)};                       \
  }

TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(pow)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(atan2)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(min)
TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL(max)
#undef TUM_STD_TYPE_SUPPORT_DATA_PER_WHEEL
template <typename T>
inline tam::types::common::DataPerWheel<T> clamp(
  const tam::types::common::DataPerWheel<T> & v, const tam::types::common::DataPerWheel<T> & lo,
  const tam::types::common::DataPerWheel<T> & hi)
{
  return tam::types::common::DataPerWheel<T>(
    clamp(v.front_left, lo.front_left, hi.front_left),
    clamp(v.front_right, lo.front_right, hi.front_right),
    clamp(v.rear_left, lo.rear_left, hi.rear_left),
    clamp(v.rear_right, lo.rear_right, hi.rear_right));
}
}  // namespace std
