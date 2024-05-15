//  Copyright 2023 Simon Sagmeister
#pragma once
#include <cmath>

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "tum_types_cpp/data_per_wheel.hpp"
namespace tam::types
{
enum class ErrorLvl { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };
enum class EmergencyLevel {
  NO_EMERGENCY = 0,
  EMERGENCY_STOP = 1,
  SOFT_EMERGENCY = 2,
  HARD_EMERGENCY = 3,
};
}  // namespace tam::types
namespace tam::types::common
{
struct Header
{
  uint32_t seq;  // Sequence counter
  uint64_t time_stamp_ns;
  std::string frame_id;
};
template <typename T>
struct Vector3D
{
  Vector3D() = default;
  Vector3D(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
  explicit Vector3D(std::vector<T> vec_in) : x(vec_in.at(0)), y(vec_in.at(1)), z(vec_in.at(2)) {}
  T x, y, z;
  Vector3D operator+(const Vector3D & other) const
  {
    return Vector3D(x + other.x, y + other.y, z + other.z);
  }
  Vector3D operator-(const Vector3D & other) const
  {
    return Vector3D(x - other.x, y - other.y, z - other.z);
  }
  // left multiplication with a factor
  friend Vector3D operator*(const double & factor, const Vector3D & obj)
  {
    return Vector3D(factor * obj.x, factor * obj.y, factor * obj.z);
  }
  // right multiplication with a factor = left multiplication
  Vector3D operator*(const double & factor) const { return factor * (*this); }
};
template <typename T>
struct Vector2D
{
  Vector2D() = default;
  Vector2D(T x_, T y_) : x(x_), y(y_) {}
  explicit Vector2D(std::vector<T> vec_in) : x(vec_in.at(0)), y(vec_in.at(1)) {}
  T x, y;
  Vector2D operator+(const Vector2D & other) const { return Vector2D(x + other.x, y + other.y); }
  Vector2D operator-(const Vector2D & other) const { return Vector2D(x - other.x, y - other.y); }
  // left multiplication with a factor
  friend Vector2D operator*(const double & factor, const Vector2D & obj)
  {
    return Vector2D(factor * obj.x, factor * obj.y);
  }
  friend Vector2D<T> operator/(const Vector2D<T> & obj, const double factor)
  {
    return Vector2D(obj.x / factor, obj.y / factor);
  }
  // right multiplication with a factor = left multiplication
  Vector2D operator*(const double & factor) const { return factor * (*this); }
};
class TUMDebugContainer
{
private:
  std::vector<std::string> _signal_names;
  std::vector<float> _values;
  bool _locked;
  std::size_t _debug_signal_config_hash;
  std::map<std::string, std::size_t> _index_lookup_table;
  /// @brief Lock the container so that the debug signals are frozen. Also calculate a hash to
  /// identify the debug signal config.
  void lock_container()
  {
    if (!_locked) {
      _locked = true;
      std::string concat_signal_names = "";
      for (auto & element : _signal_names) {
        concat_signal_names += element;
      }
      _debug_signal_config_hash = std::hash<std::string>{}(concat_signal_names);
    }
  }
  /// @brief Register a new debug signal as long as the container isn't locked. If the container is
  /// locked print an error message.
  /// @param signal_name Name of the signal
  /// @param value Value of the signal
  void handle_unregistered_signal(const std::string & signal_name, const float & value)
  {
    if (_locked) {
      std::cerr << "ERROR: Attempted registering a debug signal after locking the container: "
                << signal_name << "\n";
      return;
    }
    std::size_t new_index = _signal_names.size();
    _index_lookup_table[signal_name] = new_index;
    // Validate the signal name
    _signal_names.push_back(validate_signal_name(signal_name));
    _values.push_back(value);
  }

public:
  typedef std::shared_ptr<TUMDebugContainer> SharedPtr;
  /// @brief Removes leading and trailing slashes as well as all whitespaces from a string
  /// @param signal_name
  /// @return The validated string.
  static std::string validate_signal_name(const std::string & signal_name)
  {
    std::string validated_name =
      std::regex_replace(signal_name, std::regex("\\s+"), "$1");  // Remove whitespaces
    validated_name.erase(
      0, std::min(
           validated_name.find_first_not_of("/"),
           validated_name.size() - 1));  // Remove leading slashes
    validated_name.erase(
      validated_name.find_last_not_of("/") + 1, std::string::npos);  // Remove trailing slash
    return validated_name;
  }
  TUMDebugContainer() : _locked(false) {}
  std::vector<std::string> get_signal_names()
  {
    lock_container();  // Lock the container once the first time values are getted (and published on
                       // the debug out)
    return _signal_names;
  };
  std::vector<float> get_values()
  {
    lock_container();  // Lock the container once the first time values are getted (and published on
                       // the debug out)
    return _values;
  }
  float get_signal_value(const std::string & signal_name)
  {
    return _values[_index_lookup_table[signal_name]];
  }
  std::size_t get_config_hash()
  {
    lock_container();  // Lock the container once the first time values are getted (and published on
                       // the debug out)
    return _debug_signal_config_hash;
  }
  /// @brief Copy all signal values from this container to another container
  /// @param target_container The target container
  /// @param signal_name_prefix A prefix added to all the signals of the container
  void transfer_to(SharedPtr target_container, std::string signal_name_prefix = "")
  {
    lock_container();
    std::size_t num_signals = _signal_names.size();
    for (std::size_t i = 0; i < num_signals; i++) {
      target_container->log(signal_name_prefix + _signal_names.at(i), _values.at(i));
    }
  }
  /// @brief Log a debugging signal
  /// @param signal_name: Signal name
  /// @param value: Float value
  void log(const std::string & signal_name, const float & value)
  {
    try {
      // Set an already registered value
      std::size_t index =
        _index_lookup_table.at(signal_name);  // throws if the signal was not registered
      _values[index] = value;
    } catch (const std::out_of_range & e) {
      handle_unregistered_signal(signal_name, value);
    }
  }
  /// @brief Log a debug signal per wheel
  /// @param signal_name
  /// @param value
  void log(const std::string & signal_name, const DataPerWheel<double> & value)
  {
    log(signal_name + "/front_left", value.front_left);
    log(signal_name + "/front_right", value.front_right);
    log(signal_name + "/rear_left", value.rear_left);
    log(signal_name + "/rear_right", value.rear_right);
  }
  /// @brief Log a debug signal of Vector3D type
  /// @param signal_name
  /// @param value
  void log(const std::string & signal_name, const Vector3D<double> & value)
  {
    log(signal_name + "/x", value.x);
    log(signal_name + "/y", value.y);
    log(signal_name + "/z", value.z);
  }
};
/* Euler Angles in rad. Axes of rotation:

    1. Yaw (around Z)
    2. Pitch (around the new Y axis)
    3. Roll (around the new new X axis)
*/
struct EulerYPR
{
  explicit EulerYPR(const double yaw_, const double pitch_, const double roll_)
  : yaw(yaw_), pitch(pitch_), roll(roll_)
  {
  }
  double yaw, pitch, roll;
};
}  // namespace tam::types::common
