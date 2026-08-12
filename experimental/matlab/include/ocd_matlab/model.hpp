// Copyright 2026 Simon Sagmeister
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <variant>
#include <vector>
namespace tam::ocd::matlab
{
enum class DrivetrainType { Fx, WheelTorque, RwdLsd };
enum class SteeringType { Pt1 };
enum class VehicleDynamicsType { SingleTrack, DoubleTrack };
enum class TireType { Mf52, Linear, MfSimple, MfSimpleExtended };
enum class AerodynamicsType { Default, RideHeight };

inline constexpr std::array k_drivetrain_types{
  DrivetrainType::Fx, DrivetrainType::WheelTorque, DrivetrainType::RwdLsd};
inline constexpr std::array k_steering_types{SteeringType::Pt1};
inline constexpr std::array k_vehicle_dynamics_types{
  VehicleDynamicsType::SingleTrack, VehicleDynamicsType::DoubleTrack};
inline constexpr std::array k_tire_types{
  TireType::Mf52, TireType::Linear, TireType::MfSimple, TireType::MfSimpleExtended};
inline constexpr std::array k_aerodynamics_types{
  AerodynamicsType::Default, AerodynamicsType::RideHeight};
struct ModelSelection
{
  DrivetrainType drivetrain{DrivetrainType::Fx};
  SteeringType steering{SteeringType::Pt1};
  VehicleDynamicsType vehicle_dynamics{VehicleDynamicsType::DoubleTrack};
  TireType tire{TireType::Mf52};
  AerodynamicsType aerodynamics{AerodynamicsType::Default};
};
struct DrivetrainInput
{
  double longitudinal_force_N{};
  std::array<double, 4> drivetrain_input_torque_per_wheel_Nm{};
  double transmission_output_torque_Nm{};
  double current_engine_inertia_at_wheels_kgm2{};
  std::array<double, 4> brake_torque_per_wheel_Nm{};
};
struct ExternalInfluences
{
  std::array<double, 3> external_force_N{};
  std::array<double, 3> external_torque_Nm{};
  std::array<double, 3> wind_mps{};
  std::array<double, 4> z_height_road_m{};
  std::array<double, 4> lambda_mue{1.0, 1.0, 1.0, 1.0};
};
struct ModelOutput
{
  double time_s{};
  std::array<double, 3> position_m{};
  std::array<double, 3> velocity_mps{};
  std::array<double, 3> acceleration_mps2{};
  std::array<double, 3> orientation_rad{};
  std::array<double, 3> angular_velocity_radps{};
  std::array<double, 3> angular_acceleration_radps2{};
  std::array<double, 4> wheel_speeds_radps{};
  std::array<double, 4> steering_angle_per_wheel_rad{};
  std::array<double, 4> tire_longitudinal_slip_per_wheel{};
  std::array<double, 4> tire_slip_angle_per_wheel_rad{};
  std::array<double, 4> longitudinal_tire_force_per_wheel_N{};
  std::array<double, 4> lateral_tire_force_per_wheel_N{};
  std::array<double, 4> vertical_tire_force_per_wheel_N{};
};
using ParameterValue = std::variant<double, std::vector<double>>;
class Model
{
public:
  virtual ~Model() = default;

  [[nodiscard]] virtual ModelSelection selection() const noexcept = 0;
  [[nodiscard]] virtual double time_s() const noexcept = 0;
  [[nodiscard]] virtual double time_step_s() const noexcept = 0;

  virtual void reset(double initial_longitudinal_velocity_mps = 0.0) = 0;
  virtual void set_time_step(double time_step_s) = 0;
  virtual void set_external_influences(const ExternalInfluences & input) = 0;
  virtual ModelOutput step(const DrivetrainInput & drivetrain_input, double steering_angle_rad) = 0;

  [[nodiscard]] virtual std::vector<std::string> list_parameters() = 0;
  [[nodiscard]] virtual ParameterValue get_parameter(const std::string & name) = 0;
  virtual void set_parameter(const std::string & name, const ParameterValue & value) = 0;
};
[[nodiscard]] std::string_view to_string(DrivetrainType value);
[[nodiscard]] std::string_view to_string(SteeringType value);
[[nodiscard]] std::string_view to_string(VehicleDynamicsType value);
[[nodiscard]] std::string_view to_string(TireType value);
[[nodiscard]] std::string_view to_string(AerodynamicsType value);
[[nodiscard]] std::size_t supported_model_count() noexcept;

[[nodiscard]] std::unique_ptr<Model> create_model(
  const ModelSelection & selection = {}, double time_step_s = 0.0008,
  double initial_longitudinal_velocity_mps = 0.0);
}  // namespace tam::ocd::matlab
