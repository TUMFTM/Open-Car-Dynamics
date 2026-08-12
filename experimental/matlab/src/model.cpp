// Copyright 2026 Simon Sagmeister
#include "ocd_matlab/model.hpp"

#include <cmath>

#include <algorithm>
#include <concepts>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "ocd_aerodynamics_models_cpp/default.hpp"
#include "ocd_aerodynamics_models_cpp/ride_height.hpp"
#include "ocd_drivetrain_fx_cpp/drivetrain_fx_model.hpp"
#include "ocd_drivetrain_rwd_lsd_cpp/drivetrain_rwd_lsd_model.hpp"
#include "ocd_drivetrain_wheel_torque_cpp/drivetrain_direct_torque_model.hpp"
#include "ocd_steering_actuator_pt1_cpp/steering_actuator_pt1_model.hpp"
#include "ocd_tire_models_cpp/linear.hpp"
#include "ocd_tire_models_cpp/mf_52.hpp"
#include "ocd_tire_models_cpp/mf_simple.hpp"
#include "ocd_tire_models_cpp/mf_simple_extended.hpp"
#include "ocd_types_cpp/types.hpp"
#include "ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_model.hpp"
#include "ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_model.hpp"
#include "ocd_vehicle_model_cpp/vehicle_model.hpp"
#include "param_management_cpp/types.hpp"
namespace tam::ocd::matlab
{
namespace
{
using FxDrivetrain = tam::ocd::drivetrain::DrivetrainFxModel;
using WheelTorqueDrivetrain = tam::ocd::drivetrain::DrivetrainWheelTorqueModel;
using RwdLsdDrivetrain = tam::ocd::drivetrain::DrivetrainModel_RWD_LSD;
using SteeringActuator = tam::ocd::steering_actuator::PT1SteeringActuatorModel;
using PerWheel = tam::types::common::DataPerWheel<double>;

template <typename>
inline constexpr bool always_false = false;
void require_finite(const double value, const std::string_view name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}
template <std::size_t N>
void require_finite(const std::array<double, N> & values, const std::string_view name)
{
  for (const double value : values) require_finite(value, name);
}
void require_finite(const std::vector<double> & values, const std::string_view name)
{
  for (const double value : values) require_finite(value, name);
}
PerWheel to_per_wheel(const std::array<double, 4> & values)
{
  return {values[0], values[1], values[2], values[3]};
}
tam::types::common::Vector3D<double> to_vector3(const std::array<double, 3> & values)
{
  return {values[0], values[1], values[2]};
}
template <typename T>
std::array<double, 3> vector3_to_array(const T & value)
{
  return {value.x, value.y, value.z};
}
template <typename T>
std::array<double, 4> per_wheel_to_array(const T & value)
{
  return {value.front_left, value.front_right, value.rear_left, value.rear_right};
}
template <typename Drivetrain, typename VehicleDynamics>
class TypedModel final : public Model
{
public:
  using CoreModel = tam::ocd::VehicleModel<Drivetrain, SteeringActuator, VehicleDynamics>;

  static_assert(std::same_as<
                typename CoreModel::SteeringActuatorDriverInputType,
                tam::ocd::steering_actuator::PT1::DriverInput>);
  static_assert(SteeringActuator::k_state_vector_length == 1);
  static_assert(
    std::same_as<Drivetrain, FxDrivetrain> || std::same_as<Drivetrain, WheelTorqueDrivetrain> ||
    std::same_as<Drivetrain, RwdLsdDrivetrain>);
  TypedModel(
    const ModelSelection selection, const double time_step_s,
    const double initial_longitudinal_velocity_mps)
  : selection_(selection)
  {
    set_time_step(time_step_s);
    reset(initial_longitudinal_velocity_mps);
  }
  [[nodiscard]] ModelSelection selection() const noexcept override { return selection_; }
  [[nodiscard]] double time_s() const noexcept override { return time_s_; }
  [[nodiscard]] double time_step_s() const noexcept override { return time_step_s_; }
  void reset(const double initial_longitudinal_velocity_mps) override
  {
    require_finite(initial_longitudinal_velocity_mps, "initial longitudinal velocity");

    const auto parameters = model_.get_param_manager();
    const std::string dynamics_prefix =
      selection_.vehicle_dynamics == VehicleDynamicsType::SingleTrack
        ? "vehicle_dynamics_single_track."
        : "vehicle_dynamics_double_track.";
    const double front_radius_m =
      parameters->get_value(dynamics_prefix + "tire.rolling_radius_m.front").as_double();
    const double rear_radius_m =
      parameters->get_value(dynamics_prefix + "tire.rolling_radius_m.rear").as_double();
    if (!(front_radius_m > 0.0) || !(rear_radius_m > 0.0)) {
      throw std::runtime_error("The configured tire rolling radii must be positive");
    }

    parameters->set_value(
      "initial_state.vehicle_dynamics.v_x_mps", initial_longitudinal_velocity_mps);
    set_initial_wheel_speeds(initial_longitudinal_velocity_mps, front_radius_m, rear_radius_m);
    model_.reset();

    // VehicleModel::reset restores drivetrain states but clears its cached
    // wheel-speed signal. A zero-duration evaluation refreshes all cached
    // signals without changing the state or MATLAB-visible elapsed time.
    parameters->set_value("integration_step_size_s", 0.0);
    try {
      model_.step();
    } catch (...) {
      parameters->set_value("integration_step_size_s", time_step_s_);
      throw;
    }
    parameters->set_value("integration_step_size_s", time_step_s_);
    time_s_ = 0.0;
  }
  void set_time_step(const double time_step_s) override
  {
    require_finite(time_step_s, "time step");
    if (!(time_step_s > 0.0)) {
      throw std::invalid_argument("time step must be greater than zero");
    }
    model_.get_param_manager()->set_value("integration_step_size_s", time_step_s);
    time_step_s_ = time_step_s;
  }
  void set_external_influences(const ExternalInfluences & input) override
  {
    require_finite(input.external_force_N, "external force");
    require_finite(input.external_torque_Nm, "external torque");
    require_finite(input.wind_mps, "wind velocity");
    require_finite(input.z_height_road_m, "road height");
    require_finite(input.lambda_mue, "friction scale");

    tam::ocd::types::ExternalInfluences core_input;
    core_input.external_force_N = to_vector3(input.external_force_N);
    core_input.external_torque_Nm = to_vector3(input.external_torque_Nm);
    core_input.wind_mps = to_vector3(input.wind_mps);
    core_input.z_height_road_m = to_per_wheel(input.z_height_road_m);
    core_input.lambda_mue = to_per_wheel(input.lambda_mue);
    model_.set_external_influences(core_input);
  }
  ModelOutput step(
    const DrivetrainInput & drivetrain_input, const double steering_angle_rad) override
  {
    require_finite(steering_angle_rad, "steering angle");
    set_drivetrain_input(drivetrain_input);
    model_.set_steering_input({steering_angle_rad});
    model_.step();
    time_s_ += time_step_s_;
    return convert_output(model_.get_vehicle_model_output());
  }
  [[nodiscard]] std::vector<std::string> list_parameters() override
  {
    const auto parameter_names = model_.get_param_manager()->list_parameters();
    std::vector<std::string> result(parameter_names.begin(), parameter_names.end());
    std::ranges::sort(result);
    return result;
  }
  [[nodiscard]] ParameterValue get_parameter(const std::string & name) override
  {
    const auto value = model_.get_param_manager()->get_value(name);
    switch (value.get_type()) {
      case tam::pmg::ParameterType::DOUBLE:
        return value.as_double();
      case tam::pmg::ParameterType::DOUBLE_ARRAY:
        return value.as_double_array();
      default:
        throw std::runtime_error(
          "The MATLAB binding only supports double and double-array parameters");
    }
  }
  void set_parameter(const std::string & name, const ParameterValue & value) override
  {
    if (name == "integration_step_size_s") {
      if (!std::holds_alternative<double>(value)) {
        throw std::invalid_argument("integration_step_size_s must be a scalar double");
      }
      set_time_step(std::get<double>(value));
      return;
    }

    const auto parameters = model_.get_param_manager();
    switch (parameters->get_type(name)) {
      case tam::pmg::ParameterType::DOUBLE: {
        if (!std::holds_alternative<double>(value)) {
          throw std::invalid_argument("Parameter '" + name + "' requires a scalar double");
        }
        const double scalar = std::get<double>(value);
        require_finite(scalar, name);
        parameters->set_value(name, scalar);
        return;
      }
      case tam::pmg::ParameterType::DOUBLE_ARRAY: {
        if (!std::holds_alternative<std::vector<double>>(value)) {
          throw std::invalid_argument("Parameter '" + name + "' requires a double vector");
        }
        const auto & vector = std::get<std::vector<double>>(value);
        require_finite(vector, name);
        parameters->set_value(name, vector);
        return;
      }
      default:
        throw std::runtime_error(
          "The MATLAB binding only supports double and double-array parameters");
    }
  }

private:
  void set_initial_wheel_speeds(
    const double velocity_mps, const double front_radius_m, const double rear_radius_m)
  {
    const auto parameters = model_.get_param_manager();
    parameters->set_value("initial_state.drivetrain.omega_FL_radps", velocity_mps / front_radius_m);
    parameters->set_value("initial_state.drivetrain.omega_FR_radps", velocity_mps / front_radius_m);
    if constexpr (std::same_as<Drivetrain, WheelTorqueDrivetrain>) {
      parameters->set_value(
        "initial_state.drivetrain.omega_RL_radps", velocity_mps / rear_radius_m);
      parameters->set_value(
        "initial_state.drivetrain.omega_RR_radps", velocity_mps / rear_radius_m);
    } else {
      parameters->set_value(
        "initial_state.drivetrain.omega_rear_axle_radps", velocity_mps / rear_radius_m);
      parameters->set_value("initial_state.drivetrain.omega_diff_rear_radps", 0.0);
    }
  }
  void set_drivetrain_input(const DrivetrainInput & input)
  {
    if constexpr (std::same_as<Drivetrain, FxDrivetrain>) {
      require_finite(input.longitudinal_force_N, "longitudinal force");
      model_.set_drivetrain_input(input.longitudinal_force_N);
    } else if constexpr (std::same_as<Drivetrain, WheelTorqueDrivetrain>) {
      require_finite(input.drivetrain_input_torque_per_wheel_Nm, "wheel torque");
      typename Drivetrain::DriverInputType core_input;
      core_input.drivetrain_input_torque_per_wheel_Nm =
        to_per_wheel(input.drivetrain_input_torque_per_wheel_Nm);
      model_.set_drivetrain_input(core_input);
    } else if constexpr (std::same_as<Drivetrain, RwdLsdDrivetrain>) {
      require_finite(input.transmission_output_torque_Nm, "transmission output torque");
      require_finite(input.current_engine_inertia_at_wheels_kgm2, "engine inertia");
      require_finite(input.brake_torque_per_wheel_Nm, "brake torque");
      typename Drivetrain::DriverInputType core_input;
      core_input.transmission_output_torque_Nm = input.transmission_output_torque_Nm;
      core_input.current_engine_inertia_at_wheels_kgm2 =
        input.current_engine_inertia_at_wheels_kgm2;
      core_input.brake_torque_per_wheel_Nm = to_per_wheel(input.brake_torque_per_wheel_Nm);
      model_.set_drivetrain_input(core_input);
    } else {
      static_assert(always_false<Drivetrain>, "Unsupported drivetrain type");
    }
  }
  ModelOutput convert_output(const tam::ocd::types::VehicleModelOutput & source) const
  {
    const auto & dynamics = source.vehicle_dynamics_output;
    return {
      .time_s = time_s_,
      .position_m = vector3_to_array(dynamics.position_m),
      .velocity_mps = vector3_to_array(dynamics.velocity_mps),
      .acceleration_mps2 = vector3_to_array(dynamics.acceleration_mps2),
      .orientation_rad = vector3_to_array(dynamics.orientation_rad),
      .angular_velocity_radps = vector3_to_array(dynamics.angular_velocity_radps),
      .angular_acceleration_radps2 = vector3_to_array(dynamics.angular_acceleration_radps2),
      .wheel_speeds_radps = per_wheel_to_array(source.wheel_speeds_radps),
      .steering_angle_per_wheel_rad = per_wheel_to_array(source.steering_angle_per_wheel_rad),
      .tire_longitudinal_slip_per_wheel =
        per_wheel_to_array(dynamics.tire_longitudinal_slip_per_wheel),
      .tire_slip_angle_per_wheel_rad = per_wheel_to_array(dynamics.tire_slip_angle_per_wheel_rad),
      .longitudinal_tire_force_per_wheel_N =
        per_wheel_to_array(dynamics.longitudinal_tire_force_tire_frame_per_wheel_N),
      .lateral_tire_force_per_wheel_N =
        per_wheel_to_array(dynamics.lateral_tire_force_tire_frame_per_wheel_N),
      .vertical_tire_force_per_wheel_N =
        per_wheel_to_array(dynamics.vertical_tire_force_per_wheel_N),
    };
  }
  ModelSelection selection_;
  CoreModel model_{};
  double time_step_s_{0.0008};
  double time_s_{};
};
template <
  typename Drivetrain, template <typename, typename> class Dynamics, typename Tire,
  typename Aerodynamics>
std::unique_ptr<Model> make_typed_model(
  const ModelSelection & selection, const double time_step_s,
  const double initial_longitudinal_velocity_mps)
{
  using ConcreteDynamics = Dynamics<Tire, Aerodynamics>;
  return std::make_unique<TypedModel<Drivetrain, ConcreteDynamics>>(
    selection, time_step_s, initial_longitudinal_velocity_mps);
}
template <typename Drivetrain, template <typename, typename> class Dynamics, typename Tire>
std::unique_ptr<Model> select_aerodynamics(
  const ModelSelection & selection, const double time_step_s,
  const double initial_longitudinal_velocity_mps)
{
  switch (selection.aerodynamics) {
    case AerodynamicsType::Default:
      return make_typed_model<
        Drivetrain, Dynamics, Tire, tam::ocd::aerodynamics::DefaultAerodynamicsModel>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
    case AerodynamicsType::RideHeight:
      return make_typed_model<
        Drivetrain, Dynamics, Tire, tam::ocd::aerodynamics::RideHeightAerodynamicsModel>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
  }
  throw std::invalid_argument("Unsupported aerodynamics selection");
}
template <typename Drivetrain, template <typename, typename> class Dynamics>
std::unique_ptr<Model> select_tire(
  const ModelSelection & selection, const double time_step_s,
  const double initial_longitudinal_velocity_mps)
{
  switch (selection.tire) {
    case TireType::Mf52:
      return select_aerodynamics<Drivetrain, Dynamics, tam::ocd::tire_models::MF52>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
    case TireType::Linear:
      return select_aerodynamics<Drivetrain, Dynamics, tam::ocd::tire_models::Linear>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
    case TireType::MfSimple:
      return select_aerodynamics<Drivetrain, Dynamics, tam::ocd::tire_models::MF_Simple>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
    case TireType::MfSimpleExtended:
      return select_aerodynamics<Drivetrain, Dynamics, tam::ocd::tire_models::MF_Simple_Extended>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
  }
  throw std::invalid_argument("Unsupported tire selection");
}
template <typename Drivetrain>
std::unique_ptr<Model> select_vehicle_dynamics(
  const ModelSelection & selection, const double time_step_s,
  const double initial_longitudinal_velocity_mps)
{
  switch (selection.vehicle_dynamics) {
    case VehicleDynamicsType::SingleTrack:
      return select_tire<Drivetrain, tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
    case VehicleDynamicsType::DoubleTrack:
      return select_tire<Drivetrain, tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
  }
  throw std::invalid_argument("Unsupported vehicle-dynamics selection");
}
}  // namespace
std::string_view to_string(const DrivetrainType value)
{
  switch (value) {
    case DrivetrainType::Fx:
      return "fx";
    case DrivetrainType::WheelTorque:
      return "wheel_torque";
    case DrivetrainType::RwdLsd:
      return "rwd_lsd";
  }
  throw std::invalid_argument("Unsupported drivetrain selection");
}
std::string_view to_string(const SteeringType value)
{
  if (value == SteeringType::Pt1) return "pt1";
  throw std::invalid_argument("Unsupported steering selection");
}
std::string_view to_string(const VehicleDynamicsType value)
{
  switch (value) {
    case VehicleDynamicsType::SingleTrack:
      return "single_track";
    case VehicleDynamicsType::DoubleTrack:
      return "double_track";
  }
  throw std::invalid_argument("Unsupported vehicle-dynamics selection");
}
std::string_view to_string(const TireType value)
{
  switch (value) {
    case TireType::Mf52:
      return "mf52";
    case TireType::Linear:
      return "linear";
    case TireType::MfSimple:
      return "mf_simple";
    case TireType::MfSimpleExtended:
      return "mf_simple_extended";
  }
  throw std::invalid_argument("Unsupported tire selection");
}
std::string_view to_string(const AerodynamicsType value)
{
  switch (value) {
    case AerodynamicsType::Default:
      return "default";
    case AerodynamicsType::RideHeight:
      return "ride_height";
  }
  throw std::invalid_argument("Unsupported aerodynamics selection");
}
std::size_t supported_model_count() noexcept
{
  return k_drivetrain_types.size() * k_steering_types.size() * k_vehicle_dynamics_types.size() *
         k_tire_types.size() * k_aerodynamics_types.size();
}
std::unique_ptr<Model> create_model(
  const ModelSelection & selection, const double time_step_s,
  const double initial_longitudinal_velocity_mps)
{
  if (selection.steering != SteeringType::Pt1) {
    throw std::invalid_argument("Unsupported steering selection");
  }
  switch (selection.drivetrain) {
    case DrivetrainType::Fx:
      return select_vehicle_dynamics<FxDrivetrain>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
    case DrivetrainType::WheelTorque:
      return select_vehicle_dynamics<WheelTorqueDrivetrain>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
    case DrivetrainType::RwdLsd:
      return select_vehicle_dynamics<RwdLsdDrivetrain>(
        selection, time_step_s, initial_longitudinal_velocity_mps);
  }
  throw std::invalid_argument("Unsupported drivetrain selection");
}
}  // namespace tam::ocd::matlab
