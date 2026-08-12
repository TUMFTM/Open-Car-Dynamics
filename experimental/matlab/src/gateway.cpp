// Copyright 2026 Simon Sagmeister
#include <cmath>

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

#include "mex.h"
#include "ocd_matlab/model.hpp"
namespace
{
namespace binding = tam::ocd::matlab;

std::unique_ptr<binding::Model> model;
bool cleanup_registered = false;
void cleanup() { model.reset(); }
void ensure_cleanup_registered()
{
  if (!cleanup_registered) {
    mexAtExit(cleanup);
    cleanup_registered = true;
  }
}
binding::Model & get_model()
{
  ensure_cleanup_registered();
  if (!model) model = binding::create_model();
  return *model;
}
void require_argument_count(
  const int actual, const int expected, const std::string_view command, const std::string_view kind)
{
  if (actual != expected) {
    throw std::invalid_argument(
      std::string(command) + " expects " + std::to_string(expected) + " " + std::string(kind) +
      ", but received " + std::to_string(actual));
  }
}
void require_struct_scalar(const mxArray * value, const std::string_view name)
{
  if (!mxIsStruct(value) || mxGetNumberOfElements(value) != 1) {
    throw std::invalid_argument(std::string(name) + " must be a scalar MATLAB struct");
  }
}
const mxArray * get_field(const mxArray * structure, const char * name, const bool required = true)
{
  const mxArray * value = mxGetField(structure, 0, name);
  if (required && value == nullptr) {
    throw std::invalid_argument(std::string("Missing required struct field '") + name + "'");
  }
  return value;
}
double scalar_double(const mxArray * value, const std::string_view name)
{
  if (!mxIsDouble(value) || mxIsComplex(value) || mxGetNumberOfElements(value) != 1) {
    throw std::invalid_argument(std::string(name) + " must be a real double scalar");
  }
  const double result = mxGetScalar(value);
  if (!std::isfinite(result)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
  return result;
}
double optional_scalar(
  const mxArray * structure, const char * field_name, const double default_value = 0.0)
{
  const mxArray * value = get_field(structure, field_name, false);
  return value == nullptr ? default_value : scalar_double(value, field_name);
}
std::vector<double> double_vector(const mxArray * value, const std::string_view name)
{
  if (!mxIsDouble(value) || mxIsComplex(value)) {
    throw std::invalid_argument(std::string(name) + " must be a real double vector");
  }
  const std::size_t count = mxGetNumberOfElements(value);
  std::vector<double> result(count);
  if (count > 0) std::copy_n(mxGetPr(value), count, result.begin());
  for (const double element : result) {
    if (!std::isfinite(element)) {
      throw std::invalid_argument(std::string(name) + " must contain only finite values");
    }
  }
  return result;
}
template <std::size_t N>
std::array<double, N> fixed_vector(const mxArray * value, const std::string_view name)
{
  const auto vector = double_vector(value, name);
  if (vector.size() != N) {
    throw std::invalid_argument(
      std::string(name) + " must contain exactly " + std::to_string(N) + " elements");
  }
  std::array<double, N> result{};
  std::copy(vector.begin(), vector.end(), result.begin());
  return result;
}
template <std::size_t N>
void read_optional_vector(
  const mxArray * structure, const char * field_name, std::array<double, N> & destination)
{
  if (const mxArray * value = get_field(structure, field_name, false); value != nullptr) {
    destination = fixed_vector<N>(value, field_name);
  }
}
std::string matlab_text(const mxArray * value, const std::string_view name)
{
  mxArray * converted = nullptr;
  const mxArray * character_value = value;
  if (mxIsClass(value, "string")) {
    if (mxGetNumberOfElements(value) != 1) {
      throw std::invalid_argument(std::string(name) + " must be a scalar string");
    }
    mxArray * arguments[]{const_cast<mxArray *>(value)};
    if (mexCallMATLAB(1, &converted, 1, arguments, "char") != 0 || converted == nullptr) {
      throw std::runtime_error(std::string("Could not convert ") + std::string(name) + " to text");
    }
    character_value = converted;
  }

  if (!mxIsChar(character_value)) {
    if (converted != nullptr) mxDestroyArray(converted);
    throw std::invalid_argument(std::string(name) + " must be a character vector or scalar string");
  }
  char * raw_text = mxArrayToString(character_value);
  if (raw_text == nullptr) {
    if (converted != nullptr) mxDestroyArray(converted);
    throw std::runtime_error(std::string("Could not convert ") + std::string(name) + " to text");
  }
  std::string result(raw_text);
  mxFree(raw_text);
  if (converted != nullptr) mxDestroyArray(converted);
  return result;
}
binding::DrivetrainType parse_drivetrain(const std::string & value)
{
  if (value == "fx") return binding::DrivetrainType::Fx;
  if (value == "wheel_torque") return binding::DrivetrainType::WheelTorque;
  if (value == "rwd_lsd") return binding::DrivetrainType::RwdLsd;
  throw std::invalid_argument(
    "Unknown drivetrain '" + value + "'; expected fx, wheel_torque, or rwd_lsd");
}
binding::SteeringType parse_steering(const std::string & value)
{
  if (value == "pt1") return binding::SteeringType::Pt1;
  throw std::invalid_argument("Unknown steering model '" + value + "'; expected pt1");
}
binding::VehicleDynamicsType parse_vehicle_dynamics(const std::string & value)
{
  if (value == "single_track") return binding::VehicleDynamicsType::SingleTrack;
  if (value == "double_track") return binding::VehicleDynamicsType::DoubleTrack;
  throw std::invalid_argument(
    "Unknown vehicle-dynamics model '" + value + "'; expected single_track or double_track");
}
binding::TireType parse_tire(const std::string & value)
{
  if (value == "mf52") return binding::TireType::Mf52;
  if (value == "linear") return binding::TireType::Linear;
  if (value == "mf_simple") return binding::TireType::MfSimple;
  if (value == "mf_simple_extended") return binding::TireType::MfSimpleExtended;
  throw std::invalid_argument(
    "Unknown tire model '" + value + "'; expected mf52, linear, mf_simple, or mf_simple_extended");
}
binding::AerodynamicsType parse_aerodynamics(const std::string & value)
{
  if (value == "default") return binding::AerodynamicsType::Default;
  if (value == "ride_height") return binding::AerodynamicsType::RideHeight;
  throw std::invalid_argument(
    "Unknown aerodynamics model '" + value + "'; expected default or ride_height");
}
binding::ModelSelection parse_selection(const mxArray * value)
{
  require_struct_scalar(value, "model configuration");
  return {
    .drivetrain = parse_drivetrain(matlab_text(get_field(value, "drivetrain"), "drivetrain")),
    .steering = parse_steering(matlab_text(get_field(value, "steering"), "steering")),
    .vehicle_dynamics =
      parse_vehicle_dynamics(matlab_text(get_field(value, "vehicle_dynamics"), "vehicle_dynamics")),
    .tire = parse_tire(matlab_text(get_field(value, "tire"), "tire")),
    .aerodynamics =
      parse_aerodynamics(matlab_text(get_field(value, "aerodynamics"), "aerodynamics")),
  };
}
binding::DrivetrainInput parse_drivetrain_input(
  const mxArray * value, const binding::DrivetrainType type)
{
  binding::DrivetrainInput result;
  switch (type) {
    case binding::DrivetrainType::Fx:
      result.longitudinal_force_N = scalar_double(value, "longitudinal force");
      break;
    case binding::DrivetrainType::WheelTorque:
      result.drivetrain_input_torque_per_wheel_Nm = fixed_vector<4>(value, "wheel torque");
      break;
    case binding::DrivetrainType::RwdLsd:
      require_struct_scalar(value, "RWD LSD drivetrain input");
      result.transmission_output_torque_Nm =
        optional_scalar(value, "transmission_output_torque_Nm");
      result.current_engine_inertia_at_wheels_kgm2 =
        optional_scalar(value, "current_engine_inertia_at_wheels_kgm2");
      read_optional_vector(value, "brake_torque_per_wheel_Nm", result.brake_torque_per_wheel_Nm);
      break;
  }
  return result;
}
binding::ExternalInfluences parse_external_influences(const mxArray * value)
{
  require_struct_scalar(value, "external influences");
  binding::ExternalInfluences result;
  read_optional_vector(value, "external_force_N", result.external_force_N);
  read_optional_vector(value, "external_torque_Nm", result.external_torque_Nm);
  read_optional_vector(value, "wind_mps", result.wind_mps);
  read_optional_vector(value, "z_height_road_m", result.z_height_road_m);
  read_optional_vector(value, "lambda_mue", result.lambda_mue);
  return result;
}
template <std::size_t N>
mxArray * make_row_vector(const std::array<double, N> & values)
{
  mxArray * result = mxCreateDoubleMatrix(1, static_cast<mwSize>(N), mxREAL);
  double * data = mxGetPr(result);
  std::copy(values.begin(), values.end(), data);
  return result;
}
mxArray * make_vector(const std::vector<double> & values)
{
  mxArray * result = mxCreateDoubleMatrix(1, static_cast<mwSize>(values.size()), mxREAL);
  if (!values.empty()) std::copy(values.begin(), values.end(), mxGetPr(result));
  return result;
}
mxArray * make_output(const binding::ModelOutput & output)
{
  std::array fields{
    "time_s",
    "position_m",
    "velocity_mps",
    "acceleration_mps2",
    "orientation_rad",
    "angular_velocity_radps",
    "angular_acceleration_radps2",
    "wheel_speeds_radps",
    "steering_angle_per_wheel_rad",
    "tire_longitudinal_slip_per_wheel",
    "tire_slip_angle_per_wheel_rad",
    "longitudinal_tire_force_per_wheel_N",
    "lateral_tire_force_per_wheel_N",
    "vertical_tire_force_per_wheel_N",
  };
  mxArray * result = mxCreateStructMatrix(1, 1, static_cast<int>(fields.size()), fields.data());
  mxSetField(result, 0, fields[0], mxCreateDoubleScalar(output.time_s));
  mxSetField(result, 0, fields[1], make_row_vector(output.position_m));
  mxSetField(result, 0, fields[2], make_row_vector(output.velocity_mps));
  mxSetField(result, 0, fields[3], make_row_vector(output.acceleration_mps2));
  mxSetField(result, 0, fields[4], make_row_vector(output.orientation_rad));
  mxSetField(result, 0, fields[5], make_row_vector(output.angular_velocity_radps));
  mxSetField(result, 0, fields[6], make_row_vector(output.angular_acceleration_radps2));
  mxSetField(result, 0, fields[7], make_row_vector(output.wheel_speeds_radps));
  mxSetField(result, 0, fields[8], make_row_vector(output.steering_angle_per_wheel_rad));
  mxSetField(result, 0, fields[9], make_row_vector(output.tire_longitudinal_slip_per_wheel));
  mxSetField(result, 0, fields[10], make_row_vector(output.tire_slip_angle_per_wheel_rad));
  mxSetField(result, 0, fields[11], make_row_vector(output.longitudinal_tire_force_per_wheel_N));
  mxSetField(result, 0, fields[12], make_row_vector(output.lateral_tire_force_per_wheel_N));
  mxSetField(result, 0, fields[13], make_row_vector(output.vertical_tire_force_per_wheel_N));
  return result;
}
template <typename Range>
mxArray * make_string_cells(const Range & values)
{
  mxArray * result = mxCreateCellMatrix(1, static_cast<mwSize>(values.size()));
  std::size_t index = 0;
  for (const auto & value : values) {
    const std::string_view text = [&]() -> std::string_view {
      if constexpr (std::same_as<std::remove_cvref_t<decltype(value)>, std::string>) {
        return value;
      } else {
        return binding::to_string(value);
      }
    }();
    mxSetCell(result, index++, mxCreateString(text.data()));
  }
  return result;
}
std::string_view drivetrain_input_description(const binding::DrivetrainType type)
{
  switch (type) {
    case binding::DrivetrainType::Fx:
      return "real double scalar: longitudinal_force_N";
    case binding::DrivetrainType::WheelTorque:
      return "4-element double vector: drivetrain_input_torque_per_wheel_Nm";
    case binding::DrivetrainType::RwdLsd:
      return "struct: transmission_output_torque_Nm, current_engine_inertia_at_wheels_kgm2, "
             "brake_torque_per_wheel_Nm";
  }
  throw std::invalid_argument("Unsupported drivetrain selection");
}
mxArray * make_info(const binding::Model & current_model)
{
  const auto selection = current_model.selection();
  std::array fields{"interface_version", "supported_model_count", "drivetrain",
                    "steering",          "vehicle_dynamics",      "tire",
                    "aerodynamics",      "time_step_s",           "time_s",
                    "wheel_order",       "drivetrain_input"};
  mxArray * result = mxCreateStructMatrix(1, 1, static_cast<int>(fields.size()), fields.data());
  mxSetField(result, 0, fields[0], mxCreateDoubleScalar(1.0));
  mxSetField(
    result, 0, fields[1],
    mxCreateDoubleScalar(static_cast<double>(binding::supported_model_count())));
  mxSetField(result, 0, fields[2], mxCreateString(binding::to_string(selection.drivetrain).data()));
  mxSetField(result, 0, fields[3], mxCreateString(binding::to_string(selection.steering).data()));
  mxSetField(
    result, 0, fields[4], mxCreateString(binding::to_string(selection.vehicle_dynamics).data()));
  mxSetField(result, 0, fields[5], mxCreateString(binding::to_string(selection.tire).data()));
  mxSetField(
    result, 0, fields[6], mxCreateString(binding::to_string(selection.aerodynamics).data()));
  mxSetField(result, 0, fields[7], mxCreateDoubleScalar(current_model.time_step_s()));
  mxSetField(result, 0, fields[8], mxCreateDoubleScalar(current_model.time_s()));
  mxSetField(
    result, 0, fields[9], mxCreateString("front_left, front_right, rear_left, rear_right"));
  mxSetField(
    result, 0, fields[10],
    mxCreateString(drivetrain_input_description(selection.drivetrain).data()));
  return result;
}
mxArray * make_models()
{
  std::array fields{"drivetrain", "steering",     "vehicle_dynamics",
                    "tire",       "aerodynamics", "combination_count"};
  mxArray * result = mxCreateStructMatrix(1, 1, static_cast<int>(fields.size()), fields.data());
  mxSetField(result, 0, fields[0], make_string_cells(binding::k_drivetrain_types));
  mxSetField(result, 0, fields[1], make_string_cells(binding::k_steering_types));
  mxSetField(result, 0, fields[2], make_string_cells(binding::k_vehicle_dynamics_types));
  mxSetField(result, 0, fields[3], make_string_cells(binding::k_tire_types));
  mxSetField(result, 0, fields[4], make_string_cells(binding::k_aerodynamics_types));
  mxSetField(
    result, 0, fields[5],
    mxCreateDoubleScalar(static_cast<double>(binding::supported_model_count())));
  return result;
}
void create_command(const int nlhs, const int nrhs, const mxArray * const inputs[])
{
  require_argument_count(nlhs, 0, "create", "outputs");
  if (nrhs < 2 || nrhs > 4) {
    throw std::invalid_argument(
      "create expects a model configuration, optionally followed by initial velocity and time "
      "step");
  }
  const auto selection = parse_selection(inputs[1]);
  const double initial_velocity_mps =
    nrhs >= 3 ? scalar_double(inputs[2], "initial longitudinal velocity") : 0.0;
  const double time_step_s = nrhs == 4 ? scalar_double(inputs[3], "time step") : 0.0008;
  ensure_cleanup_registered();
  model = binding::create_model(selection, time_step_s, initial_velocity_mps);
}
void reset_command(const int nlhs, const int nrhs, const mxArray * const inputs[])
{
  require_argument_count(nlhs, 0, "reset", "outputs");
  if (nrhs < 1 || nrhs > 3) {
    throw std::invalid_argument(
      "reset expects optional initial velocity and optional time step arguments");
  }
  auto & current_model = get_model();
  if (nrhs == 3) current_model.set_time_step(scalar_double(inputs[2], "time step"));
  current_model.reset(nrhs >= 2 ? scalar_double(inputs[1], "initial longitudinal velocity") : 0.0);
}
void step_command(
  const int nlhs, mxArray * outputs[], const int nrhs, const mxArray * const inputs[])
{
  require_argument_count(nlhs, 1, "step", "output");
  require_argument_count(nrhs, 3, "step", "inputs");
  auto & current_model = get_model();
  const auto drivetrain_input =
    parse_drivetrain_input(inputs[1], current_model.selection().drivetrain);
  const double steering_angle_rad = scalar_double(inputs[2], "steering angle");
  outputs[0] = make_output(current_model.step(drivetrain_input, steering_angle_rad));
}
void set_external_command(const int nlhs, const int nrhs, const mxArray * const inputs[])
{
  require_argument_count(nlhs, 0, "set_external", "outputs");
  if (nrhs < 1 || nrhs > 2) {
    throw std::invalid_argument("set_external expects zero or one external-influences struct");
  }
  get_model().set_external_influences(
    nrhs == 2 ? parse_external_influences(inputs[1]) : binding::ExternalInfluences{});
}
void set_parameter_command(const int nlhs, const int nrhs, const mxArray * const inputs[])
{
  require_argument_count(nlhs, 0, "set_parameter", "outputs");
  require_argument_count(nrhs, 3, "set_parameter", "inputs");
  auto & current_model = get_model();
  const std::string name = matlab_text(inputs[1], "parameter name");
  const auto current_value = current_model.get_parameter(name);
  binding::ParameterValue new_value;
  if (std::holds_alternative<double>(current_value)) {
    new_value = scalar_double(inputs[2], "parameter value");
  } else {
    new_value = double_vector(inputs[2], "parameter value");
  }
  current_model.set_parameter(name, new_value);
}
void get_parameter_command(
  const int nlhs, mxArray * outputs[], const int nrhs, const mxArray * const inputs[])
{
  require_argument_count(nlhs, 1, "get_parameter", "output");
  require_argument_count(nrhs, 2, "get_parameter", "inputs");
  const auto value = get_model().get_parameter(matlab_text(inputs[1], "parameter name"));
  outputs[0] = std::visit(
    [](const auto & typed_value) -> mxArray * {
      using Value = std::remove_cvref_t<decltype(typed_value)>;
      if constexpr (std::same_as<Value, double>) {
        return mxCreateDoubleScalar(typed_value);
      } else {
        return make_vector(typed_value);
      }
    },
    value);
}
void info_command(const int nlhs, mxArray * outputs[], const int nrhs)
{
  require_argument_count(nlhs, 1, "info", "output");
  require_argument_count(nrhs, 1, "info", "input");
  outputs[0] = make_info(get_model());
}
void models_command(const int nlhs, mxArray * outputs[], const int nrhs)
{
  require_argument_count(nlhs, 1, "models", "output");
  require_argument_count(nrhs, 1, "models", "input");
  outputs[0] = make_models();
}
void list_parameters_command(const int nlhs, mxArray * outputs[], const int nrhs)
{
  require_argument_count(nlhs, 1, "list_parameters", "output");
  require_argument_count(nrhs, 1, "list_parameters", "input");
  outputs[0] = make_string_cells(get_model().list_parameters());
}
}  // namespace
void mexFunction(int nlhs, mxArray * outputs[], int nrhs, const mxArray * inputs[])
{
  try {
    if (nrhs == 0) {
      throw std::invalid_argument(
        "Expected create, reset, step, set_external, set_parameter, get_parameter, "
        "list_parameters, models, or info");
    }
    const std::string command = matlab_text(inputs[0], "command");
    if (command == "create") {
      create_command(nlhs, nrhs, inputs);
    } else if (command == "reset") {
      reset_command(nlhs, nrhs, inputs);
    } else if (command == "step") {
      step_command(nlhs, outputs, nrhs, inputs);
    } else if (command == "set_external") {
      set_external_command(nlhs, nrhs, inputs);
    } else if (command == "set_parameter") {
      set_parameter_command(nlhs, nrhs, inputs);
    } else if (command == "get_parameter") {
      get_parameter_command(nlhs, outputs, nrhs, inputs);
    } else if (command == "list_parameters") {
      list_parameters_command(nlhs, outputs, nrhs);
    } else if (command == "models") {
      models_command(nlhs, outputs, nrhs);
    } else if (command == "info") {
      info_command(nlhs, outputs, nrhs);
    } else {
      throw std::invalid_argument("Unknown command '" + command + "'; run help ocd_matlab");
    }
  } catch (const std::exception & exception) {
    mexErrMsgIdAndTxt("open_car_dynamics:matlab", "%s", exception.what());
  } catch (...) {
    mexErrMsgIdAndTxt("open_car_dynamics:matlab", "%s", "Unknown C++ exception");
  }
}
