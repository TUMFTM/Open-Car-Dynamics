// Copyright 2026 Simon Sagmeister
#include <cmath>

#include <array>
#include <stdexcept>
#include <string>
#include <variant>

#include "ocd_matlab/model.hpp"

namespace binding = tam::ocd::matlab;
template <typename Range>
void require_finite(const Range & values)
{
  for (const double value : values) {
    if (!std::isfinite(value)) throw std::runtime_error("Model output is not finite");
  }
}
void require_finite(const binding::ModelOutput & output)
{
  if (!std::isfinite(output.time_s)) throw std::runtime_error("Model time is not finite");
  require_finite(output.position_m);
  require_finite(output.velocity_mps);
  require_finite(output.acceleration_mps2);
  require_finite(output.orientation_rad);
  require_finite(output.angular_velocity_radps);
  require_finite(output.angular_acceleration_radps2);
  require_finite(output.wheel_speeds_radps);
  require_finite(output.steering_angle_per_wheel_rad);
  require_finite(output.tire_longitudinal_slip_per_wheel);
  require_finite(output.tire_slip_angle_per_wheel_rad);
  require_finite(output.longitudinal_tire_force_per_wheel_N);
  require_finite(output.lateral_tire_force_per_wheel_N);
  require_finite(output.vertical_tire_force_per_wheel_N);
}
int main()
{
  constexpr double time_step_s = 0.001;
  constexpr double initial_velocity_mps = 10.0;
  std::size_t tested_combinations = 0;

  for (const auto drivetrain : binding::k_drivetrain_types) {
    for (const auto steering : binding::k_steering_types) {
      for (const auto dynamics : binding::k_vehicle_dynamics_types) {
        for (const auto tire : binding::k_tire_types) {
          for (const auto aerodynamics : binding::k_aerodynamics_types) {
            const binding::ModelSelection selection{
              .drivetrain = drivetrain,
              .steering = steering,
              .vehicle_dynamics = dynamics,
              .tire = tire,
              .aerodynamics = aerodynamics,
            };
            auto model = binding::create_model(selection, time_step_s, initial_velocity_mps);

            if (
              model->selection().drivetrain != drivetrain ||
              model->selection().vehicle_dynamics != dynamics || model->selection().tire != tire ||
              model->selection().aerodynamics != aerodynamics) {
              throw std::runtime_error("Factory returned the wrong model selection");
            }
            if (model->list_parameters().empty()) {
              throw std::runtime_error("Model did not expose parameters");
            }

            binding::ExternalInfluences external;
            external.wind_mps = {1.0, 0.0, 0.0};
            model->set_external_influences(external);

            binding::DrivetrainInput input;
            if (drivetrain == binding::DrivetrainType::Fx) {
              input.longitudinal_force_N = 10.0;
            } else if (drivetrain == binding::DrivetrainType::WheelTorque) {
              input.drivetrain_input_torque_per_wheel_Nm = {1.0, 1.0, 1.0, 1.0};
            } else {
              input.transmission_output_torque_Nm = 10.0;
              input.current_engine_inertia_at_wheels_kgm2 = 0.1;
            }
            const auto output = model->step(input, 0.001);
            require_finite(output);
            if (std::abs(output.time_s - time_step_s) > 1e-15) {
              throw std::runtime_error("Model step did not advance time correctly");
            }
            ++tested_combinations;
          }
        }
      }
    }
  }

  if (tested_combinations != binding::supported_model_count() || tested_combinations != 48) {
    throw std::runtime_error("Factory coverage count is not 48");
  }

  auto zero_velocity_model = binding::create_model();
  require_finite(zero_velocity_model->step({}, 0.0));
  if (zero_velocity_model->time_s() != 0.0008) {
    throw std::runtime_error("Default zero-velocity model did not advance correctly");
  }
  auto model = binding::create_model({}, time_step_s, 5.0);
  const std::string mass_parameter = "vehicle_dynamics_double_track.mass_vehicle_kg";
  const auto original_mass = model->get_parameter(mass_parameter);
  if (!std::holds_alternative<double>(original_mass)) {
    throw std::runtime_error("Scalar parameter did not round-trip as a scalar");
  }
  model->set_parameter(mass_parameter, 900.0);
  if (std::get<double>(model->get_parameter(mass_parameter)) != 900.0) {
    throw std::runtime_error("Scalar parameter update failed");
  }

  const std::string radius_scaling_parameter =
    "vehicle_dynamics_double_track.tire.rolling_radius_m.velocity_scaling.factors";
  const auto original_scaling = model->get_parameter(radius_scaling_parameter);
  if (!std::holds_alternative<std::vector<double>>(original_scaling)) {
    throw std::runtime_error("Array parameter did not round-trip as a vector");
  }
  model->set_parameter(radius_scaling_parameter, std::vector<double>{1.0, 1.01});
  if (
    std::get<std::vector<double>>(model->get_parameter(radius_scaling_parameter)) !=
    std::vector<double>({1.0, 1.01})) {
    throw std::runtime_error("Array parameter update failed");
  }

  model->reset(5.0);
  binding::DrivetrainInput input;
  for (int step = 0; step < 1000; ++step) {
    input.longitudinal_force_N = 500.0;
    require_finite(model->step(input, 0.01));
  }
  if (std::abs(model->time_s() - 1.0) > 1e-12) {
    throw std::runtime_error("One-second simulation did not advance time correctly");
  }

  bool invalid_time_step_rejected = false;
  try {
    [[maybe_unused]] auto invalid_model = binding::create_model({}, 0.0, 0.0);
  } catch (const std::invalid_argument &) {
    invalid_time_step_rejected = true;
  }
  if (!invalid_time_step_rejected) {
    throw std::runtime_error("Invalid time step was not rejected");
  }
  return 0;
}
