// Copyright 2026 Simon Sagmeister
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "ocd_vehicle_model_base_cpp/base_class.hpp"
#include "vehicle_model.hpp"
namespace tam::ocd
{
#define VEHICLE_T VehicleModel<DT_MODEL_T, SA_MODEL_T, VD_MODEL_T>
template <
  interfaces::concepts::DrivetrainModel DT_MODEL_T,
  interfaces::concepts::SteeringActuatorModel SA_MODEL_T,
  interfaces::concepts::VehicleDynamicsModel VD_MODEL_T>
VEHICLE_T::VehicleModel()
{
  param_manager_ = std::make_shared<tam::pmg::ParamValueManager>();
  std::vector<tam::pmg::MgmtInterface::SharedPtr> param_manager_assembly = {
    vehicle_dynamics_model_.get_param_manager(), drivetrain_model_.get_param_manager(),
    steering_actuator_model_.get_param_manager(), param_manager_};
  param_manager_composed_ =
    std::make_shared<tam::pmg::ParamManagerComposer>(param_manager_assembly);
  // Declare parameters
  declare_parameters();
  model_output_.vehicle_dynamics_output.orientation_rad.x = 0.0;
  model_output_.vehicle_dynamics_output.orientation_rad.y = 0.0;
  model_output_.vehicle_dynamics_output.orientation_rad.z = 0.0;
  // <--
  reset();

  // Assemble composed logger
  debug_container_composed_->register_logger(
    vehicle_dynamics_model_.get_logger(), "vehicle_dynamics/");
  debug_container_composed_->register_logger(
    steering_actuator_model_.get_logger(), "steering_actuator/");
  debug_container_composed_->register_logger(drivetrain_model_.get_logger(), "drivetrain/");
}
template <
  interfaces::concepts::DrivetrainModel DT_MODEL_T,
  interfaces::concepts::SteeringActuatorModel SA_MODEL_T,
  interfaces::concepts::VehicleDynamicsModel VD_MODEL_T>
void VEHICLE_T::step()
{
  // Init local variables
  double h = param_manager_->get_value("integration_step_size_s").as_double();

  // Log some stuff
  debug_container_->log("integration_step_size_s", h);

  // Bind the function for the integrator
  std::function<state_vector_t(double, state_vector_t)> bound_ode =
    std::bind(&VEHICLE_T::ode, this, std::placeholders::_1, std::placeholders::_2);

  // Calculate the new x - Aka do the integration step
  x = tam::helpers::numerical::integration_step_DoPri45<state_vector_t>(bound_ode, 0, x, h);

  set_state_vector_of_models(x);

  model_output_.vehicle_dynamics_output = vehicle_dynamics_output_;
  model_output_.wheel_speeds_radps = wheel_speeds_radps_;
  model_output_.drivetrain_load_torque_per_wheel_Nm = drivetrain_load_torque_per_wheel_Nm_;
  model_output_.steering_angle_per_wheel_rad = steering_angle_per_wheel_rad_;
  model_output_.steering_load_torque_per_wheel_Nm = steering_load_torque_per_wheel_Nm_;
}
template <
  interfaces::concepts::DrivetrainModel DT_MODEL_T,
  interfaces::concepts::SteeringActuatorModel SA_MODEL_T,
  interfaces::concepts::VehicleDynamicsModel VD_MODEL_T>
void VEHICLE_T::set_state_vector_of_models(state_vector_t x_)
{
  // Set the state vectors first for all the models
  // Vehicle dynamics model
  vehicle_dynamics_model_.set_x_vec(x_.segment(OFFSET_VD, VD_MODEL_T::k_state_vector_length));
  drivetrain_model_.set_x_vec(x_.segment(OFFSET_DT, DT_MODEL_T::k_state_vector_length));
  steering_actuator_model_.set_x_vec(x_.segment(OFFSET_SA, SA_MODEL_T::k_state_vector_length));
}
template <
  interfaces::concepts::DrivetrainModel DT_MODEL_T,
  interfaces::concepts::SteeringActuatorModel SA_MODEL_T,
  interfaces::concepts::VehicleDynamicsModel VD_MODEL_T>
VEHICLE_T::state_vector_t VEHICLE_T::ode(double t, state_vector_t x_)
{
  // Silence warning of unused param
  (void)t;

  state_vector_t x_dot_;

  set_state_vector_of_models(x_);  // Set state vector of models

  // Evaluate steering actuator
  steering_actuator_model_.set_driver_input(steering_actuator_driver_input_);
  steering_actuator_model_.set_load(steering_load_torque_per_wheel_Nm_);
  steering_actuator_model_.evaluate();
  steering_angle_per_wheel_rad_ = steering_actuator_model_.get_steering_angles();
  if constexpr (!std::is_void_v<typename SA_MODEL_T::FeedbackType>) {
    steering_actuator_feedback_ = steering_actuator_model_.get_feedback();
  }
  x_dot_.segment(OFFSET_SA, SA_MODEL_T::k_state_vector_length) =
    steering_actuator_model_.get_x_dot_vec();

  // region eval the vd model
  vehicle_dynamics_model_.set_wheel_speeds(wheel_speeds_radps_);
  vehicle_dynamics_model_.set_steering_angles(steering_angle_per_wheel_rad_);
  vehicle_dynamics_model_.set_external_influences(external_influences_);
  vehicle_dynamics_model_.evaluate();
  drivetrain_load_torque_per_wheel_Nm_ = vehicle_dynamics_model_.get_wheel_load();
  steering_load_torque_per_wheel_Nm_ = vehicle_dynamics_model_.get_steering_load();
  vehicle_dynamics_output_ = vehicle_dynamics_model_.get_vehicle_dynamics_output();
  x_dot_.segment(OFFSET_VD, VD_MODEL_T::k_state_vector_length) =
    vehicle_dynamics_model_.get_x_dot_vec();
  // endregion

  // region eval the drivetrain model
  drivetrain_model_.set_driver_input(drivetrain_driver_input_);
  drivetrain_model_.set_load(drivetrain_load_torque_per_wheel_Nm_);
  drivetrain_model_.evaluate();
  wheel_speeds_radps_ = drivetrain_model_.get_wheel_speeds();
  if constexpr (!std::is_void_v<typename DT_MODEL_T::FeedbackType>) {
    drivetrain_feedback_ = drivetrain_model_.get_feedback();
  }
  x_dot_.segment(OFFSET_DT, DT_MODEL_T::k_state_vector_length) = drivetrain_model_.get_x_dot_vec();
  // endregion

  return x_dot_;
}
template <
  interfaces::concepts::DrivetrainModel DT_MODEL_T,
  interfaces::concepts::SteeringActuatorModel SA_MODEL_T,
  interfaces::concepts::VehicleDynamicsModel VD_MODEL_T>
void VEHICLE_T::reset()
{
  // Reset the state vector with the parameters from the submodels
  for (std::size_t i = 0; i < VD_MODEL_T::k_state_vector_length; i++) {
    x[OFFSET_VD + i] =
      param_manager_
        ->get_value(
          "initial_state.vehicle_dynamics." + std::string(VD_MODEL_T::StateNamesTrait::value[i]))
        .as_double();
  }
  for (std::size_t i = 0; i < DT_MODEL_T::k_state_vector_length; i++) {
    x[OFFSET_DT + i] =
      param_manager_
        ->get_value(
          "initial_state.drivetrain." + std::string(DT_MODEL_T::StateNamesTrait::value[i]))
        .as_double();
  }
  for (std::size_t i = 0; i < SA_MODEL_T::k_state_vector_length; i++) {
    x[OFFSET_SA + i] =
      param_manager_
        ->get_value(
          "initial_state.steering_actuator." + std::string(SA_MODEL_T::StateNamesTrait::value[i]))
        .as_double();
  }

  // Reset the models inputs as well
  steering_actuator_driver_input_ = {};
  drivetrain_driver_input_ = {};
  steering_load_torque_per_wheel_Nm_ = {};
  wheel_speeds_radps_ = {};
  steering_angle_per_wheel_rad_ = {};
  external_influences_ = {};
  drivetrain_load_torque_per_wheel_Nm_ = {};
}
template <
  interfaces::concepts::DrivetrainModel DT_MODEL_T,
  interfaces::concepts::SteeringActuatorModel SA_MODEL_T,
  interfaces::concepts::VehicleDynamicsModel VD_MODEL_T>
void VEHICLE_T::declare_parameters()
{
  // Declare the integration step size for the model
  param_manager_->declare_parameter(
    "integration_step_size_s", 0.0008, tam::pmg::ParameterType::DOUBLE,
    "Integration step size of the model");

  // Declare parameters for the initial states of all submodels
  for (std::size_t i = 0; i < VD_MODEL_T::k_state_vector_length; i++) {
    param_manager_->declare_parameter(
      "initial_state.vehicle_dynamics." + std::string(VD_MODEL_T::StateNamesTrait::value[i]), 0.0,
      tam::pmg::ParameterType::DOUBLE,
      "initial value for the state variable of the vehicle dynamics model");
  }
  for (std::size_t i = 0; i < DT_MODEL_T::k_state_vector_length; i++) {
    param_manager_->declare_parameter(
      std::string("initial_state.drivetrain.") + std::string(DT_MODEL_T::StateNamesTrait::value[i]),
      0.0, tam::pmg::ParameterType::DOUBLE,
      "initial value for the state variable of the drivetrain model");
  }
  for (std::size_t i = 0; i < SA_MODEL_T::k_state_vector_length; i++) {
    param_manager_->declare_parameter(
      "initial_state.steering_actuator." + std::string(SA_MODEL_T::StateNamesTrait::value[i]), 0.0,
      tam::pmg::ParameterType::DOUBLE, "initial value for the state variable of steering actuator");
  }
}
#undef VEHICLE_T
}  // namespace tam::ocd
