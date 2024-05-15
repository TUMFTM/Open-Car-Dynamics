// Copyright 2023 Simon Sagmeister

#include "vehicle_model_double_track_cpp/double_track_model.hpp"
namespace tam::sim
{
VehicleModelDoubleTrack::VehicleModelDoubleTrack()
{
  param_manager_ = std::make_shared<tam::core::ParamManager>();
  std::vector<tam::interfaces::ParamManagerBase::SharedPtr> param_manager_assembly = {
    vd_model.get_param_manager(), drivetrain_model.get_param_manager(),
    steering_actuator.get_param_manager(), param_manager_};
  param_manager_composed_ =
    std::make_shared<tam::core::ParamManagerComposer>(param_manager_assembly);
  // Declare parameters
  declare_parameters();
  reset();
}
void VehicleModelDoubleTrack::set_driver_input(const tam::types::DriverInput & input)
{
  driver_input = input;
  // Consider time delays
  db_steering.set(internal_time_us_, driver_input.steering_angle_rad);
  db_accel.set(internal_time_us_, driver_input.ax_target);
}
void VehicleModelDoubleTrack::set_external_influences(const tam::types::ExternalInfluences & input)
{
  external_influences = input;
}
tam::types::VehicleModelOutput VehicleModelDoubleTrack::get_output() const { return model_output; }
tam::types::common::TUMDebugContainer::SharedPtr VehicleModelDoubleTrack::get_debug_out() const
{
  return debug_container_;
}
tam::interfaces::ParamManagerBase * VehicleModelDoubleTrack::get_param_manager()
{
  return param_manager_composed_.get();
}
void VehicleModelDoubleTrack::update_delay_blocks()
{
  driver_input_delayed.steering_angle_rad = db_steering.get(internal_time_us_);
  driver_input_delayed.ax_target = db_accel.get(internal_time_us_);
}
void VehicleModelDoubleTrack::step()
{
  // Start of the step -> Query all delay blocks
  update_delay_blocks();

  // Init local variabls
  double h = param_manager_->get_parameter_value("integration_step_size_s").as_double();

  auto x_dot = ode(0, x);

  // Log some stuff
  debug_container_->log("integration_step_size_s", h);
  vd_model.get_debug_container()->transfer_to(debug_container_, "vd_double_track/");
  steering_actuator.get_debug_container()->transfer_to(debug_container_, "steering_actuator/");
  drivetrain_model.get_debug_container()->transfer_to(debug_container_, "drivetrain/");

  // Delay logging
  debug_container_->log("driver_input/steering_angle_rad", driver_input.steering_angle_rad);
  debug_container_->log("driver_input/acceleration", driver_input.ax_target);
  debug_container_->log(
    "driver_input_delayed/steering_angle_rad", driver_input_delayed.steering_angle_rad);
  debug_container_->log("driver_input_delayed/acceleration", driver_input_delayed.ax_target);

  // Bind the function for the integrator
  std::function<state_vector_t(double, state_vector_t)> bound_ode =
    std::bind(&VehicleModelDoubleTrack::ode, this, std::placeholders::_1, std::placeholders::_2);

  // Calculate the new x - Aka do the integration step
  x = tam::helpers::numerical::integration_step_DoPri45<state_vector_t>(bound_ode, 0, x, h);
  // Progress time
  internal_time_us_ += static_cast<uint64_t>(h * 1e6);

  set_state_vector_of_models(x);

  auto dt_output = drivetrain_model.get_y();
  // auto vd_output = vd_model.get_output();

  model_output.position_m.x = x[OFFSET_VD + VD_MODEL_T::state_enum::x_m];
  model_output.position_m.y = x[OFFSET_VD + VD_MODEL_T::state_enum::y_m];
  model_output.orientation_rad.z = x[OFFSET_VD + VD_MODEL_T::state_enum::psi_rad];
  model_output.velocity_mps.x = x[OFFSET_VD + VD_MODEL_T::state_enum::v_x_mps];
  model_output.velocity_mps.y = x[OFFSET_VD + VD_MODEL_T::state_enum::v_y_mps];
  model_output.angular_velocity_radps.z = x[OFFSET_VD + VD_MODEL_T::state_enum::psi_dot_radps];
  // Adapt the local accelerations to global accelerations since this is what the imu would measure
  model_output.acceleration_mps2.x = x_dot[OFFSET_VD + VD_MODEL_T::state_enum::v_x_mps] -
                                     x[OFFSET_VD + VD_MODEL_T::state_enum::psi_dot_radps] *
                                       x[OFFSET_VD + VD_MODEL_T::state_enum::v_y_mps];
  model_output.acceleration_mps2.y = x_dot[OFFSET_VD + VD_MODEL_T::state_enum::v_y_mps] +
                                     x[OFFSET_VD + VD_MODEL_T::state_enum::psi_dot_radps] *
                                       x[OFFSET_VD + VD_MODEL_T::state_enum::v_x_mps];
  model_output.steering_angle = x[OFFSET_SA + SA_MODEL_T::state_enum::position_rad];
  model_output.wheel_speed_radps = dt_output.omega_wheel_radps;
  // is here
}
void VehicleModelDoubleTrack::set_state_vector_of_models(state_vector_t x_)
{
  // Set the state vectors first for all the models
  // Vehicle dynamics model
  vd_model.set_x_vec(x_.segment(OFFSET_VD, VD_MODEL_T::state_vector_length));
  drivetrain_model.set_x_vec(x_.segment(OFFSET_DT, DT_MODEL_T::state_vector_length));
  steering_actuator.set_x_vec(x_.segment(OFFSET_SA, SA_MODEL_T::state_vector_length));
}
VehicleModelDoubleTrack::state_vector_t VehicleModelDoubleTrack::ode(double t, state_vector_t x_)
{
  // Silence warning of unused param
  (void)t;

  state_vector_t x_dot;

  // Evaluate the aero model
  tam::sim::helpers::aerodynamics::Parameters aero_params;
  aero_params.c_d = param_manager_->get_parameter_value("aerodynamics.c_d").as_double();
  aero_params.c_l = param_manager_->get_parameter_value("aerodynamics.c_l").as_double();
  aero_params.A_m2 = param_manager_->get_parameter_value("aerodynamics.A_m2").as_double();
  aero_params.d_aero_center_cog_m =
    param_manager_->get_parameter_value("aerodynamics.d_aero_center_cog_m").as_double();

  tam::sim::helpers::aerodynamics::AeroModelOutput aero_influence_out =
    tam::sim::helpers::aerodynamics::aero_model(
      x_[OFFSET_VD + VD_MODEL_T::state_enum::v_x_mps],
      x_[OFFSET_VD + VD_MODEL_T::state_enum::v_y_mps], aero_params);
  tam::sim::vd_double_track::ExternalInfluences external_influence;
  external_influence.external_forces_cog_N.x =
    aero_influence_out.force_cog_N.x + external_influences.external_force_N.x;
  external_influence.external_forces_cog_N.y =
    aero_influence_out.force_cog_N.y + external_influences.external_force_N.y;
  external_influence.external_forces_cog_N.z =
    aero_influence_out.force_cog_N.z + external_influences.external_force_N.z;
  external_influence.external_moments_cog_Nm.x =
    aero_influence_out.torque_Nm.x + external_influences.external_torque_Nm.x;
  external_influence.external_moments_cog_Nm.y =
    aero_influence_out.torque_Nm.y + external_influences.external_torque_Nm.y;
  external_influence.external_moments_cog_Nm.z =
    aero_influence_out.torque_Nm.z + external_influences.external_torque_Nm.z;

  set_state_vector_of_models(x_);  // Set state vector of models

  // Evaluate steering actuator
  steering_actuator.set_u(driver_input_delayed.steering_angle_rad);
  steering_actuator.evaluate();
  auto y_steering_actuator = steering_actuator.get_y();
  x_dot.segment(OFFSET_SA, SA_MODEL_T::state_vector_length) = steering_actuator.get_x_dot_vec();

  // Set the external forces on the model
  vd_model.set_external_influences(external_influence);

  // region eval the vd model
  vd_double_track::Input vd_input;
  vd_input.steering_angle_tire_rad = y_steering_actuator;
  vd_input.omega_wheel_radps =
    drivetrain_model.get_y().omega_wheel_radps;  // This just depends from the state vector
  vd_model.set_u(vd_input);
  vd_model.evaluate();
  x_dot.segment(OFFSET_VD, VD_MODEL_T::state_vector_length) = vd_model.get_x_dot_vec();
  // endregion

  // region eval the drivetrain model
  drivetrain::Input dt_input;
  dt_input.ax_target = driver_input_delayed.ax_target;
  dt_input.tire_torque_rotational_Nm = vd_model.get_y().tire_torque_rotational_Nm;
  drivetrain_model.set_u(dt_input);
  // TODO Cleanup and consider rolling resistnace
  tam::types::ExternalInfluences ext_infl = external_influences;
  ext_infl.external_force_N.x += aero_influence_out.force_cog_N.x;
  drivetrain_model.set_external_influences(ext_infl);
  drivetrain_model.evaluate();
  x_dot.segment(OFFSET_DT, DT_MODEL_T::state_vector_length) = drivetrain_model.get_x_dot_vec();
  // endregion

  return x_dot;
}
void VehicleModelDoubleTrack::reset()
{
  // Reset the state vector with the parameters from the submodels
  for (std::size_t i = 0; i < VD_MODEL_T::state_vector_length; i++) {
    x[OFFSET_VD + i] =
      param_manager_
        ->get_parameter_value(
          "initial_state.vehicle_dynamics." + std::string(VD_MODEL_T::state_enum_to_string(i)))
        .as_double();
  }
  for (std::size_t i = 0; i < DT_MODEL_T::state_vector_length; i++) {
    x[OFFSET_DT + i] =
      param_manager_
        ->get_parameter_value(
          "initial_state.drivetrain." + std::string(DT_MODEL_T::state_enum_to_string(i)))
        .as_double();
  }
  for (std::size_t i = 0; i < SA_MODEL_T::state_vector_length; i++) {
    x[OFFSET_SA + i] =
      param_manager_
        ->get_parameter_value(
          "initial_state.steering_actuator." + std::string(SA_MODEL_T::state_enum_to_string(i)))
        .as_double();
  }

  // Reset the delay blocks
  db_steering = {
    static_cast<uint64_t>(
      param_manager_->get_parameter_value("delay.steering.actuation_delay_us").as_int()),
    param_manager_->get_parameter_value("delay.steering.initial_value").as_double()};
  db_accel = {
    static_cast<uint64_t>(
      param_manager_->get_parameter_value("delay.acceleration.actuation_delay_us").as_int()),
    param_manager_->get_parameter_value("delay.acceleration.initial_value").as_double()};
}
void VehicleModelDoubleTrack::declare_parameters()
{
  // Declare the integration step size for the model
  param_manager_->declare_parameter(
    "integration_step_size_s", 0.0008, tam::types::param::ParameterType::DOUBLE,
    "Integration step size of the model");

  // Declare parameters for the initial states of all submodels
  for (std::size_t i = 0; i < VD_MODEL_T::state_vector_length; i++) {
    param_manager_->declare_parameter(
      "initial_state.vehicle_dynamics." + std::string(VD_MODEL_T::state_enum_to_string(i)), 0.0,
      tam::types::param::ParameterType::DOUBLE,
      "initial value for the state variable of the vehicle dynamics model");
  }
  for (std::size_t i = 0; i < DT_MODEL_T::state_vector_length; i++) {
    param_manager_->declare_parameter(
      "initial_state.drivetrain." + std::string(DT_MODEL_T::state_enum_to_string(i)), 0.0,
      tam::types::param::ParameterType::DOUBLE,
      "initial value for the state variable of the drivetrain model");
  }
  for (std::size_t i = 0; i < SA_MODEL_T::state_vector_length; i++) {
    param_manager_->declare_parameter(
      "initial_state.steering_actuator." + std::string(SA_MODEL_T::state_enum_to_string(i)), 0.0,
      tam::types::param::ParameterType::DOUBLE,
      "initial value for the state variable of steering actuator");
  }

  // aerodynamics
  param_manager_->declare_parameter(
    "aerodynamics.A_m2", 1.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "aerodynamics.c_d", 1.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "aerodynamics.c_l", -3.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "aerodynamics.d_aero_center_cog_m", 0.0, tam::types::param::ParameterType::DOUBLE, "");

  // Actuation delay
  param_manager_->declare_parameter(
    "delay.steering.actuation_delay_us", 50'000, tam::types::param::ParameterType::INTEGER, "");
  param_manager_->declare_parameter(
    "delay.acceleration.actuation_delay_us", 10'000, tam::types::param::ParameterType::INTEGER, "");
  param_manager_->declare_parameter(
    "delay.steering.initial_value", 0.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "delay.acceleration.initial_value", 0.0, tam::types::param::ParameterType::DOUBLE, "");
}
}  // namespace tam::sim
