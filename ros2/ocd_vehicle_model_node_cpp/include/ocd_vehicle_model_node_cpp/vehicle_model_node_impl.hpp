// Copyright 2026 Simon Sagmeister
#pragma once
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ocd_vehicle_model_node_cpp/helpers.hpp"
#include "ocd_vehicle_model_node_cpp/vehicle_model_node.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_ros_helpers_cpp/timer.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/coordinate_frames.hpp"
namespace tam::ocd
{
template <
  interfaces::concepts::VehicleModel VEHICLE_T,
  interfaces::concepts::CommunicationHandler DT_COMM_HANDLER_T,
  interfaces::concepts::CommunicationHandler SA_COMM_HANDLER_T>
VehicleModelNode<VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::VehicleModelNode(
  std::unique_ptr<VEHICLE_T> && veh_model, const rclcpp::NodeOptions & options)
: Node(std::string(node_name_), std::string(namespace_), options), veh_model_(std::move(veh_model))
{
  high_frequency_logging_ =
    this->declare_parameter<bool>("enable_high_frequency_output", high_frequency_logging_);
  double output_rate_hz = this->declare_parameter<double>("output_rate_hz", 100.0);
  
  enable_step_size_compensation_ = this->declare_parameter<bool>(
    "time_skew.enable_step_size_compensation", enable_step_size_compensation_);
  // Time skew monitoring / compensation tuning parameters.
  time_skew_filter_alpha_ =
    this->declare_parameter<double>("time_skew.filter_alpha", time_skew_filter_alpha_);
  time_skew_threshold_ =
    this->declare_parameter<double>("time_skew.warn_threshold", time_skew_threshold_);
  step_size_compensation_max_deviation_ = this->declare_parameter<double>(
    "time_skew.step_size_compensation_max_relative", step_size_compensation_max_deviation_);

  dt_comm_handler_ = std::make_shared<DT_COMM_HANDLER_T>(this);
  sa_comm_handler_ = std::make_shared<SA_COMM_HANDLER_T>(this);

  try {
    param_manager_composer_ = std::make_shared<tam::pmg::ParamManagerComposer>(
      std::vector<tam::pmg::MgmtInterface::SharedPtr>{
        veh_model_->get_param_manager(), dt_comm_handler_->get_param_manager(),
        sa_comm_handler_->get_param_manager()});
    callback_handle_ = tam::pmg::connect_param_manager_to_ros_cb(this, param_manager_composer_);
    tam::pmg::declare_ros_params_from_param_manager(this, param_manager_composer_.get());
  } catch (const std::out_of_range & e) {
    std::cerr << "Parameter manager error: " << e.what() << std::endl;
    throw;
  }

  logger_ = std::make_shared<tam::tsl::ValueLogger>();
  logger_composer_ = std::make_shared<tam::tsl::LoggerComposer>(
    std::vector<tam::tsl::LoggerAccessInterface::SharedPtr>(
      {logger_, veh_model_->get_logger(), dt_comm_handler_->get_logger(),
       sa_comm_handler_->get_logger()}));
  tsl_pub_ = std::make_unique<tam::tsl::TSLPublisher>(this, logger_composer_);

  // model update timer
  timer_period_s_ =
    std::chrono::duration<double>(this->get_parameter("integration_step_size_s").as_double());
  model_update_timer_ = tam::create_timer(
    this, timer_period_s_,
    std::bind(
      &VehicleModelNode<VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::model_update_callback,
      this));

  if (!high_frequency_logging_) {
    if (output_rate_hz <= 0.0) {
      RCLCPP_ERROR(
        this->get_logger(), "output_rate_hz must be > 0, got %f. Falling back to 100 Hz.",
        output_rate_hz);
      output_rate_hz = 100.0;
    }
    output_timer_ = tam::create_timer(
      this, std::chrono::duration<double>(1.0 / output_rate_hz),
      std::bind(
        &VehicleModelNode<VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::output_callback, this));
  }

  auto qos = tam::ros::get_qos(tam::ros::TopicType::DEFAULT);

  sub_external_influences_ = this->create_subscription<ocd_interfaces::msg::ExternalInfluences>(
    std::string(topic_sub_external_influences_), qos,
    std::bind(
      &VehicleModelNode<
        VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::external_influences_callback,
      this, std::placeholders::_1));

  odometry_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(std::string(topic_pub_odometry_), qos);
  accel_pub_ = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    std::string(topic_pub_accel_), qos);
  wheel_speed_pub_ = this->create_publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    std::string(topic_pub_wheelspeeds_), qos);
  tire_force_long_pub_ = this->create_publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    std::string(topic_pub_tire_force_long_), qos);
  tire_force_lat_pub_ = this->create_publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    std::string(topic_pub_tire_force_lat_), qos);
  tire_force_vert_pub_ = this->create_publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    std::string(topic_pub_tire_force_vert_), qos);
  long_slip_pub_ = this->create_publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    std::string(topic_pub_long_slip_), qos);
  slip_angle_pub_ = this->create_publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    std::string(topic_pub_slip_angle_), qos);

  // set initial state
  external_input_.external_force_N = tam::types::common::Vector3D<double>(0.0, 0.0, 0.0);
  external_input_.external_torque_Nm = tam::types::common::Vector3D<double>(0.0, 0.0, 0.0);
  external_input_.lambda_mue = tam::types::common::DataPerWheel<double>(1.0, 1.0, 1.0, 1.0);
}
template <
  interfaces::concepts::VehicleModel VEHICLE_T,
  interfaces::concepts::CommunicationHandler DT_COMM_HANDLER_T,
  interfaces::concepts::CommunicationHandler SA_COMM_HANDLER_T>
void VehicleModelNode<VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::model_update_callback()
{
  rclcpp::Time callback_time = this->get_clock()->now();
  const std::chrono::microseconds nominal_step =
    std::chrono::duration_cast<std::chrono::microseconds>(timer_period_s_);

  std::chrono::microseconds callback_dt = nominal_step;
  if (initial_cycle_) {
    first_callback_time_ = callback_time;
    last_callback_time_ = callback_time;
  } else {
    callback_dt = (callback_time - last_callback_time_).to_chrono<std::chrono::microseconds>();
    last_callback_time_ = callback_time;
  }

  const std::chrono::microseconds elapsed_since_start =
    (callback_time - first_callback_time_).to_chrono<std::chrono::microseconds>();
  const std::chrono::microseconds accumulated_skew = elapsed_since_start - sim_time_since_start_;

  // Choose the integration step size. If compensation is enabled, add the accumulated skew on top
  // of the nominal step so the simulation catches up, clamped to a bounded deviation from nominal.
  std::chrono::microseconds effective_step = nominal_step;
  if (enable_step_size_compensation_) {
    const std::chrono::microseconds max_deviation =
      std::chrono::duration_cast<std::chrono::microseconds>(
        nominal_step * step_size_compensation_max_deviation_);
    effective_step = std::clamp(
      nominal_step + accumulated_skew, nominal_step - max_deviation, nominal_step + max_deviation);
    veh_model_->get_param_manager()->set_value(
      "integration_step_size_s", std::chrono::duration<double>(effective_step).count());
  }

  sim_time_since_start_ += effective_step;

  const double skew_rate =
    static_cast<double>(callback_dt.count()) / static_cast<double>(nominal_step.count());
  filtered_skew_rate_ += time_skew_filter_alpha_ * (skew_rate - filtered_skew_rate_);

  // Get the chrono time to actually have an execution time measurement (not affected by sim time)
  auto t_start = std::chrono::high_resolution_clock::now().time_since_epoch();

  update_model_inputs();

  if (has_new_external_influence_) update_external_influences();

  veh_model_->step();
  dt_comm_handler_->update_delay_timer(effective_step);
  sa_comm_handler_->update_delay_timer(effective_step);

  auto t_stop = std::chrono::high_resolution_clock::now().time_since_epoch();

  logger_->log(
    "node/model_step_time_us",
    std::chrono::duration_cast<std::chrono::microseconds>(t_stop - t_start).count());
  logger_->log("node/time_between_callbacks_us", callback_dt.count());
  logger_->log("node/effective_step_size_us", effective_step.count());
  logger_->log("node/accumulated_skew_us", accumulated_skew.count());
  logger_->log("node/skew_rate", skew_rate);
  logger_->log("node/skew_rate_filtered", filtered_skew_rate_);

  if (std::abs(filtered_skew_rate_ - 1.0) > time_skew_threshold_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Filtered simulation time skew rate is %.2f%% off real-time (threshold %.2f%%). The "
      "simulation is not keeping up with real-time.",
      (filtered_skew_rate_ - 1.0) * 100.0, time_skew_threshold_ * 100.0);
  }

  if (initial_cycle_) {
    initial_cycle_ = false;
    return;
  }

  if ((t_stop - t_start) > timer_period_s_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Model exceeded %ldus update time!\nPhysics no longer guaranteed to be correct!",
      std::chrono::duration_cast<std::chrono::microseconds>(timer_period_s_).count());
  }

  if (high_frequency_logging_) {
    output_callback();
  }
}
template <
  interfaces::concepts::VehicleModel VEHICLE_T,
  interfaces::concepts::CommunicationHandler DT_COMM_HANDLER_T,
  interfaces::concepts::CommunicationHandler SA_COMM_HANDLER_T>
void VehicleModelNode<VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::output_callback()
{
  if (initial_cycle_) return;
  typename types::VehicleModelOutput vehicle_model_output = veh_model_->get_vehicle_model_output();
  nav_msgs::msg::Odometry odom =
    tam::ocd::helpers::type_conversion::toMsg(vehicle_model_output.vehicle_dynamics_output);
  rclcpp::Time stamp = get_clock()->now();

  odom.header.stamp = stamp;
  odom.header.frame_id = CoordinateFrames::local_cartesian;
  odom.child_frame_id = CoordinateFrames::vehicle_cg;
  odometry_pub_->publish(odom);

  tum_msgs::msg::TUMFloat64PerWheelStamped wheel_speed;
  wheel_speed.stamp = stamp;
  wheel_speed.data =
    tam::ocd::helpers::type_conversion::toMsg(vehicle_model_output.wheel_speeds_radps);
  wheel_speed_pub_->publish(wheel_speed);

  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.header.stamp = stamp;
  accel.header.frame_id = CoordinateFrames::vehicle_cg;
  accel.accel.accel.linear = tam::ocd::helpers::type_conversion::toMsg(
    vehicle_model_output.vehicle_dynamics_output.acceleration_mps2);
  accel.accel.accel.angular = tam::ocd::helpers::type_conversion::toMsg(
    vehicle_model_output.vehicle_dynamics_output.angular_acceleration_radps2);
  accel_pub_->publish(accel);

  tum_msgs::msg::TUMFloat64PerWheelStamped tire_force_long;
  tire_force_long.stamp = stamp;
  tire_force_long.data = tam::ocd::helpers::type_conversion::toMsg(
    vehicle_model_output.vehicle_dynamics_output.longitudinal_tire_force_tire_frame_per_wheel_N);
  tire_force_long_pub_->publish(tire_force_long);

  tum_msgs::msg::TUMFloat64PerWheelStamped tire_force_lat;
  tire_force_lat.stamp = stamp;
  tire_force_lat.data = tam::ocd::helpers::type_conversion::toMsg(
    vehicle_model_output.vehicle_dynamics_output.lateral_tire_force_tire_frame_per_wheel_N);
  tire_force_lat_pub_->publish(tire_force_lat);

  tum_msgs::msg::TUMFloat64PerWheelStamped tire_force_vert;
  tire_force_vert.stamp = stamp;
  tire_force_vert.data = tam::ocd::helpers::type_conversion::toMsg(
    vehicle_model_output.vehicle_dynamics_output.vertical_tire_force_per_wheel_N);
  tire_force_vert_pub_->publish(tire_force_vert);

  tum_msgs::msg::TUMFloat64PerWheelStamped long_slip;
  long_slip.stamp = stamp;
  long_slip.data = tam::ocd::helpers::type_conversion::toMsg(
    vehicle_model_output.vehicle_dynamics_output.tire_longitudinal_slip_per_wheel);
  long_slip_pub_->publish(long_slip);

  tum_msgs::msg::TUMFloat64PerWheelStamped slip_angle;
  slip_angle.stamp = stamp;
  slip_angle.data = tam::ocd::helpers::type_conversion::toMsg(
    vehicle_model_output.vehicle_dynamics_output.tire_slip_angle_per_wheel_rad);
  slip_angle_pub_->publish(slip_angle);

  // Static assert that the input types match
  if constexpr (!std::is_void_v<typename VEHICLE_T::DrivetrainFeedbackType>) {
    static_assert(
      std::convertible_to<
        typename VEHICLE_T::DrivetrainFeedbackType, typename DT_COMM_HANDLER_T::FeedbackType>,
      "Type mismatch: VEHICLE_T::get_drivetrain_feedback() must return a type convertible to "
      "DT_COMM_HANDLER_T::Feedback (or provide an adapter/conversion).");
    dt_comm_handler_->publish_feedback(veh_model_->get_drivetrain_feedback(), stamp);
  }
  if constexpr (!std::is_void_v<typename VEHICLE_T::SteeringActuatorFeedbackType>) {
    static_assert(
      std::convertible_to<
        typename VEHICLE_T::SteeringActuatorFeedbackType, typename SA_COMM_HANDLER_T::FeedbackType>,
      "Type mismatch: VEHICLE_T::get_steering_actuator_feedback() must return a type convertible "
      "to "
      "SA_COMM_HANDLER_T::Feedback (or provide an adapter/conversion).");
    sa_comm_handler_->publish_feedback(veh_model_->get_steering_actuator_feedback(), stamp);
  }

  // Publish the debug outputs
  tsl_pub_->trigger();
}
template <
  interfaces::concepts::VehicleModel VEHICLE_T,
  interfaces::concepts::CommunicationHandler DT_COMM_HANDLER_T,
  interfaces::concepts::CommunicationHandler SA_COMM_HANDLER_T>
void VehicleModelNode<VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::update_model_inputs()
{
  // Static assert that the input types match
  static_assert(
    std::convertible_to<
      typename DT_COMM_HANDLER_T::DriverInputType, typename VEHICLE_T::DrivetrainDriverInputType>,
    "Type mismatch: DT_COMM_HANDLER_T::get_driver_input() must return a type convertible to "
    "VEHICLE_T::DrivetrainDriverInput (or provide an adapter/conversion).");
  static_assert(
    std::convertible_to<
      typename SA_COMM_HANDLER_T::DriverInputType,
      typename VEHICLE_T::SteeringActuatorDriverInputType>,
    "Type mismatch: SA_COMM_HANDLER_T::get_driver_input() must return a type convertible to "
    "VEHICLE_T::SteeringActuatorDriverInput (or provide an adapter/conversion).");

  // Only check that inputs match if they are required
  if constexpr (!std::is_void_v<typename VEHICLE_T::DrivetrainAuxiliaryInputType>) {
    static_assert(
      std::convertible_to<
        typename DT_COMM_HANDLER_T::AuxiliaryInputType,
        typename VEHICLE_T::DrivetrainAuxiliaryInputType>,
      "Type mismatch: DT_COMM_HANDLER_T::get_auxiliary_input() must return a type convertible to "
      "VEHICLE_T::DrivetrainAuxiliaryInputType (or provide an adapter/conversion).");
  }
  if constexpr (!std::is_void_v<typename VEHICLE_T::SteeringActuatorAuxiliaryInputType>) {
    static_assert(
      std::convertible_to<
        typename SA_COMM_HANDLER_T::AuxiliaryInputType,
        typename VEHICLE_T::SteeringActuatorAuxiliaryInputType>,
      "Type mismatch: SA_COMM_HANDLER_T::get_auxiliary_input() must return a type convertible to "
      "VEHICLE_T::SteeringActuatorAuxiliaryInputType (or provide an adapter/conversion).");
  }
  if (dt_comm_handler_->get_new_input_flag()) {
    veh_model_->set_drivetrain_input(dt_comm_handler_->get_driver_input());
    dt_comm_handler_->reset_new_input_flag();
  }
  if (sa_comm_handler_->get_new_input_flag()) {
    veh_model_->set_steering_input(sa_comm_handler_->get_driver_input());
    sa_comm_handler_->reset_new_input_flag();
  }
  if constexpr (!std::is_void_v<typename VEHICLE_T::DrivetrainAuxiliaryInputType>) {
    if (dt_comm_handler_->get_new_auxiliary_input_flag()) {
      veh_model_->set_auxiliary_input_drivetrain(dt_comm_handler_->get_auxiliary_input());
      dt_comm_handler_->reset_new_auxiliary_input_flag();
    }
  }
  if constexpr (!std::is_void_v<typename VEHICLE_T::SteeringActuatorAuxiliaryInputType>) {
    if (sa_comm_handler_->get_new_auxiliary_input_flag()) {
      veh_model_->set_auxiliary_input_steering_actuator(sa_comm_handler_->get_auxiliary_input());
      sa_comm_handler_->reset_new_auxiliary_input_flag();
    }
  }
}
template <
  interfaces::concepts::VehicleModel VEHICLE_T,
  interfaces::concepts::CommunicationHandler DT_COMM_HANDLER_T,
  interfaces::concepts::CommunicationHandler SA_COMM_HANDLER_T>
void VehicleModelNode<VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::update_external_influences()
{
  veh_model_->set_external_influences(external_input_);
  has_new_external_influence_ = false;
}
template <
  interfaces::concepts::VehicleModel VEHICLE_T,
  interfaces::concepts::CommunicationHandler DT_COMM_HANDLER_T,
  interfaces::concepts::CommunicationHandler SA_COMM_HANDLER_T>
void VehicleModelNode<VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::
  external_influences_callback(const ocd_interfaces::msg::ExternalInfluences::SharedPtr msg)
{
  has_new_external_influence_ = true;
  external_input_ = tam::ocd::helpers::type_conversion::external_influences_type_from_msg(*msg);
}
template <
  interfaces::concepts::VehicleModel VEHICLE_T,
  interfaces::concepts::CommunicationHandler DT_COMM_HANDLER_T,
  interfaces::concepts::CommunicationHandler SA_COMM_HANDLER_T>
void VehicleModelNode<VEHICLE_T, DT_COMM_HANDLER_T, SA_COMM_HANDLER_T>::reset()
{
  veh_model_->reset();
  dt_comm_handler_->reset_input_delay();
  sa_comm_handler_->reset_input_delay();
}
}  // namespace tam::ocd
