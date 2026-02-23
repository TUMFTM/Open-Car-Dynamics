// Copyright 2025 Georg Jank

#include "ocd_steering_actuator_pt1_communication_handler_cpp/comm_handler.hpp"

using std::placeholders::_1;
namespace tam::ocd::communication_handlers
{
SteeringActuatorPT1CommunicationHandler::SteeringActuatorPT1CommunicationHandler(
  rclcpp::Node * node)
: node_(node)
{
  declare_delay_parameters();
  // define subscribers
  sub_driver_input_ =
    node_->create_subscription<autoware_auto_control_msgs::msg::AckermannLateralCommand>(
      std::string(topic_driver_input_), tam::ros::get_qos(),
      std::bind(&SteeringActuatorPT1CommunicationHandler::driver_input_callback, this, _1));
  // define feedback publishers
  pub_fb_steering_angle_ = node_->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    std::string(topic_fb_steering_angle_), tam::ros::get_qos());
  // set initial input
  sa_input_.steering_angle_rad = 0.0;
  sa_delayed_input_.steering_angle_rad = 0.0;
  reset_input_delay();
}
// reads driver input messages
void SteeringActuatorPT1CommunicationHandler::driver_input_callback(
  const autoware_auto_control_msgs::msg::AckermannLateralCommand::SharedPtr msg)
{
  sa_input_.steering_angle_rad = msg->steering_tire_angle;
  steering_angle_delay_.set(internal_time_us_, sa_input_.steering_angle_rad);
}
void SteeringActuatorPT1CommunicationHandler::publish_feedback(
  SteeringActuatorPT1CommunicationHandler::FeedbackType feedback, rclcpp::Time timestamp)
{
  autoware_auto_vehicle_msgs::msg::SteeringReport steering_report;
  steering_report.stamp = timestamp;
  steering_report.steering_tire_angle = feedback.steering_angle_rad;
  pub_fb_steering_angle_->publish(steering_report);
}
SteeringActuatorPT1CommunicationHandler::DriverInputType
SteeringActuatorPT1CommunicationHandler::get_driver_input()
{
  sa_delayed_input_.steering_angle_rad = steering_angle_delay_.get(internal_time_us_);
  has_new_input_ = steering_angle_delay_.get_new_signal_flag();
  assign_debug_outputs();
  return sa_delayed_input_;
}
void SteeringActuatorPT1CommunicationHandler::declare_delay_parameters()
{
  param_manager_->declare_parameter(
    "delay.steering.actuation_delay_us", 60'000, tam::pmg::ParameterType::INTEGER, "");
  param_manager_->declare_parameter(
    "delay.steering.initial_value", 0.0, tam::pmg::ParameterType::DOUBLE, "");
}
void SteeringActuatorPT1CommunicationHandler::reset_input_delay()
{
  steering_angle_delay_ = {
    static_cast<uint64_t>(param_manager_->get_value("delay.steering.actuation_delay_us").as_int()),
    param_manager_->get_value("delay.steering.initial_value").as_double()};
}
void SteeringActuatorPT1CommunicationHandler::assign_debug_outputs()
{
  debug_container_->log("driver_input/steering_angle_rad", sa_input_.steering_angle_rad);
  debug_container_->log(
    "driver_input_delayed/steering_angle_rad", sa_delayed_input_.steering_angle_rad);
}
void SteeringActuatorPT1CommunicationHandler::update_delay_timer(
  std::chrono::duration<double> integration_step_s)
{
  uint64_t time_step_us =
    std::chrono::duration_cast<std::chrono::microseconds>(integration_step_s).count();
  internal_time_us_ += time_step_us;
}
bool SteeringActuatorPT1CommunicationHandler::get_new_input_flag()
{
  get_driver_input();
  return has_new_input_;
}
}  // namespace tam::ocd::communication_handlers
