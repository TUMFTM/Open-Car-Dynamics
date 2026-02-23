// Copyright 2025 Georg Jank

#include "ocd_drivetrain_wheel_torque_communication_handler_cpp/comm_handler.hpp"
using std::placeholders::_1;
namespace tam::ocd::communication_handlers
{
DrivetrainWheelTorqueCommunicationHandler::DrivetrainWheelTorqueCommunicationHandler(
  rclcpp::Node * node)
: node_(node)
{
  // define subscribers
  sub_driver_input_ = node_->create_subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    std::string(topic_driver_input_), tam::ros::get_qos(),
    std::bind(&DrivetrainWheelTorqueCommunicationHandler::driver_input_callback, this, _1));
  // define feedback publishers
  pub_fb_input_torque_ = node_->create_publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    std::string(topic_fb_input_torque_), tam::ros::get_qos());

  // set initial input
  dt_input_.drivetrain_input_torque_per_wheel_Nm = {0, 0, 0, 0};
  reset_input_delay();
}
// reads driver input messages
void DrivetrainWheelTorqueCommunicationHandler::driver_input_callback(
  const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  dt_input_.drivetrain_input_torque_per_wheel_Nm.front_left = msg->data.front_left;
  dt_input_.drivetrain_input_torque_per_wheel_Nm.front_right = msg->data.front_right;
  dt_input_.drivetrain_input_torque_per_wheel_Nm.rear_left = msg->data.rear_left;
  dt_input_.drivetrain_input_torque_per_wheel_Nm.rear_right = msg->data.rear_right;
  has_new_input_ = true;
}
void DrivetrainWheelTorqueCommunicationHandler::publish_feedback(
  DrivetrainWheelTorqueCommunicationHandler::FeedbackType feedback, rclcpp::Time timestamp)
{
  tum_msgs::msg::TUMFloat64PerWheelStamped fb_input_torque;
  fb_input_torque.stamp = timestamp;
  fb_input_torque.data =
    tam::ocd::helpers::type_conversion::toMsg(feedback.drivetrain_input_torque_per_wheel_Nm);
  pub_fb_input_torque_->publish(fb_input_torque);
}
DrivetrainWheelTorqueCommunicationHandler::DriverInputType
DrivetrainWheelTorqueCommunicationHandler::get_driver_input()
{
  assign_debug_outputs();
  return dt_input_;
}
void DrivetrainWheelTorqueCommunicationHandler::assign_debug_outputs()
{
  debug_container_->log(
    "driver_input/input_torque_per_wheel_Nm", dt_input_.drivetrain_input_torque_per_wheel_Nm);
}
}  // namespace tam::ocd::communication_handlers
