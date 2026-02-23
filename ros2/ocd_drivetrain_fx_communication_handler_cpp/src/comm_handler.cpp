// Copyright 2026 Georg Jank

#include "ocd_drivetrain_fx_communication_handler_cpp/comm_handler.hpp"
using std::placeholders::_1;
namespace tam::ocd::communication_handlers
{
DrivetrainFxCommunicationHandler::DrivetrainFxCommunicationHandler(rclcpp::Node * node)
: node_(node)
{
  // define subscribers
  sub_driver_input_ = node_->create_subscription<std_msgs::msg::Float64>(
    std::string(topic_driver_input_), tam::ros::get_qos(),
    std::bind(&DrivetrainFxCommunicationHandler::driver_input_callback, this, _1));
}
// reads driver input messages
void DrivetrainFxCommunicationHandler::driver_input_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  dt_input_ = msg->data;
  has_new_input_ = true;
}
DrivetrainFxCommunicationHandler::DriverInputType
DrivetrainFxCommunicationHandler::get_driver_input()
{
  return dt_input_;
}
}  // namespace tam::ocd::communication_handlers