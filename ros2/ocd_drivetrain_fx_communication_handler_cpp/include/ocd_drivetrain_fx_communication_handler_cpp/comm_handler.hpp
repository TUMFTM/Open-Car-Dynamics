// Copyright 2026 Simon Sagmeister

#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ocd_communication_handler_base_cpp/base_class.hpp"
#include "ocd_drivetrain_fx_cpp/drivetrain_fx_model.hpp"
#include "ocd_vehicle_model_node_cpp/helpers.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"
namespace tam::ocd::communication_handlers
{
class DrivetrainFxCommunicationHandler : public tam::ocd::interfaces::CommunicationHandlerBase<
                                           tam::ocd::drivetrain::DrivetrainFxModel::DriverInputType,
                                           tam::ocd::drivetrain::DrivetrainFxModel::FeedbackType>
{
public:
  explicit DrivetrainFxCommunicationHandler(rclcpp::Node * node);

private:
  // Node
  rclcpp::Node * node_;

  // Subscription
  static constexpr std::string_view topic_driver_input_ = "/simulation/actuation/longitudinal";
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_driver_input_{};

  DriverInputType dt_input_;
  bool has_new_input_{true};

  tam::tsl::ValueLogger::SharedPtr debug_container_ = std::make_shared<tam::tsl::ValueLogger>();

  void driver_input_callback(const std_msgs::msg::Float64::SharedPtr msg);

  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();

public:
  // Checks for new driver input messages
  bool get_new_input_flag() { return has_new_input_; }
  void reset_new_input_flag() { has_new_input_ = false; }
  // Input/feedback handling
  DriverInputType get_driver_input();
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() { return param_manager_; }
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() { return debug_container_; }
  void update_delay_timer(std::chrono::duration<double>) override{};
  void reset_input_delay() override{};
};
}  // namespace tam::ocd::communication_handlers
