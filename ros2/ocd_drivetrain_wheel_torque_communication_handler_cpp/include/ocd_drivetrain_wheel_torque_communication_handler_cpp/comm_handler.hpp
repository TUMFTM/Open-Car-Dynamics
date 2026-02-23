// Copyright 2025 Georg Jank

#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ocd_communication_handler_base_cpp/base_class.hpp"
#include "ocd_drivetrain_wheel_torque_cpp/drivetrain_direct_torque_model.hpp"
#include "ocd_vehicle_model_node_cpp/helpers.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tum_msgs/msg/tum_float32_stamped.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "tum_msgs/msg/tum_int8_stamped.hpp"
#include "tum_msgs/msg/tum_longitudinal_cmd.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
namespace tam::ocd::communication_handlers
{
class DrivetrainWheelTorqueCommunicationHandler
: public tam::ocd::interfaces::CommunicationHandlerBase<
    tam::ocd::drivetrain::DrivetrainWheelTorqueModel::DriverInputType,
    tam::ocd::drivetrain::DrivetrainWheelTorqueModel::FeedbackType>
{
public:
  explicit DrivetrainWheelTorqueCommunicationHandler(rclcpp::Node * node);

private:
  // Node
  rclcpp::Node * node_;

  // Subscription
  static constexpr std::string_view topic_driver_input_ = "/simulation/actuation/longitudinal";
  rclcpp::Subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr sub_driver_input_{};

  // Publishing
  static constexpr std::string_view topic_fb_input_torque_ = "/simulation/sensor/input_torque_Nm";
  rclcpp::Publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr pub_fb_input_torque_{};

  DriverInputType dt_input_;
  bool has_new_input_{true};

  tam::tsl::ValueLogger::SharedPtr debug_container_ = std::make_shared<tam::tsl::ValueLogger>();

  void driver_input_callback(const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);

  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  void assign_debug_outputs();

public:
  // Checks for new driver input messages
  bool get_new_input_flag() { return has_new_input_; }
  void reset_new_input_flag() { has_new_input_ = false; }
  // Input/feedback handling
  DriverInputType get_driver_input();
  void publish_feedback(FeedbackType feedback, rclcpp::Time timestamp);
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() { return param_manager_; }
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() { return debug_container_; }
  // Delay handling (inactive)
  void update_delay_timer(std::chrono::duration<double>) {}
  void reset_input_delay() {}
};
}  // namespace tam::ocd::communication_handlers
