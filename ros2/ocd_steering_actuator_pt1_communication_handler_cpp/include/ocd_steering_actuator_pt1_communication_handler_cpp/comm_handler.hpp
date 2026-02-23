// Copyright 2025 Georg Jank
#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "ocd_communication_handler_base_cpp/base_class.hpp"
#include "ocd_helpers_cpp/time_delay.hpp"
#include "ocd_steering_actuator_pt1_cpp/steering_actuator_pt1_model.hpp"
#include "ocd_vehicle_model_node_cpp/helpers.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tum_msgs/msg/tum_float32_stamped.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "tum_msgs/msg/tum_int8_stamped.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
namespace tam::ocd::communication_handlers
{
class SteeringActuatorPT1CommunicationHandler
: public tam::ocd::interfaces::CommunicationHandlerBase<
    tam::ocd::steering_actuator::PT1SteeringActuatorModel::DriverInputType,
    tam::ocd::steering_actuator::PT1SteeringActuatorModel::FeedbackType>
{
public:
  explicit SteeringActuatorPT1CommunicationHandler(rclcpp::Node * node);

private:
  // Node
  rclcpp::Node * node_;

  // Subscription
  static constexpr std::string_view topic_driver_input_ = "/simulation/actuation/lateral";
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannLateralCommand>::SharedPtr
    sub_driver_input_{};

  // Publishing
  static constexpr std::string_view topic_fb_steering_angle_ = "/simulation/sensor/steering_report";
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    pub_fb_steering_angle_{};

  DriverInputType sa_input_;
  bool has_new_input_{true};

  tam::tsl::ValueLogger::SharedPtr debug_container_ = std::make_shared<tam::tsl::ValueLogger>();

  void driver_input_callback(
    const autoware_auto_control_msgs::msg::AckermannLateralCommand::SharedPtr msg);

  // Delay handling
  uint64_t internal_time_us_{0};
  DriverInputType sa_delayed_input_;
  tam::ocd::helpers::time_delay::TimeDelay<double> steering_angle_delay_;
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  void declare_delay_parameters();
  void assign_debug_outputs();

public:
  // Checks for new driver input messages
  bool get_new_input_flag();
  void reset_new_input_flag() { has_new_input_ = false; }
  // Input/feedback handling
  DriverInputType get_driver_input();
  void publish_feedback(FeedbackType feedback, rclcpp::Time timestamp);

  // Delay handling
  void update_delay_timer(std::chrono::duration<double> integration_step_s);
  void reset_input_delay();
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() { return param_manager_; }
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() { return debug_container_; }
};
}  // namespace tam::ocd::communication_handlers
