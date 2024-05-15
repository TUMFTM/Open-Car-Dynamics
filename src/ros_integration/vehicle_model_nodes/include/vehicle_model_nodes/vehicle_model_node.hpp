// Copyright 2023 Simon Hoffmann
#pragma once
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "ros_debug_helpers_cpp/debug_publisher.hpp"
#include "ros_param_helpers_cpp/helper_functions.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tum_msgs/msg/tum_debug_signal_names.hpp"
#include "tum_msgs/msg/tum_debug_values.hpp"
#include "tum_msgs/msg/tum_external_vehicle_influences.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "vehicle_model_base_cpp/vehicle_model_base.h"
#include "vehicle_model_nodes/helpers.hpp"
class VehicleModelNode : public rclcpp::Node
{
private:
  static constexpr std::string_view node_name_ = "VehicleModel";
  static constexpr std::string_view namespace_ = "/simulation";
  // Subscription
  static constexpr std::string_view topic_sub_ctrl_cmd_ = "/simulation/actuation/control_command";
  static constexpr std::string_view topic_sub_external_influences_ =
    "/simulation/external_influences";
  // Publishing
  static constexpr std::string_view topic_pub_odometry_ = "/simulation/sensor/odometry";
  static constexpr std::string_view topic_pub_accel_ = "/simulation/sensor/acceleration";
  static constexpr std::string_view topic_pub_wheelspeeds_ = "/simulation/sensor/wheelspeed_radps";
  static constexpr std::string_view topic_pub_steering_ = "/simulation/sensor/steering_report";

public:
  explicit VehicleModelNode(std::unique_ptr<tam::interfaces::VehicleModelBase> && dyn_model);

  void reset();

private:
  bool initial_cycle_{true};
  std::unique_ptr<tam::interfaces::VehicleModelBase> veh_model_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rclcpp::TimerBase::SharedPtr model_update_timer_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  tam::types::DriverInput model_input_;
  tam::types::ExternalInfluences external_input_;
  std::chrono::duration<double> timer_period_s_;
  bool has_new_input{true};
  bool has_new_external_influence{true};

  // Timer callbacks
  void model_update_callback();

  void output_callback();
  void ctrl_cmd_callback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
  void external_influences_callback(
    const tum_msgs::msg::TUMExternalVehicleInfluences::SharedPtr msg);

  void update_model_inputs();
  void update_external_influences();

  // Subscriptions
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_ctrl_cmd_;
  rclcpp::Subscription<tum_msgs::msg::TUMExternalVehicleInfluences>::SharedPtr
    sub_external_influences_;

  // Todo: Subscribe external influences
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr wheel_speed_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_angle_pub_;

  // Debug publisher
  tam::ros::DebugPublisher debug_pub_{this};
};
