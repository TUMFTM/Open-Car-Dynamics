// Copyright 2026 Simon Sagmeister
#pragma once
#include <tf2/LinearMath/Quaternion.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ocd_communication_handler_base_cpp/concept.hpp"
#include "ocd_vehicle_model_base_cpp/concept.hpp"
#include "param_management_cpp/base.hpp"
#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "param_management_ros2_integration_cpp/helper_functions.hpp"
#include "tsl_logger_cpp/composer.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"
#include "tum_msgs/msg/tum_external_vehicle_influences.hpp"
#include "tum_msgs/msg/tum_float32_stamped.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
/// @brief VehicleModelNode class implementing a vehicle model node
namespace tam::ocd
{
template <
  interfaces::concepts::VehicleModel VEHICLE_T,
  interfaces::concepts::CommunicationHandler DT_COMM_HANDLER_T,
  interfaces::concepts::CommunicationHandler SA_COMM_HANDLER_T>
class VehicleModelNode : public rclcpp::Node
{
private:
  // clang-format off
  static constexpr std::string_view node_name_ = "VehicleModel";
  static constexpr std::string_view namespace_ = "/simulation";
  // Subscription
  static constexpr std::string_view topic_sub_external_influences_ ="/simulation/road_plane/external_influences"; // NOLINT
  // Publishing
  static constexpr std::string_view topic_pub_odometry_ = "/simulation/road_plane/odometry";
  static constexpr std::string_view topic_pub_accel_ = "/simulation/road_plane/acceleration";
  static constexpr std::string_view topic_pub_wheelspeeds_ = "/simulation/sensor/wheelspeed_radps";
  static constexpr std::string_view topic_pub_tire_force_long_ ="/simulation/sensor/tire_force_long"; // NOLINT
  static constexpr std::string_view topic_pub_tire_force_lat_ = "/simulation/sensor/tire_force_lat";
  static constexpr std::string_view topic_pub_tire_force_vert_ ="/simulation/sensor/tire_force_vert"; // NOLINT
  static constexpr std::string_view topic_pub_long_slip_ = "/simulation/sensor/long_slip";
  static constexpr std::string_view topic_pub_slip_angle_ = "/simulation/sensor/slip_angle";
  // clang-format on

public:
  /// Constructor
  explicit VehicleModelNode(
    std::unique_ptr<VEHICLE_T> && veh_model, const rclcpp::NodeOptions & options);

  /// @brief  Reset the model to initial state
  void reset();

private:
  bool initial_cycle_{true};
  bool high_frequency_logging_{false};
  std::unique_ptr<VEHICLE_T> veh_model_{};

  /// Generic communication handlers for abstracting the input/output types of the drivetrain and
  /// the steering actuator.
  std::shared_ptr<DT_COMM_HANDLER_T> dt_comm_handler_{};
  std::shared_ptr<SA_COMM_HANDLER_T> sa_comm_handler_{};

  OnSetParametersCallbackHandle::SharedPtr callback_handle_{};
  rclcpp::TimerBase::SharedPtr model_update_timer_{};
  rclcpp::TimerBase::SharedPtr output_timer_{};
  tam::pmg::ParamManagerComposer::SharedPtr param_manager_composer_{};
  tam::tsl::ValueLogger::SharedPtr logger_{};
  tam::tsl::LoggerComposer::SharedPtr logger_composer_{};
  types::ExternalInfluences external_input_{};
  std::chrono::duration<double> timer_period_s_{};
  bool has_new_input_{true};
  bool has_new_external_influence_{true};

  // Subscriptions
  // clang-format off
  rclcpp::Subscription<tum_msgs::msg::TUMExternalVehicleInfluences>::SharedPtr sub_external_influences_{}; // NOLINT
  // clang-format on

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_{};
  rclcpp::Publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr wheel_speed_pub_{};
  rclcpp::Publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr tire_force_long_pub_{};
  rclcpp::Publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr tire_force_lat_pub_{};
  rclcpp::Publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr tire_force_vert_pub_{};
  rclcpp::Publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr long_slip_pub_{};
  rclcpp::Publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr slip_angle_pub_{};

  // TSL Publisher
  tam::tsl::TSLPublisher::UniquePtr tsl_pub_{};

  /// ============ Callback and update functions ============
  /// @brief Step the vehicle model
  void model_update_callback();

  /// @brief Publish all outputs of the vehicle model at the current time step
  void output_callback();

  // Subscription callbacks
  /// @brief Callback for receiving external influences
  void external_influences_callback(
    const tum_msgs::msg::TUMExternalVehicleInfluences::SharedPtr msg);

  // Update functions
  void update_model_inputs();
  void update_external_influences();
};
}  // namespace tam::ocd
// Include the implementation file
#include "ocd_vehicle_model_node_cpp/vehicle_model_node_impl.hpp"
