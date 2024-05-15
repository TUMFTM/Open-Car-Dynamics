// Copyright 2023 Simon Sagmeister
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tum_msgs/msg/tum_debug_signal_names.hpp>
#include <tum_msgs/msg/tum_debug_values.hpp>
#include <tum_types_cpp/common.hpp>
namespace tam::ros
{
namespace qos_settings
{
static rclcpp::QoS const DEBUG_SIGNAL_NAMES = rclcpp::QoS(1);
static rclcpp::QoS const DEBUG_VALUES = rclcpp::QoS(1);
}  // namespace qos_settings
class DebugPublisher
{
private:
  static constexpr double _signal_name_pub_frequency = 0.5;
  bool _initialized = false;
  std::string _channel_name = "";
  std::size_t _config_hash;
  rclcpp::Time _signal_names_last_published;
  rclcpp::Node * _node_handle{};
  std::shared_ptr<rclcpp::Publisher<tum_msgs::msg::TUMDebugSignalNames>> _signal_name_publisher{};
  std::shared_ptr<rclcpp::Publisher<tum_msgs::msg::TUMDebugValues>> _value_publisher{};
  rcl_serialized_message_t _rcl_serialized_signal_name_message;
  // We have to store this as well, otherwise the destrctor of this object cleans up the above
  // object
  rclcpp::SerializedMessage _serialized_signal_name_message;
  tum_msgs::msg::TUMDebugValues _values_msg;
  // Helper functions
  static std::string get_validated_channel_name(const std::string & debug_channel_name);
  void publish_signal_names();
  void connect_to_node(rclcpp::Node * node_handle);

public:
  /// @brief Constructs a debug publisher for a non default channel. This is especially useful if a
  /// node wants to publish multiple debug messages asynchronously (e.g. on subscription) of another
  /// message. The topics published by this publisher will be:
  /// - /debug/${fully_qualified_node_name}/${debug_channel_name}/signal_names
  /// - /debug/${fully_qualified_node_name}/${debug_channel_name}/values
  /// @param node_handle
  explicit DebugPublisher(rclcpp::Node * node_handle) { connect_to_node(node_handle); }
  /// @brief Constructs a debug publisher for a non default channel. This is especially useful if a
  /// node wants to publish multiple debug messages asynchronously (e.g. on subscription) of another
  /// message. The topics published by this publisher will be:
  /// - /debug/${fully_qualified_node_name}/${debug_channel_name}/signal_names
  /// - /debug/${fully_qualified_node_name}/${debug_channel_name}/values
  /// @param node_handle
  /// @param debug_channel_name
  explicit DebugPublisher(rclcpp::Node * node_handle, const std::string & debug_channel_name)
  : _channel_name(get_validated_channel_name(debug_channel_name))
  {
    connect_to_node(node_handle);
  }
  void publish_debug_outputs(tam::types::common::TUMDebugContainer * debug_container);
  // Stuff below this line is just here for backwards compatibility
  // ======================================================================

  /// DEPRECATED: DO NOT USE ANY MORE
  void connect_to_node(std::shared_ptr<rclcpp::Node> node_handle)
  {
    RCLCPP_WARN(
      node_handle->get_logger(),
      "DEPRECATION WARNING: DebugPubliser | Do not explicitly use the connect_to_node function any "
      "more. This syntax will be removed. The new version is to directly pass the node handle in "
      "the constructor of the debug publisher");
    connect_to_node(node_handle.get());
  };
  /// DEPRECATED: DO NOT USE ANY MORE
  DebugPublisher() = default;
  /// DEPRECATED: DO NOT USE ANY MORE
  explicit DebugPublisher(const std::string & debug_channel_name)
  : _channel_name(get_validated_channel_name(debug_channel_name))
  {
  }
};
}  // namespace tam::ros
