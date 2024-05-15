// Copyright 2023 Simon Sagmeister
#include "ros_debug_helpers_cpp/debug_publisher.hpp"

#include <rclcpp/serialization.hpp>
namespace tam::ros
{
void DebugPublisher::publish_signal_names()
{
  _signal_name_publisher->publish(_rcl_serialized_signal_name_message);
  _signal_names_last_published = _node_handle->get_clock()->now();
}
std::string DebugPublisher::get_validated_channel_name(const std::string & debug_channel_name)
{
  auto validated_channel_name =
    tam::types::common::TUMDebugContainer::validate_signal_name(debug_channel_name);
  if (validated_channel_name.empty()) {
    throw std::invalid_argument(
      std::string("Given channel name is not valid: ") + debug_channel_name);
  }
  validated_channel_name.insert(0, 1, '/');
  return validated_channel_name;
};
void DebugPublisher::connect_to_node(rclcpp::Node * node_handle)
{
  if (_node_handle) {
    RCLCPP_ERROR(
      _node_handle->get_logger(), "ERROR: The debug publisher is already connnected to a node");
    return;
  }
  _node_handle = node_handle;
  rclcpp::PublisherOptions pub_opts;
  // Disable intraprocess comm, otherwise it is not possible to publish the serialized message.
  pub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  _signal_name_publisher = node_handle->create_publisher<tum_msgs::msg::TUMDebugSignalNames>(
    std::string("/debug") + node_handle->get_fully_qualified_name() + _channel_name +
      "/signal_names",
    qos_settings::DEBUG_SIGNAL_NAMES, pub_opts);
  _value_publisher = node_handle->create_publisher<tum_msgs::msg::TUMDebugValues>(
    std::string("/debug") + node_handle->get_fully_qualified_name() + _channel_name + "/values",
    qos_settings::DEBUG_VALUES);
}
void DebugPublisher::publish_debug_outputs(tam::types::common::TUMDebugContainer * debug_container)
{
  if (!_node_handle) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DebugPublisherFallbackLogger"),
      "ERROR: The debug publisher is not connnected to a node. Call the "
      "DebugPublisher::connect_to_node function beforehand");
    return;
  }
  if (!_initialized) {
    _initialized = true;
    _config_hash = debug_container->get_config_hash();
    // Create and serialize the debug message once to not have to serialize the
    // vector of strings every time since we saw quite some performance
    // hit from the serialization
    tum_msgs::msg::TUMDebugSignalNames signal_names_msg;
    signal_names_msg.names = debug_container->get_signal_names();
    rclcpp::Serialization<tum_msgs::msg::TUMDebugSignalNames> serializer;
    serializer.serialize_message(&signal_names_msg, &_serialized_signal_name_message);
    _rcl_serialized_signal_name_message =
      _serialized_signal_name_message.get_rcl_serialized_message();
    publish_signal_names();
  }
  if (_config_hash != debug_container->get_config_hash()) {
    // Do nothing if the config hash differs from the first published container
    RCLCPP_ERROR(
      _node_handle->get_logger(),
      "ERROR publishing debug outputs. The config hash of the container changed");
    return;
  }
  // Update the messages
  _values_msg.values = debug_container->get_values();

  // Do not publish values any more
  if (
    (_node_handle->get_clock()->now() - _signal_names_last_published).seconds() >=
    1.0 / _signal_name_pub_frequency) {
    publish_signal_names();
  }

  // Publish the value message
  _value_publisher->publish(_values_msg);
}
}  // namespace tam::ros
