// Copyright 2026 Simon Sagmeister
#include "ocd_external_influence_aggregator_cpp/external_influence_aggregator_node.hpp"

#include <chrono>

#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_ros_helpers_cpp/timer.hpp"
namespace tam::ocd
{
namespace
{
// Build an identity element with every component set to `v` (0 for add, 1 for multiply).
geometry_msgs::msg::Wrench wrench_filled(double v)
{
  geometry_msgs::msg::Wrench out;
  out.force.x = out.force.y = out.force.z = v;
  out.torque.x = out.torque.y = out.torque.z = v;
  return out;
}
tum_msgs::msg::TUMFloat64PerWheel per_wheel_filled(double v)
{
  tum_msgs::msg::TUMFloat64PerWheel out;
  out.front_left = out.front_right = out.rear_left = out.rear_right = v;
  return out;
}
}  // namespace
CombineOp ExternalInfluenceAggregatorNode::parse_combine_op(
  std::string const & op, rclcpp::Logger const & logger)
{
  if (op == "multiply") return CombineOp::multiply;
  if (op != "add") {
    RCLCPP_WARN(
      logger, "Unknown combine op '%s', falling back to 'add'. Use 'add' or 'multiply'.",
      op.c_str());
  }
  return CombineOp::add;
}
ExternalInfluenceAggregatorNode::ExternalInfluenceAggregatorNode(
  rclcpp::NodeOptions const & options)
: Node(std::string(node_name_), std::string(ros_namespace_), options)
{
  auto const wrench_topics = this->declare_parameter<std::vector<std::string>>(
    "wrench_source_topics", std::vector<std::string>{});
  auto const grip_topics = this->declare_parameter<std::vector<std::string>>(
    "grip_source_topics", std::vector<std::string>{});
  auto const z_height_topics = this->declare_parameter<std::vector<std::string>>(
    "z_height_source_topics", std::vector<std::string>{});
  auto const wrench_op = parse_combine_op(
    this->declare_parameter<std::string>("wrench_combine_op", "add"), this->get_logger());
  auto const grip_op = parse_combine_op(
    this->declare_parameter<std::string>("grip_combine_op", "multiply"), this->get_logger());
  auto const z_height_op = parse_combine_op(
    this->declare_parameter<std::string>("z_height_combine_op", "add"), this->get_logger());
  auto const publish_period_ms = this->declare_parameter<int>("publish_period_ms", 10);
  auto const output_topic = this->declare_parameter<std::string>(
    "output_topic", "/simulation/road_plane/external_influences");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "vehicle_cg_footprint");

  auto qos = tam::ros::get_qos(tam::ros::TopicType::DEFAULT);

  // Force/torque contributions add up; grip scales multiply; road heights add.
  // The identity element (0 for add, 1 for multiply) neutralises un-published
  // sources for the chosen op.
  using WrenchChannel =
    InfluenceChannel<geometry_msgs::msg::WrenchStamped, geometry_msgs::msg::Wrench>;
  using PerWheelChannel =
    InfluenceChannel<tum_msgs::msg::TUMFloat64PerWheelStamped, tum_msgs::msg::TUMFloat64PerWheel>;
  auto const to_data = [](tum_msgs::msg::TUMFloat64PerWheelStamped const & m) { return m.data; };

  wrench_channel_ = std::make_unique<WrenchChannel>(
    this, qos, wrench_topics, wrench_op,
    wrench_filled(wrench_op == CombineOp::multiply ? 1.0 : 0.0),
    [](geometry_msgs::msg::WrenchStamped const & m) { return m.wrench; });
  grip_channel_ = std::make_unique<PerWheelChannel>(
    this, qos, grip_topics, grip_op, per_wheel_filled(grip_op == CombineOp::multiply ? 1.0 : 0.0),
    to_data);
  z_height_channel_ = std::make_unique<PerWheelChannel>(
    this, qos, z_height_topics, z_height_op,
    per_wheel_filled(z_height_op == CombineOp::multiply ? 1.0 : 0.0), to_data);

  pub_ = this->create_publisher<ocd_interfaces::msg::ExternalInfluences>(output_topic, qos);

  timer_ = tam::create_timer(
    this, std::chrono::milliseconds(publish_period_ms),
    std::bind(&ExternalInfluenceAggregatorNode::publish, this));
}
void ExternalInfluenceAggregatorNode::publish()
{
  ocd_interfaces::msg::ExternalInfluences msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = frame_id_;
  msg.wrench = wrench_channel_->combined();
  msg.lambda_mue = grip_channel_->combined();
  msg.z_height_road_m = z_height_channel_->combined();

  pub_->publish(std::move(msg));
}
}  // namespace tam::ocd
