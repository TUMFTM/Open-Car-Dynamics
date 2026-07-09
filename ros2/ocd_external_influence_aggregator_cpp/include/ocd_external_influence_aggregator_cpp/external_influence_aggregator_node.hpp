// Copyright 2026 Simon Sagmeister

#pragma once

#include <functional>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ocd_interfaces/msg/external_influences.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
namespace tam::ocd
{
/// How the contributions of a single source list are folded together.
enum class CombineOp { add, multiply };
/// Component-wise operators on \c geometry_msgs/Wrench so the wrench channel can
/// fold force/torque contributions with the same generic reducer used for the
/// per-wheel channels. Defined in this namespace (not in \c geometry_msgs) and
/// only used internally by the aggregator.
inline geometry_msgs::msg::Wrench operator+(
  geometry_msgs::msg::Wrench const & a, geometry_msgs::msg::Wrench const & b)
{
  geometry_msgs::msg::Wrench out;
  out.force.x = a.force.x + b.force.x;
  out.force.y = a.force.y + b.force.y;
  out.force.z = a.force.z + b.force.z;
  out.torque.x = a.torque.x + b.torque.x;
  out.torque.y = a.torque.y + b.torque.y;
  out.torque.z = a.torque.z + b.torque.z;
  return out;
}
inline geometry_msgs::msg::Wrench operator*(
  geometry_msgs::msg::Wrench const & a, geometry_msgs::msg::Wrench const & b)
{
  geometry_msgs::msg::Wrench out;
  out.force.x = a.force.x * b.force.x;
  out.force.y = a.force.y * b.force.y;
  out.force.z = a.force.z * b.force.z;
  out.torque.x = a.torque.x * b.torque.x;
  out.torque.y = a.torque.y * b.torque.y;
  out.torque.z = a.torque.z * b.torque.z;
  return out;
}
/// Component-wise operators on \c tum_msgs/TUMFloat64PerWheel so the grip and
/// road-height channels can be folded with the same generic reducer.
inline tum_msgs::msg::TUMFloat64PerWheel operator+(
  tum_msgs::msg::TUMFloat64PerWheel const & a, tum_msgs::msg::TUMFloat64PerWheel const & b)
{
  tum_msgs::msg::TUMFloat64PerWheel out;
  out.front_left = a.front_left + b.front_left;
  out.front_right = a.front_right + b.front_right;
  out.rear_left = a.rear_left + b.rear_left;
  out.rear_right = a.rear_right + b.rear_right;
  return out;
}
inline tum_msgs::msg::TUMFloat64PerWheel operator*(
  tum_msgs::msg::TUMFloat64PerWheel const & a, tum_msgs::msg::TUMFloat64PerWheel const & b)
{
  tum_msgs::msg::TUMFloat64PerWheel out;
  out.front_left = a.front_left * b.front_left;
  out.front_right = a.front_right * b.front_right;
  out.rear_left = a.rear_left * b.rear_left;
  out.rear_right = a.rear_right * b.rear_right;
  return out;
}
/// A single typed external-influence channel: subscribes to a parametric list
/// of source topics of message type \c MsgT, caches the latest converted value
/// per source and folds them into one \c ValueT via a combine op. Sources that
/// have not published yet contribute the (op-dependent) identity element, so an
/// empty or partially populated list is neutral.
template <typename MsgT, typename ValueT>
class InfluenceChannel
{
public:
  using Converter = std::function<ValueT(MsgT const &)>;
  InfluenceChannel(
    rclcpp::Node * node, rclcpp::QoS const & qos, std::vector<std::string> const & topics,
    CombineOp op, ValueT identity, Converter convert)
  : identity_(identity), op_(op), latest_(topics.size(), identity)
  {
    subscriptions_.reserve(topics.size());
    for (std::size_t i = 0; i < topics.size(); ++i) {
      subscriptions_.push_back(node->create_subscription<MsgT>(
        topics[i], qos,
        [this, i, convert](typename MsgT::SharedPtr msg) { latest_[i] = convert(*msg); }));
    }
  }
  /// Fold all cached source values into the combined contribution.
  ValueT combined() const
  {
    ValueT acc = identity_;
    for (auto const & value : latest_) {
      acc = (op_ == CombineOp::multiply) ? acc * value : acc + value;
    }
    return acc;
  }

private:
  ValueT identity_;
  CombineOp op_;
  std::vector<ValueT> latest_;
  std::vector<typename rclcpp::Subscription<MsgT>::SharedPtr> subscriptions_;
};
/// Aggregates any number of external-influence sources into a single
/// \c ocd_interfaces/ExternalInfluences message. Force/torque, grip and road
/// height are configured as independent lists of source topics (see the ROS
/// parameters below); each list is folded with its own combine op and the
/// result is republished on a fixed-rate timer. Adding a new influence source
/// only requires adding its topic to the corresponding parameter list.
class ExternalInfluenceAggregatorNode : public rclcpp::Node
{
public:
  explicit ExternalInfluenceAggregatorNode(rclcpp::NodeOptions const & options);

private:
  static constexpr std::string_view node_name_ = "ExternalInfluenceAggregator";
  static constexpr std::string_view ros_namespace_ = "/simulation";

  void publish();

  static CombineOp parse_combine_op(std::string const & op, rclcpp::Logger const & logger);

  std::string frame_id_;
  rclcpp::Publisher<ocd_interfaces::msg::ExternalInfluences>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<InfluenceChannel<geometry_msgs::msg::WrenchStamped, geometry_msgs::msg::Wrench>>
    wrench_channel_;
  std::unique_ptr<
    InfluenceChannel<tum_msgs::msg::TUMFloat64PerWheelStamped, tum_msgs::msg::TUMFloat64PerWheel>>
    grip_channel_;
  std::unique_ptr<
    InfluenceChannel<tum_msgs::msg::TUMFloat64PerWheelStamped, tum_msgs::msg::TUMFloat64PerWheel>>
    z_height_channel_;
};
}  // namespace tam::ocd
