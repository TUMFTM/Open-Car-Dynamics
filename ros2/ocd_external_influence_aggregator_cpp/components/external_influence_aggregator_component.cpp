// Copyright 2026 Simon Sagmeister
#include <rclcpp_components/register_node_macro.hpp>

#include "ocd_external_influence_aggregator_cpp/external_influence_aggregator_node.hpp"
// The node needs no extra construction logic, so it doubles as its own
// component: rclcpp_components only requires a NodeOptions constructor.
RCLCPP_COMPONENTS_REGISTER_NODE(tam::ocd::ExternalInfluenceAggregatorNode)
