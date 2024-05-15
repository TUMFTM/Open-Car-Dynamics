// Copyright 2023 Simon Sagmeister
// #include <fenv.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "vehicle_model_double_track_cpp/double_track_model.hpp"
#include "vehicle_model_nodes/vehicle_model_node.hpp"
int main(int argc, char ** argv)
{
  // feenableexcept(FE_ALL_EXCEPT & ~FE_INEXACT);
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<VehicleModelNode>(std::make_unique<tam::sim::VehicleModelDoubleTrack>());
  // Apply all the parameters after they have been loaded via a param file
  node->reset();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
