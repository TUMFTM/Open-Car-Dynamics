// Copyright 2026 Simon Sagmeister
#pragma once
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "ocd_aerodynamics_models_cpp/default.hpp"
#include "ocd_aerodynamics_models_cpp/ride_height.hpp"
#include "ocd_drivetrain_fx_communication_handler_cpp/comm_handler.hpp"
#include "ocd_drivetrain_wheel_torque_communication_handler_cpp/comm_handler.hpp"
// #include "ocd_drivetrain_fx_cpp/drivetrain_fx_model.hpp"
#include "ocd_steering_actuator_pt1_communication_handler_cpp/comm_handler.hpp"
#include "ocd_steering_actuator_pt1_cpp/steering_actuator_pt1_model.hpp"
#include "ocd_tire_models_cpp/linear.hpp"
#include "ocd_tire_models_cpp/mf_52.hpp"
#include "ocd_tire_models_cpp/mf_simple.hpp"
#include "ocd_tire_models_cpp/mf_simple_extended.hpp"
#include "ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_model.hpp"
#include "ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_model.hpp"
#include "ocd_vehicle_model_cpp/vehicle_model.hpp"
#include "ocd_vehicle_model_node_cpp/vehicle_model_node.hpp"

#define OCD_NODE_COMPONENT(                                                                        \
  ClassName, DT_T, DT_COMM_HANDLER_T, SA_T, SA_COMM_HANDLER_T, VD_T, AERO_T, TIRE_T)               \
  namespace ocd_vehicle_nodes_cpp                                                                  \
  {                                                                                                \
  class ClassName : public tam::ocd::VehicleModelNode<                                             \
                      tam::ocd::VehicleModel<DT_T, SA_T, VD_T<TIRE_T, AERO_T>>, DT_COMM_HANDLER_T, \
                      SA_COMM_HANDLER_T>                                                           \
  {                                                                                                \
  public:                                                                                          \
    explicit ClassName(const rclcpp::NodeOptions & options)                                        \
    : tam::ocd::VehicleModelNode<                                                                  \
        tam::ocd::VehicleModel<DT_T, SA_T, VD_T<TIRE_T, AERO_T>>, DT_COMM_HANDLER_T,               \
        SA_COMM_HANDLER_T>(                                                                        \
        std::make_unique<tam::ocd::VehicleModel<DT_T, SA_T, VD_T<TIRE_T, AERO_T>>>(), options)     \
    {                                                                                              \
      this->reset();                                                                               \
    }                                                                                              \
  };                                                                                               \
  RCLCPP_COMPONENTS_REGISTER_NODE(ocd_vehicle_nodes_cpp::ClassName);                               \
  }  // namespace ocd_vehicle_nodes_cpp
