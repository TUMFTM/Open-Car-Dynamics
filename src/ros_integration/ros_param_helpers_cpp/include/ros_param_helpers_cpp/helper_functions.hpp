// Copyright 2023 Simon Hoffmann
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "param_manager_cpp/param_manager_base.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
namespace tam::ros
{
/// @brief Goes through all the parameters in param_manager and declares them as ROS Parameters
/// @param node_instance Handle to the Node Instance
/// @param param_manager Handle to the Param Manager
void declare_ros_params_from_param_manager(
  rclcpp::Node * node_instance, tam::interfaces::ParamManagerBase * param_manager);
/// @brief Ensures the ParamManager is updated on ROS Parameter changes
/// @param node_instance Handle to the Node Instance
/// @param param_manager Handle to the Param Manager
/// @return OnSetParametersCallbackHandle
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr connect_param_manager_to_ros_cb(
  rclcpp::Node * node_instance, tam::interfaces::ParamManagerBase * param_manager);
/// @brief Callback function that is called in case a Paramter is changed on runtime
/// @param parameters Parameters that are changing
/// @param param_manager Handle to the Param Manager
/// @return SetParametersResult
rcl_interfaces::msg::SetParametersResult param_set_callback(
  const std::vector<rclcpp::Parameter> & parameters,
  tam::interfaces::ParamManagerBase * param_manager);
/// @brief Update Parameter in Param Manager with a Ros Parameter
/// @param param_manager Parameter Manager is updated with the provided Ros Parameter
/// @param param Input Ros Parameter
/// @return Boolean if successfull
bool set_from_ros_param(
  tam::interfaces::ParamManagerBase * param_manager, const rclcpp::Parameter & param);
/// @brief Loads Parameters from the provided YAML File and overwrites existing ROS Parameters. No
/// new parameters are declared
/// @param node_instance Handle to the Node Instance
/// @param path Path to .yml file
/// @param namesp Namespace inside yaml file in which the "ros__parameters" are declared
/// @param throw_if_overwr_non_existent_param woll throw an exception if trying to set a overwrite a
/// parameter that does not exist (default=true)
void load_overwrites_from_yaml(
  rclcpp::Node * node_instance, const std::string & path, const std::string & namesp,
  bool throw_if_overwr_non_existent_param = true);
/// @brief Loads Parameters from the provided YAML File and overwrites existing Parameters inside
/// Param Manager. No new parameters are declared. Meant for Debugging the Algorithm Class without a
/// ROS-Node and still being able to test different param files
/// @param param_manager Handle to the Param Manager
/// @param path Path to .yml file
/// @param namesp Namespace inside yaml file in which the "ros__parameters" are declared
/// @param throw_if_overwr_non_existent_param woll throw an exception if trying to set a overwrite a
/// parameter that does not exist (default=false)
void load_overwrites_from_yaml(
  tam::interfaces::ParamManagerBase * param_manager, const std::string & path,
  const std::string & namesp, bool throw_if_overwr_non_existent_param = false,
  bool mandatory_overwrite = false);
/// @brief Will throw if the parameter file provided with "--params-file" does not exist, contains
/// parameters that don't exist or does not contain the desired node. Call this function after node
/// is initialzed and before it is spinned.
/// @param argc argc argument main-function
/// @param argv argv argument of main-function
/// @param node pointer to node
void validate_param_overrides(int argc, char ** argv, rclcpp::Node * node);
}  // namespace tam::ros
