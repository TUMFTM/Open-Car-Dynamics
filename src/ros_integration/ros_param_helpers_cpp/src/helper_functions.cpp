// Copyright Simon Hoffmann
#include "ros_param_helpers_cpp/helper_functions.hpp"

#include <filesystem>
namespace tam::ros
{
void load_overwrites_from_yaml(
  rclcpp::Node * node_instance, const std::string & path, const std::string & namesp,
  bool throw_if_overwr_non_existent_param)
{
  rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file(path);
  std::vector<rclcpp::Parameter> params = map[namesp];
  for (const auto & param : params) {
    if (!node_instance->has_parameter(param.get_name()) && throw_if_overwr_non_existent_param) {
      throw rclcpp::exceptions::ParameterNotDeclaredException(
        "Trying to overwrite non-existent Parameter " + param.get_name());
    }
    node_instance->set_parameter(param);
  }
}
void load_overwrites_from_yaml(
  tam::interfaces::ParamManagerBase * param_manager, const std::string & path,
  const std::string & namesp, bool throw_if_overwr_non_existent_param, bool mandatory_overwrite)
{
  rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file(path);
  std::vector<rclcpp::Parameter> params = map[namesp];
  if (mandatory_overwrite) {
    // Go through param_manager and throw if param not in map
    for (const auto & param : param_manager->list_parameters()) {
      if (!std::any_of(params.begin(), params.end(), [param](const rclcpp::Parameter & i) {
            return i.get_name() == param;
          })) {
        throw rclcpp::exceptions::ParameterNotDeclaredException(
          "The following parameter is not overwritten in yaml file (" + path + "): " + param +
          " (This is mandatory!)");
      }
    }
  }

  for (const auto & param : params) {
    if (!param_manager->has_parameter(param.get_name())) {
      if (throw_if_overwr_non_existent_param) {
        throw rclcpp::exceptions::ParameterNotDeclaredException(
          "Trying to overwrite non-existent Parameter " + param.get_name());
      } else {
        continue;
      }
    }
    set_from_ros_param(param_manager, param);
  }
}
void declare_ros_params_from_param_manager(
  rclcpp::Node * node_instance, tam::interfaces::ParamManagerBase * param_manager)
{
  for (std::string name : param_manager->list_parameters()) {
    switch (param_manager->get_parameter_type(name)) {
      case tam::types::param::ParameterType::BOOL:
        node_instance->declare_parameter(name, param_manager->get_parameter_value(name).as_bool());
        break;
      case tam::types::param::ParameterType::BOOL_ARRAY:
        node_instance->declare_parameter(
          name, param_manager->get_parameter_value(name).as_bool_array());
        break;
      case tam::types::param::ParameterType::INTEGER:
        node_instance->declare_parameter(name, param_manager->get_parameter_value(name).as_int());
        break;
      case tam::types::param::ParameterType::DOUBLE:
        node_instance->declare_parameter(
          name, param_manager->get_parameter_value(name).as_double());
        break;
      case tam::types::param::ParameterType::DOUBLE_ARRAY:
        node_instance->declare_parameter(
          name, param_manager->get_parameter_value(name).as_double_array());
        break;
      case tam::types::param::ParameterType::STRING:
        node_instance->declare_parameter(
          name, param_manager->get_parameter_value(name).as_string());
        break;
      case tam::types::param::ParameterType::STRING_ARRAY:
        node_instance->declare_parameter(
          name, param_manager->get_parameter_value(name).as_string_array());
        break;
      default:
        RCLCPP_ERROR_STREAM(
          node_instance->get_logger(), name << " could not be declared as ros parameter.");
        break;
    }
  }
}
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr connect_param_manager_to_ros_cb(
  rclcpp::Node * node_instance, tam::interfaces::ParamManagerBase * param_manager)
{
  return node_instance->add_on_set_parameters_callback(
    std::bind(&param_set_callback, std::placeholders::_1, param_manager));
}
rcl_interfaces::msg::SetParametersResult param_set_callback(
  const std::vector<rclcpp::Parameter> & parameters,
  tam::interfaces::ParamManagerBase * param_manager)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto & param : parameters) {
    if ((param_manager->has_parameter(param.get_name()) &&
         !set_from_ros_param(param_manager, param))) {
      result.successful = false;
      result.reason = "Could not set at least one parameter";
      return result;
    }
  }
  result.successful = true;
  result.reason = "Successfully set parameter values";
  return result;
}
bool set_from_ros_param(
  tam::interfaces::ParamManagerBase * param_manager, const rclcpp::Parameter & param)
{
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      param_manager->set_parameter_value(param.get_name(), param.as_bool());
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      param_manager->set_parameter_value(param.get_name(), param.as_bool_array());
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      param_manager->set_parameter_value(param.get_name(), static_cast<int>(param.as_int()));
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      param_manager->set_parameter_value(param.get_name(), param.as_double());
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      param_manager->set_parameter_value(param.get_name(), param.as_double_array());
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      param_manager->set_parameter_value(param.get_name(), param.as_string());
      break;
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      param_manager->set_parameter_value(param.get_name(), param.as_string_array());
      break;
    default:
      return false;
      break;
  }
  return true;
}
void validate_param_overrides(int argc, char ** argv, rclcpp::Node * node)
{
  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::string param_file{""};
  std::string node_name = node->get_fully_qualified_name();
  // Get param_file path from argv
  for (std::size_t i = 0; i < arguments.size(); i++) {
    if (arguments.at(i) == "--params-file") {
      param_file = arguments.at(i + 1);
    }
  }
  if (param_file == "") {
    return;
  }
  // Check if absolute path and convert if relative path
  if (param_file.rfind("/", 0) != 0) {
    param_file = std::filesystem::current_path().string() + std::string("/") + param_file;
  }
  // check if Param file exists and throw
  if (!std::filesystem::exists(param_file)) {
    throw std::invalid_argument(
      node_name + ": You were trying to set a non-existing parameter file: " + param_file);
  }

  // Load params from param file
  auto param_map = rclcpp::parameter_map_from_yaml_file(param_file);

  if (!param_map.count(node_name)) {  // Key not in map
    throw std::invalid_argument(
      node_name + ": Key (fully qualified node name) not part of param-file: " + node_name);
  }

  auto parameters = param_map[node_name];

  // Verify parameters
  for (const auto & param : parameters) {
    if (!node->has_parameter(param.get_name())) {
      throw std::invalid_argument(
        node_name + ": You were trying to set a non-existing parameter: " + param.get_name());
    }
  }
}
}  // namespace tam::ros
