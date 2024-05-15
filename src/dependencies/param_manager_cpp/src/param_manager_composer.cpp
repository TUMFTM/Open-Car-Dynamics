// Copyright 2023 Simon Sagmeister
#include "param_manager_cpp/param_manager_composer.hpp"
namespace tam::core
{
// Constructor
ParamManagerComposer::ParamManagerComposer(
  std::vector<std::shared_ptr<tam::interfaces::ParamManagerBase>> list)
: param_manager_list_(list)
{
  std::unordered_set<std::string> unique_names;
  std::unordered_map<std::string, tam::types::param::ParameterValue> unique_param_values;
  // loop through every listed manager
  for (std::shared_ptr<tam::interfaces::ParamManagerBase> manager : param_manager_list_) {
    // loop through every param name
    for (std::string name : manager->list_parameters()) {
      if (unique_names.count(name)) {
        std::cout << "[ParamComposer]: " << name.c_str()
                  << " existing in more than one param manager." << std::endl;
        if (unique_param_values.at(name).get_type() != manager->get_parameter_type(name)) {
          throw std::runtime_error(
            "[ParamComposer]: '" + name + "' parameters have different datatypes.");
        } else {
          switch (manager->get_parameter_type(name)) {
            case tam::types::param::ParameterType::BOOL:
              if (
                unique_param_values.at(name).as_bool() !=
                manager->get_parameter_value(name).as_bool()) {
                throw std::runtime_error(
                  "[ParamComposer]: '" + name + "' parameters have different values.");
              }
              break;
            case tam::types::param::ParameterType::INTEGER:
              if (
                unique_param_values.at(name).as_int() !=
                manager->get_parameter_value(name).as_int()) {
                throw std::runtime_error(
                  "[ParamComposer]: '" + name + "' parameters have different values.");
              }
              break;
            case tam::types::param::ParameterType::DOUBLE:
              if (
                unique_param_values.at(name).as_double() !=
                manager->get_parameter_value(name).as_double()) {
                std::cout << "[ParamComposer]: " << name.c_str()
                          << " parameters have different values." << std::endl;
                throw std::runtime_error(
                  "[ParamComposer]: '" + name + "' parameters have different values.");
              }
              break;

            default:
              std::cout << "[ParamComposer]: " << name
                        << " exists more than once, but has unknown datatype. " << std::endl;
              break;
          }
        }
      } else {
        unique_names.insert(name);
        tam::types::param::ParameterValue out = manager->get_parameter_value(name);
        unique_param_values.insert({name, out});
      }
    }
  }
}
// methods
bool ParamManagerComposer::has_parameter(const std::string & name) const
{
  if (param_names.count(name)) {
    return true;
  }
  for (std::shared_ptr<tam::interfaces::ParamManagerBase> manager : param_manager_list_) {
    if (manager->has_parameter(name)) {
      return true;
    }
  }
  return false;
}
std::unordered_set<std::string> ParamManagerComposer::list_parameters() const
{
  std::unordered_set<std::string> out = param_names;
  for (std::shared_ptr<tam::interfaces::ParamManagerBase> manager : param_manager_list_) {
    out.insert(manager->list_parameters().begin(), manager->list_parameters().end());
  }
  return out;
}
tam::types::param::ParameterType ParamManagerComposer::get_parameter_type(
  const std::string & name) const
{
  if (param_names.count(name)) {
    return param_values.at(name).get_type();
  }
  for (std::shared_ptr<tam::interfaces::ParamManagerBase> manager : param_manager_list_) {
    if (manager->has_parameter(name)) {
      return manager->get_parameter_type(name);
    }
  }
  tam::types::param::ParameterType out{tam::types::param::ParameterType::STRING};
  return out;
}
std::string ParamManagerComposer::get_parameter_description(const std::string & name) const
{
  if (param_names.count(name)) {
    return param_descriptions.at(name);
  }
  for (std::shared_ptr<tam::interfaces::ParamManagerBase> manager : param_manager_list_) {
    if (manager->has_parameter(name)) {
      manager->get_parameter_description(name);
    }
  }
  std::string out{"parameter does not exist"};
  return out;
}
void ParamManagerComposer::set_parameter_value(
  const std::string & name, const tam::types::param::param_variant_t & value)
{
  if (param_names.count(name)) {
    param_values.at(name).set_parameter_value(value);
  }
  for (std::shared_ptr<tam::interfaces::ParamManagerBase> manager : param_manager_list_) {
    if (manager->has_parameter(name)) {
      manager->set_parameter_value(name, value);
    }
  }
}
tam::types::param::ParameterValue ParamManagerComposer::get_parameter_value(
  const std::string & name) const
{
  if (param_names.count(name)) {
    return param_values.at(name);
  }
  for (std::shared_ptr<tam::interfaces::ParamManagerBase> manager : param_manager_list_) {
    if (manager->has_parameter(name)) {
      return manager->get_parameter_value(name);
    }
  }
  tam::types::param::ParameterValue out{tam::types::param::ParameterType::DOUBLE, 0.0};
  return out;
}
void ParamManagerComposer::declare_parameter(
  const std::string & name, const tam::types::param::param_variant_t & default_value,
  const tam::types::param::ParameterType & param_type, const std::string & description)
{
  param_names.insert(name);
  param_descriptions.insert({name, description});
  param_values.insert({name, tam::types::param::ParameterValue(param_type, default_value)});
}
}  // namespace tam::core