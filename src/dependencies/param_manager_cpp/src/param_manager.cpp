// Copyright 2023 Simon Sagmeister
#include "param_manager_cpp/param_manager.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
namespace tam::core
{
// Function definitions
bool ParamManager::has_parameter(const std::string & name) const
{
  if (param_names.count(name)) {
    return true;
  } else {
    return false;
  }
}
std::unordered_set<std::string> ParamManager::list_parameters() const { return param_names; }
tam::types::param::ParameterType ParamManager::get_parameter_type(const std::string & name) const
{
  try {
    return param_values.at(name).get_type();
  } catch (const std::out_of_range & oor) {
    throw std::out_of_range(
      std::string("ParamManager: Tried accessing non declared parameter: ") + name);
  }
}
std::string ParamManager::get_parameter_description(const std::string & name) const
{
  try {
    return param_descriptions.at(name);
  } catch (const std::out_of_range & oor) {
    throw std::out_of_range(
      std::string("ParamManager: Tried accessing non declared parameter: ") + name);
  }
}
void ParamManager::set_parameter_value(
  const std::string & name, const tam::types::param::param_variant_t & value)
{
  try {
    param_values.at(name).set_parameter_value(value);
  } catch (const std::out_of_range & oor) {
    throw std::out_of_range(
      std::string("ParamManager: Tried setting non declared parameter: ") + name);
  }
}
tam::types::param::ParameterValue ParamManager::get_parameter_value(const std::string & name) const
{
  try {
    return param_values.at(name);
  } catch (const std::out_of_range & oor) {
    throw std::out_of_range(
      std::string("ParamManager: Tried accessing non declared parameter: ") + name);
  }
}
void ParamManager::declare_parameter(
  const std::string & name, const tam::types::param::param_variant_t & default_value,
  const tam::types::param::ParameterType & param_type, const std::string & description)
{
  param_names.insert(name);
  param_descriptions.insert({name, description});
  param_values.insert({name, tam::types::param::ParameterValue(param_type, default_value)});
}
}  // namespace tam::core
