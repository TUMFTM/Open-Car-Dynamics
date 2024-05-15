// Copyright 2023 Simon Sagmeister
#include "param_manager_cpp/param_manager_non_owning.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
namespace tam::core
{
// Function definitions
bool ParamManagerNonOwning::has_parameter(const std::string & name) const
{
  if (param_names.count(name)) {
    return true;
  } else {
    return false;
  }
}
std::unordered_set<std::string> ParamManagerNonOwning::list_parameters() const
{
  return param_names;
}
tam::types::param::ParameterType ParamManagerNonOwning::get_parameter_type(
  const std::string & name) const
{
  try {
    return param_references.at(name).get_type();
  } catch (const std::out_of_range & oor) {
    throw std::out_of_range(
      std::string("ParamManager: Tried accessing non declared parameter: ") + name);
  }
}
std::string ParamManagerNonOwning::get_parameter_description(const std::string & name) const
{
  try {
    return param_descriptions.at(name);
  } catch (const std::out_of_range & oor) {
    throw std::out_of_range(
      std::string("ParamManager: Tried accessing non declared parameter: ") + name);
  }
}
void ParamManagerNonOwning::set_parameter_value(
  const std::string & name, const tam::types::param::param_variant_t & value)
{
  try {
    param_references.at(name).set_parameter_value(value);
  } catch (const std::out_of_range & oor) {
    throw std::out_of_range(
      std::string("ParamManager: Tried setting non declared parameter: ") + name);
  }
}
tam::types::param::ParameterValue ParamManagerNonOwning::get_parameter_value(
  const std::string & name) const
{
  try {
    return param_references.at(name).to_value();
  } catch (const std::out_of_range & oor) {
    throw std::out_of_range(
      std::string("ParamManager: Tried accessing non declared parameter: ") + name);
  }
}
void ParamManagerNonOwning::declare_parameter_non_owning(
  const std::string & name, const tam::types::param::param_variant_t & default_value,
  const tam::types::param::ParameterType & param_type, const std::string & description,
  tam::types::param::param_ptr_variant_t storage_ptr)
{
  param_names.insert(name);
  param_descriptions.insert({name, description});
  param_references.insert(
    {name, tam::types::param::ParameterReference(param_type, storage_ptr, default_value)});
}
}  // namespace tam::core
