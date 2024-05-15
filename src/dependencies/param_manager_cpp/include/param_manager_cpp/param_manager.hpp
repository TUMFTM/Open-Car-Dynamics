// Copyright 2023 Simon Sagmeister
#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "param_manager_base.hpp"
namespace tam::core
{
class ParamManager : public tam::interfaces::ParamManagerBase
{
private:
  std::unordered_set<std::string> param_names;
  std::unordered_map<std::string, std::string> param_descriptions;
  std::unordered_map<std::string, tam::types::param::ParameterValue> param_values;

public:
  bool has_parameter(const std::string & name) const override;
  std::unordered_set<std::string> list_parameters() const override;
  tam::types::param::ParameterType get_parameter_type(const std::string & name) const override;
  std::string get_parameter_description(const std::string & name) const override;
  void set_parameter_value(
    const std::string & name, const tam::types::param::param_variant_t & value) override;
  tam::types::param::ParameterValue get_parameter_value(const std::string & name) const override;
  void declare_parameter(
    const std::string & name, const tam::types::param::param_variant_t & default_value,
    const tam::types::param::ParameterType & param_type, const std::string & description) override;
};
}  // namespace tam::core
