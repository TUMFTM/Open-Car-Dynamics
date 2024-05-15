// Copyright 2023 Daniel Esser
#pragma once
#include <algorithm>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "param_manager_base.hpp"
namespace tam::core
{
class ParamManagerComposer : public tam::interfaces::ParamManagerBase
{
private:
  std::unordered_set<std::string> param_names;
  std::unordered_map<std::string, std::string> param_descriptions;
  std::unordered_map<std::string, tam::types::param::ParameterValue> param_values;
  // sub param manager
  std::vector<std::shared_ptr<tam::interfaces::ParamManagerBase>> param_manager_list_;

public:
  // Constructor
  ParamManagerComposer(std::vector<std::shared_ptr<tam::interfaces::ParamManagerBase>> list);
  // methods
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