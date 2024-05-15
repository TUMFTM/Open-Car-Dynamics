// Copyright 2023 Simon Sagmeister
#pragma once

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "param_manager_base.hpp"
namespace tam::types::param
{
typedef std::variant<
  bool *, int *, double *, std::string *, std::vector<bool> *, std::vector<int> *,
  std::vector<double> *, std::vector<std::string> *>
  param_ptr_variant_t;
class ParameterReference
{
private:
  ParameterType parameter_type;
  param_ptr_variant_t value_ptr;

public:
  ParameterReference(
    ParameterType const & parameter_type_, param_ptr_variant_t & value_ptr_,
    param_variant_t const & value_)
  : parameter_type{parameter_type_}, value_ptr{value_ptr_}
  {
    set_parameter_value(value_);
  }
  void set_parameter_value(const param_variant_t & value_) const
  {
    // Do this switch here to ensure type safety on set
    switch (parameter_type) {
      case ParameterType::BOOL:
        *(std::get<bool *>(value_ptr)) = std::get<bool>(value_);
        break;
      case ParameterType::INTEGER:
        *(std::get<int *>(value_ptr)) = std::get<int>(value_);
        break;
      case ParameterType::DOUBLE:
        *(std::get<double *>(value_ptr)) = std::get<double>(value_);
        break;
      case ParameterType::STRING:
        *(std::get<std::string *>(value_ptr)) = std::get<std::string>(value_);
        break;
      case ParameterType::BOOL_ARRAY:
        *(std::get<std::vector<bool> *>(value_ptr)) = std::get<std::vector<bool>>(value_);
        break;
      case ParameterType::INTEGER_ARRAY:
        *(std::get<std::vector<int> *>(value_ptr)) = std::get<std::vector<int>>(value_);
        break;
      case ParameterType::DOUBLE_ARRAY:
        *(std::get<std::vector<double> *>(value_ptr)) = std::get<std::vector<double>>(value_);
        break;
      case ParameterType::STRING_ARRAY:
        *(std::get<std::vector<std::string> *>(value_ptr)) =
          std::get<std::vector<std::string>>(value_);
        break;
      default:
        break;
    }
  }
  ParameterValue to_value() const
  {
    switch (parameter_type) {
      // clang-format off
      case ParameterType::BOOL: return ParameterValue(parameter_type, *(std::get<bool *>(value_ptr))); // NOLINT
      case ParameterType::INTEGER: return ParameterValue(parameter_type, *(std::get<int *>(value_ptr))); // NOLINT
      case ParameterType::DOUBLE: return ParameterValue(parameter_type, *(std::get<double *>(value_ptr))); // NOLINT
      case ParameterType::STRING: return ParameterValue(parameter_type, *(std::get<std::string *>(value_ptr))); // NOLINT
      case ParameterType::BOOL_ARRAY: return ParameterValue(parameter_type, *(std::get<std::vector<bool> *>(value_ptr))); // NOLINT
      case ParameterType::INTEGER_ARRAY: return ParameterValue(parameter_type, *(std::get<std::vector<int> *>(value_ptr))); // NOLINT
      case ParameterType::DOUBLE_ARRAY: return ParameterValue(parameter_type, *(std::get<std::vector<double> *>(value_ptr))); // NOLINT
      case ParameterType::STRING_ARRAY: return ParameterValue(parameter_type, *(std::get<std::vector<std::string> *>(value_ptr))); // NOLINT
    }
    throw std::runtime_error("Something wrent wrong: ID 84bd1e6326");
    // clang-format on
  }
  ParameterType get_type() const { return parameter_type; }
};
}  // namespace tam::types::param
namespace tam::core
{
class ParamManagerNonOwning : public tam::interfaces::ParamManagerBase
{
private:
  std::unordered_set<std::string> param_names;
  std::unordered_map<std::string, std::string> param_descriptions;
  std::unordered_map<std::string, tam::types::param::ParameterReference> param_references;

public:
  bool has_parameter(const std::string & name) const override;
  std::unordered_set<std::string> list_parameters() const override;
  tam::types::param::ParameterType get_parameter_type(const std::string & name) const override;
  std::string get_parameter_description(const std::string & name) const override;
  void set_parameter_value(
    const std::string & name, const tam::types::param::param_variant_t & value) override;
  tam::types::param::ParameterValue get_parameter_value(const std::string & name) const override;
  void declare_parameter(
    const std::string &, const tam::types::param::param_variant_t &,
    const tam::types::param::ParameterType &, const std::string &) override
  {
    throw std::runtime_error(
      "Declaring an owning parameter is not allowed on the non owning param manager");
  }
  void declare_parameter_non_owning(
    const std::string & name, const tam::types::param::param_variant_t & default_value,
    const tam::types::param::ParameterType & param_type, const std::string & description,
    tam::types::param::param_ptr_variant_t storage_ptr);
};
}  // namespace tam::core
