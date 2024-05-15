// Copyright 2023 Simon Sagmeister
#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <variant>
#include <vector>
namespace tam::types::param
{

enum class ParameterType {
  // NOT_SET = 0,  // Won't be supported even though ros2 in theory supports it
  BOOL = 1,
  INTEGER = 2,
  DOUBLE = 3,
  STRING = 4,
  // BYTE_ARRAY = 5, // Won't be supported even though ros2 in theory supports it
  BOOL_ARRAY = 6,
  INTEGER_ARRAY = 7,
  DOUBLE_ARRAY = 8,
  STRING_ARRAY = 9,
};

typedef std::variant<
  bool, int, double, std::string, std::vector<bool>, std::vector<int>, std::vector<double>,
  std::vector<std::string>>
  param_variant_t;
class ParameterValue
{
private:
  ParameterType parameter_type;
  param_variant_t value;

public:
  ParameterValue(const ParameterType & parameter_type_, const param_variant_t & value_)
  : parameter_type{parameter_type_}
  {
    set_parameter_value(value_);
  }
  void set_parameter_value(const param_variant_t & value_)
  {
    // Do this switch here to ensure type safety on set
    switch (parameter_type) {
      case ParameterType::BOOL:
        value = std::get<bool>(value_);
        break;
      case ParameterType::INTEGER:
        value = std::get<int>(value_);
        break;
      case ParameterType::DOUBLE:
        value = std::get<double>(value_);
        break;
      case ParameterType::STRING:
        value = std::get<std::string>(value_);
        break;
      case ParameterType::BOOL_ARRAY:
        value = std::get<std::vector<bool>>(value_);
        break;
      case ParameterType::INTEGER_ARRAY:
        value = std::get<std::vector<int>>(value_);
        break;
      case ParameterType::DOUBLE_ARRAY:
        value = std::get<std::vector<double>>(value_);
        break;
      case ParameterType::STRING_ARRAY:
        value = std::get<std::vector<std::string>>(value_);
        break;
      default:
        break;
    }
  }
  ParameterType get_type() const { return parameter_type; }
  bool as_bool() const { return std::get<bool>(value); }
  int as_int() const { return std::get<int>(value); }
  double as_double() const { return std::get<double>(value); }
  std::string as_string() const { return std::get<std::string>(value); }
  std::vector<bool> as_bool_array() const { return std::get<std::vector<bool>>(value); }
  std::vector<int> as_int_array() const { return std::get<std::vector<int>>(value); }
  std::vector<double> as_double_array() const { return std::get<std::vector<double>>(value); }
  std::vector<std::string> as_string_array() const
  {
    return std::get<std::vector<std::string>>(value);
  }
};
}  // namespace tam::types::param
namespace tam::interfaces
{
class ParamManagerBase
{
public:
  typedef std::shared_ptr<ParamManagerBase> SharedPtr;
  //
  virtual ~ParamManagerBase() = default;
  // Interace methods
  virtual bool has_parameter(const std::string & name) const = 0;
  virtual std::unordered_set<std::string> list_parameters() const = 0;
  virtual tam::types::param::ParameterType get_parameter_type(const std::string & name) const = 0;
  virtual std::string get_parameter_description(const std::string & name) const = 0;

  // Set parameter value functions
  virtual void set_parameter_value(
    const std::string & name, const tam::types::param::param_variant_t & value) = 0;

  // Get parameter value functions
  virtual tam::types::param::ParameterValue get_parameter_value(const std::string & name) const = 0;

  // Declare parameter values functions per datatype
  virtual void declare_parameter(
    const std::string & name, const tam::types::param::param_variant_t & default_value,
    const tam::types::param::ParameterType & param_type, const std::string & description) = 0;
};
}  // namespace tam::interfaces
