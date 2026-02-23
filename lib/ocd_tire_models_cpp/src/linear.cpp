// Copyright 2026 Simon Sagmeister

#include "ocd_tire_models_cpp/linear.hpp"
namespace tam::ocd::tire_models
{
types::TireModelOutput Linear::evaluate(types::TireModelInput const & input)
{
  types::TireModelOutput out;
  // clang-format off
  out.lateral_force_N = p_.C_alpha * input.slip_angle_rad * input.F_z_N;
  out.longitudinal_force_N = p_.C_sx * input.longitudinal_slip * input.F_z_N;
  out.self_aligning_moment_Nm = 0;
  return out;
}
void Linear::declare_parameters(
  tam::pmg::ParamReferenceManager * param_manager, std::string name_prefix)
{
  // clang-format off
  param_manager->declare_parameter(name_prefix + "C_sx",&(p_.C_sx), 3000.0, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  param_manager->declare_parameter(name_prefix + "C_alpha", &(p_.C_alpha), -0.1, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  // clang-format on
}
}  // namespace tam::ocd::tire_models
