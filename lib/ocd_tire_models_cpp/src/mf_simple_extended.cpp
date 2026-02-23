// Copyright 2026 Simon Sagmeister

#include "ocd_tire_models_cpp/mf_simple_extended.hpp"
namespace tam::ocd::tire_models
{
types::TireModelOutput MF_Simple_Extended::evaluate(types::TireModelInput const & input)
{
  types::TireModelInput edited_input = input;
  MF_Simple::Parameters edited_params = mf_simple_.p_;
  // region respect combined slip sitations
  // Idea for combined slip according to bechtloff2018:
  // Calc a single combined slip provide this slip to both the lateral and the longitudinal model
  // Afterwards scale down to preserve the original direction of force vector
  double slip_lateral = std::tan(std::clamp(input.slip_angle_rad, -0.45 * M_PI, 0.45 * M_PI));
  double slip_overall = std::sqrt(std::pow(slip_lateral, 2) + std::pow(input.longitudinal_slip, 2));
  edited_input.longitudinal_slip = slip_overall;
  edited_input.slip_angle_rad = std::atan(slip_overall);
  // endregion
  // region tire load degression from pajceka
  double d_fz = (input.F_z_N - p_.FNOMIN) / p_.FNOMIN;
  edited_params.lateral.D += p_.PDY2 * d_fz;
  edited_params.longitudinal.D += p_.PDY2 * d_fz;
  // endregion
  auto mf_simple_out = mf_simple_.evaluate_static(edited_input, edited_params);

  auto mf_extended_out = mf_simple_out;
  // region respect combined slip sitations
  if (slip_overall < 1e-6) return mf_extended_out;  // Avoid zero devision
  // Apply combination of slip accoring to bechtloff2018
  mf_extended_out.lateral_force_N = mf_simple_out.lateral_force_N * (slip_lateral / slip_overall);
  mf_extended_out.longitudinal_force_N =
    mf_simple_out.longitudinal_force_N * (input.longitudinal_slip / slip_overall);
  // endregion

  return mf_extended_out;
}
void MF_Simple_Extended::declare_parameters(
  tam::pmg::ParamReferenceManager * param_manager, std::string name_prefix)
{
  // clang-format off
  mf_simple_.declare_parameters(param_manager, name_prefix);
  param_manager->declare_parameter(name_prefix + "FNOMIN", &(p_.FNOMIN), 3000.0, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  param_manager->declare_parameter(name_prefix + "PDY2", &(p_.PDY2), -0.1, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  // clang-format on
}
}  // namespace tam::ocd::tire_models
