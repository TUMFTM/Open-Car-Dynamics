// Copyright 2026 Simon Sagmeister

#include "ocd_tire_models_cpp/mf_simple.hpp"
namespace tam::ocd::tire_models
{
types::TireModelOutput MF_Simple::evaluate_static(
  types::TireModelInput const & input, Parameters const & parameters)
{
  types::TireModelOutput out;
  // clang-format off
  out.longitudinal_force_N = input.F_z_N *
    (parameters.longitudinal.D *
      std::sin(
       parameters.longitudinal.C *
       std::atan(
         parameters.longitudinal.B * input.longitudinal_slip - parameters.longitudinal.E *
          ( parameters.longitudinal.B * input.longitudinal_slip -
            std::atan(parameters.longitudinal.B * input.longitudinal_slip)
          )
        )
      )
    );
  out.lateral_force_N = input.F_z_N *
    (parameters.lateral.D *
      std::sin(
        parameters.lateral.C *
        std::atan(
          parameters.lateral.B * std::tan(input.slip_angle_rad) - parameters.lateral.E *
            ( parameters.lateral.B * std::tan(input.slip_angle_rad) -
              std::atan(parameters.lateral.B * std::tan(input.slip_angle_rad))
            )
        )
      )
    );
  // clang-format on

  return out;
}
types::TireModelOutput MF_Simple::evaluate(types::TireModelInput const & input)
{
  return evaluate_static(input, p_);
}
void MF_Simple::declare_parameters(
  tam::pmg::ParamReferenceManager * param_manager, std::string name_prefix)
{
  // clang-format off
  param_manager->declare_parameter(name_prefix + "longitudinal.B", &(p_.longitudinal.B), 20.0, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  param_manager->declare_parameter(name_prefix + "longitudinal.C", &(p_.longitudinal.C), 2.0, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  param_manager->declare_parameter(name_prefix + "longitudinal.D", &(p_.longitudinal.D), 1.7, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  param_manager->declare_parameter(name_prefix + "longitudinal.E", &(p_.longitudinal.E), 0.7, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  param_manager->declare_parameter(name_prefix + "lateral.B", &(p_.lateral.B), 10.0, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  param_manager->declare_parameter(name_prefix + "lateral.C", &(p_.lateral.C), 1.6, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  param_manager->declare_parameter(name_prefix + "lateral.D", &(p_.lateral.D), 1.7, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  param_manager->declare_parameter(name_prefix + "lateral.E", &(p_.lateral.E), -1.9, tam::pmg::ParameterType::DOUBLE, ""); // NOLINT
  // clang-format on
}
}  // namespace tam::ocd::tire_models
