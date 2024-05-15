// Copyright 2023 Simon Sagmeister

#include "vehicle_dynamics_helpers_cpp/tire_models.hpp"
namespace tam::sim::helpers::tire_models
{
TireModelOutput Linear::operator()(
  TireModelInput const & input, Parameters const & parameters) const
{
  TireModelOutput out;
  // clang-format off
  out.lateral_force_N = parameters.C_alpha * input.slip_angle_rad * input.F_z_N;
  out.longitudinal_force_N = parameters.C_sx * input.longitudinal_slip * input.F_z_N;
  out.self_aligning_moment_Nm = 0;
  return out;
}
namespace integration
{
void _declare_tire_model_parameters(
  std::string prefix, Linear::Parameters * param_struct,
  tam::core::ParamManagerNonOwning * param_manager)
{
  // clang-format off
  param_manager->declare_parameter_non_owning(prefix + "C_sx", 3000.0, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->C_sx)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "C_alpha", -0.1, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->C_alpha)); // NOLINT
  // clang-format on
}
}  // namespace integration
TireModelOutput MF_Simple::operator()(
  TireModelInput const & input, Parameters const & parameters) const
{
  TireModelOutput out;
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
namespace integration
{
void _declare_tire_model_parameters(
  std::string prefix, MF_Simple::Parameters * param_struct,
  tam::core::ParamManagerNonOwning * param_manager)
{
  // clang-format off
  param_manager->declare_parameter_non_owning(prefix + "longitudinal.B", 20.0, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->longitudinal.B)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "longitudinal.C", 2.0, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->longitudinal.C)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "longitudinal.D", 1.7, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->longitudinal.D)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "longitudinal.E", 0.7, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->longitudinal.E)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "lateral.B", 10.0, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->lateral.B)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "lateral.C", 1.6, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->lateral.C)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "lateral.D", 1.7, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->lateral.D)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "lateral.E", -1.9, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->lateral.E)); // NOLINT
  // clang-format on
}
}  // namespace integration
TireModelOutput MF_Simple_Extended::operator()(
  TireModelInput const & input, Parameters const & parameters) const
{
  auto edited_input = input;
  auto edited_params = parameters.p_mf_simple;
  // region respect combined slip sitations
  // Idea for combined slip according to bechtloff2018:
  // Calc a single combined slip provide this slip to both the lateral and the longitunal model
  // Afterwards scale down to preserve the original direction of force vector
  double slip_lateral = std::tan(std::clamp(input.slip_angle_rad, -0.45 * M_PI, 0.45 * M_PI));
  double slip_overall = std::sqrt(std::pow(slip_lateral, 2) + std::pow(input.longitudinal_slip, 2));
  edited_input.longitudinal_slip = slip_overall;
  edited_input.slip_angle_rad = std::atan(slip_overall);
  // endregion
  // region tire load degression from pajceka
  double d_fz = (input.F_z_N - parameters.FNOMIN) / parameters.FNOMIN;
  edited_params.lateral.D += parameters.PDY2 * d_fz;
  edited_params.longitudinal.D += parameters.PDY2 * d_fz;
  // endregion
  auto mf_simple_out = mf_simple(edited_input, edited_params);

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
namespace integration
{
void _declare_tire_model_parameters(
  std::string prefix, MF_Simple_Extended::Parameters * param_struct,
  tam::core::ParamManagerNonOwning * param_manager)
{
  // clang-format off
  _declare_tire_model_parameters(prefix, &(param_struct->p_mf_simple), param_manager);
  param_manager->declare_parameter_non_owning(prefix + "FNOMIN", 3000.0, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->FNOMIN)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PDY2", -0.1, tam::types::param::ParameterType::DOUBLE, "", &(param_struct->PDY2)); // NOLINT
  // clang-format on
}
}  // namespace integration
// MF52
// ====================================================================================================
TireModelOutput MF52::operator()(TireModelInput const & input, Parameters const & parameters) const
{
  TireModelOutput out;
  double F_x_o = long_force_pure_slip(input.F_z_N, input.longitudinal_slip, parameters);
  double F_y_o = lat_force_pure_slip(input.F_z_N, -input.slip_angle_rad, 0.0, parameters);
  out.longitudinal_force_N = long_force_combined_slip(
    input.F_z_N, input.longitudinal_slip, -input.slip_angle_rad, F_x_o, 0.0, parameters);
  out.lateral_force_N = lat_force_combined_slip(
    input.F_z_N, input.longitudinal_slip, -input.slip_angle_rad, F_y_o, 0.0, parameters);
  // Implement MF52 in here
  return out;
}
double MF52::long_force_pure_slip(double F_z, double kappa, Parameters parameters) const
{
  double F_z_o_a = parameters.LFZO * parameters.FNOMIN;
  double dfz = (F_z - F_z_o_a) / F_z_o_a;
  double S_H_x = (parameters.PHX1 + parameters.PHX2 * dfz) * parameters.LHX;
  double S_V_x = F_z * (parameters.PVX1 + parameters.PVX2 * dfz) * parameters.LVX * parameters.LMUX;
  double kappa_x = kappa + S_H_x;
  double C_x = parameters.PCX1 * parameters.LCX;
  double mu_x = (parameters.PDX1 + parameters.PDX2 * dfz) * parameters.LMUX;
  double D_x = mu_x * F_z;
  double E_x = (parameters.PEX1 + parameters.PEX2 * dfz + parameters.PEX3 * std::pow(dfz, 2)) *
               (1.0 - parameters.PEX4 * std::copysign(1.0, kappa_x)) * parameters.LEX;
  double K_x = F_z * (parameters.PKX1 + parameters.PKX2 * dfz) * std::exp(parameters.PKX3 * dfz) *
               parameters.LKX;
  double B_x = K_x / (C_x * D_x);
  // Longitudinal Force (pure longitudinal slip)
  double F_x_o =
    D_x *
      std::sin(C_x * std::atan(B_x * kappa_x - E_x * (B_x * kappa_x - std::atan(B_x * kappa_x)))) +
    S_V_x;
  return F_x_o;
}
double MF52::lat_force_pure_slip(
  double F_z, double alpha, double gamma, Parameters parameters) const
{
  double gamma_s = std::sin(gamma);
  double F_z_o_a = parameters.LFZO * parameters.FNOMIN;
  double dfz = (F_z - F_z_o_a) / F_z_o_a;
  double K_y_g_o = F_z * (parameters.PKY6 + parameters.PKY7 * dfz) * parameters.LKYG;
  double S_V_y_g =
    F_z * (parameters.PVY3 + parameters.PVY4 * dfz) * gamma_s * parameters.LKYG * parameters.LMUY;
  double S_V_y =
    F_z * (parameters.PVY1 + parameters.PVY2 * dfz) * parameters.LVY * parameters.LMUY + S_V_y_g;
  double K_y =
    parameters.PKY1 * F_z_o_a *
    std::sin(
      parameters.PKY4 *
      std::atan(F_z / ((parameters.PKY2 + parameters.PKY5 * std::pow(gamma_s, 2)) * F_z_o_a))) /
    (1.0 + parameters.PKY3 * std::pow(gamma_s, 2)) * parameters.LKY;
  double S_H_y = (parameters.PHY1 + parameters.PHY2 * dfz) * parameters.LHY +
                 (K_y_g_o * gamma_s - S_V_y_g) / K_y;
  double C_y = parameters.PCY1 * parameters.LCY;
  double mu_y =
    ((parameters.PDY1 + parameters.PDY2 * dfz) / (1.0 + parameters.PDY3 * std::pow(gamma_s, 2))) *
    parameters.LMUY;
  double D_y = mu_y * F_z;
  double B_y = K_y / (C_y * D_y);
  double alpha_y = alpha + S_H_y;
  double E_y = (parameters.PEY1 + parameters.PEY2 * dfz) *
               (1.0 + parameters.PEY5 * std::pow(gamma_s, 2) -
                (parameters.PEY3 + parameters.PEY4 * gamma_s) * std::copysign(1.0, alpha_y)) *
               parameters.LEY;
  double F_y_o =
    D_y *
      std::sin(C_y * std::atan(B_y * alpha_y - E_y * (B_y * alpha_y - std::atan(B_y * alpha_y)))) +
    S_V_y;
  return F_y_o;
}
double MF52::long_force_combined_slip(
  double F_z, double kappa, double alpha, double F_x_o, double gamma, Parameters parameters) const
{
  double gamma_s = std::sin(gamma);
  double F_z_o_a = parameters.LFZO * parameters.FNOMIN;
  double dfz = (F_z - F_z_o_a) / F_z_o_a;
  double S_H_x_a = parameters.RHX1;
  double E_x_a = parameters.REX1 + parameters.REX2 * dfz;
  double C_x_a = parameters.RCX1;
  double B_x_a = (parameters.RBX1 + parameters.RBX3 * std::pow(gamma_s, 2)) *
                 std::cos(std::atan(parameters.RBX2 * kappa)) * parameters.LXAL;
  double alpha_S = alpha + S_H_x_a;
  double G_x_a_o = std::cos(
    C_x_a * std::atan(B_x_a * S_H_x_a - E_x_a * (B_x_a * S_H_x_a - std::atan(B_x_a * S_H_x_a))));
  double G_x_a =
    std::cos(
      C_x_a * std::atan(B_x_a * alpha_S - E_x_a * (B_x_a * alpha_S - std::atan(B_x_a * alpha_S)))) /
    G_x_a_o;
  double F_x = G_x_a * F_x_o;
  return F_x;
}
double MF52::lat_force_combined_slip(
  double F_z, double kappa, double alpha, double F_y_o, double gamma, Parameters parameters) const
{
  double gamma_s = std::sin(gamma);
  double F_z_o_a = parameters.LFZO * parameters.FNOMIN;
  double dfz = (F_z - F_z_o_a) / F_z_o_a;
  double mu_y =
    ((parameters.PDY1 + parameters.PDY2 * dfz) / (1.0 + parameters.PDY3 * std::pow(gamma_s, 2))) *
    parameters.LMUY;
  double D_V_y_k = mu_y * F_z *
                   (parameters.RVY1 + parameters.RVY2 * dfz + parameters.RVY3 * gamma_s) *
                   std::cos(std::atan(parameters.RVY4 * alpha));
  double S_V_y_k =
    D_V_y_k * std::sin(parameters.RVY5 * std::atan(parameters.RVY6 * kappa)) * parameters.LVYKA;
  double S_H_y_k = parameters.RHY1 + parameters.RHY2 * dfz;
  double E_y_k = parameters.REY1 + parameters.REY2 * dfz;
  double C_y_k = parameters.RCY1;
  double B_y_k = (parameters.RBY1 + parameters.RBY4 * std::pow(gamma_s, 2)) *
                 std::cos(std::atan(parameters.RBY2 * (alpha - parameters.RBY3))) * parameters.LYKA;
  double kappa_S = kappa + S_H_y_k;
  double G_y_k_o = std::cos(
    C_y_k * std::atan(B_y_k * S_H_y_k - E_y_k * (B_y_k * S_H_y_k - std::atan(B_y_k * S_H_y_k))));
  double G_y_k =
    std::cos(
      C_y_k * std::atan(B_y_k * kappa_S - E_y_k * (B_y_k * kappa_S - std::atan(B_y_k * kappa_S)))) /
    G_y_k_o;
  double F_y = G_y_k * F_y_o + S_V_y_k;
  return F_y;
}
namespace integration
{
void _declare_tire_model_parameters(
  std::string prefix, MF52::Parameters * param_struct,
  tam::core::ParamManagerNonOwning * param_manager)
{
  // clang-format off
  // Long parameters
  param_manager->declare_parameter_non_owning(prefix + "PCX1", 1.6    , tam::types::param::ParameterType::DOUBLE, "Shape factor Cfx for longitudinal force", &(param_struct->PCX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PDX1", 1.5    , tam::types::param::ParameterType::DOUBLE, "Longitudinal friction Mux at Fznom", &(param_struct->PDX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PDX2", -0.04  , tam::types::param::ParameterType::DOUBLE, "Variation of friction Mux with load", &(param_struct->PDX2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PDX3", 0.1    , tam::types::param::ParameterType::DOUBLE, "Variation of friction Mux with camber", &(param_struct->PDX3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PEX1", 0.7    , tam::types::param::ParameterType::DOUBLE, "Longitudinal curvature Efx at Fznom", &(param_struct->PEX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PEX2", -0.17  , tam::types::param::ParameterType::DOUBLE, "Variation of curvature Efx with load", &(param_struct->PEX2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PEX3", 0.023  , tam::types::param::ParameterType::DOUBLE, "Variation of curvature Efx with load squared", &(param_struct->PEX3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PEX4", -0.14  , tam::types::param::ParameterType::DOUBLE, "Factor in curvature Efx while driving", &(param_struct->PEX4)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKX1", 30.7   , tam::types::param::ParameterType::DOUBLE, "Longitudinal slip stiffness Kfx/Fz at Fznom", &(param_struct->PKX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKX2", 0.27   , tam::types::param::ParameterType::DOUBLE, "Variation of slip stiffness Kfx/Fz with load", &(param_struct->PKX2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKX3", 0.13   , tam::types::param::ParameterType::DOUBLE, "Exponent in slip stiffness Kfx/Fz with load", &(param_struct->PKX3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PHX1", 0.0    , tam::types::param::ParameterType::DOUBLE, "Horizontal shift Shx at Fznom", &(param_struct->PHX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PHX2", 0.0    , tam::types::param::ParameterType::DOUBLE, "Variation of shift Shx with load", &(param_struct->PHX2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PVX1", 0.0    , tam::types::param::ParameterType::DOUBLE, "Vertical shift Svx/Fz at Fznom", &(param_struct->PVX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PVX2", 0.0    , tam::types::param::ParameterType::DOUBLE, "ariation of shift Svx/Fz with load", &(param_struct->PVX2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RBX1", 17.4   , tam::types::param::ParameterType::DOUBLE, "Slope factor for combined slip Fx reduction", &(param_struct->RBX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RBX2", 12.9   , tam::types::param::ParameterType::DOUBLE, "Variation of slope Fx reduction with kappa", &(param_struct->RBX2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RBX3", 0.0    , tam::types::param::ParameterType::DOUBLE, "Camber influence on combined slip longitudinal force", &(param_struct->RBX3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RCX1", 1.1    , tam::types::param::ParameterType::DOUBLE, "Shape factor for combined slip Fx reduction", &(param_struct->RCX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "REX1", 0.0    , tam::types::param::ParameterType::DOUBLE, "Curvature factor of combined Fx", &(param_struct->REX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "REX2", 0.0    , tam::types::param::ParameterType::DOUBLE, "Curvature factor of combined Fx with load", &(param_struct->REX2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RHX1", 0.001  , tam::types::param::ParameterType::DOUBLE, "Shift factor for combined slip Fx reduction", &(param_struct->RHX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PTX1", 0.75   , tam::types::param::ParameterType::DOUBLE, "Relaxation length SigKap0/Fz at Fznom", &(param_struct->PTX1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PTX2", 0.007  , tam::types::param::ParameterType::DOUBLE, "Variation of SigKap0/Fz with load", &(param_struct->PTX2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PTX3", -0.013 , tam::types::param::ParameterType::DOUBLE, "Variation of SigKap0/Fz with exponent of load", &(param_struct->PTX3)); // NOLINT
  // clang-format on

  // clang-format off
  // Lat parameters
  param_manager->declare_parameter_non_owning(prefix + "PCY1", 1.5    , tam::types::param::ParameterType::DOUBLE, "Shape factor Cfy for lateral forces", &(param_struct->PCY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PDY1", 1.2    , tam::types::param::ParameterType::DOUBLE, "Lateral friction Muy ", &(param_struct->PDY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PDY2", -0.09  , tam::types::param::ParameterType::DOUBLE, "Variation of friction Muy with load ", &(param_struct->PDY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PDY3", 0.1    , tam::types::param::ParameterType::DOUBLE, "Variation of friction Muy with squared camber", &(param_struct->PDY3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PEY1", 0.1    , tam::types::param::ParameterType::DOUBLE, "Lateral curvature Efy at Fznom", &(param_struct->PEY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PEY2", -1.2   , tam::types::param::ParameterType::DOUBLE, "Variation of curvature Efy with load", &(param_struct->PEY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PEY3", 0.05   , tam::types::param::ParameterType::DOUBLE, "Zero order camber dependency of curvature Efy", &(param_struct->PEY3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PEY4", -8.1   , tam::types::param::ParameterType::DOUBLE, "Variation of curvature Efy with camber", &(param_struct->PEY4)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKY1", -75.5  , tam::types::param::ParameterType::DOUBLE, "Maximum value of stiffness Kfy/Fznom", &(param_struct->PKY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKY2", 4.65   , tam::types::param::ParameterType::DOUBLE, "Load at which Kfy reaches maximum value", &(param_struct->PKY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKY3", -0.6   , tam::types::param::ParameterType::DOUBLE, "Variation of Kfy/Fznom with camber", &(param_struct->PKY3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PHY1", 0.003  , tam::types::param::ParameterType::DOUBLE, "Horizontal shift Shy at Fznom", &(param_struct->PHY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PHY2", -0.0006, tam::types::param::ParameterType::DOUBLE, "Variation of shift Shy with load", &(param_struct->PHY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PHY3", 0.004  , tam::types::param::ParameterType::DOUBLE, "Variation of shift Shy with camber", &(param_struct->PHY3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PVY1", 0.04   , tam::types::param::ParameterType::DOUBLE, "Vertical shift in Svy/Fz at Fznom", &(param_struct->PVY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PVY2", -0.02  , tam::types::param::ParameterType::DOUBLE, "Variation of shift Svy/Fz with load", &(param_struct->PVY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PVY3", -0.97 	, tam::types::param::ParameterType::DOUBLE, "Variation of shift Svy/Fz with camber", &(param_struct->PVY3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PVY4", -0.41  , tam::types::param::ParameterType::DOUBLE, "Variation of shift Svy/Fz with camber and load", &(param_struct->PVY4)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RBY1", 20.6   , tam::types::param::ParameterType::DOUBLE, "Slope factor for combined Fy reduction", &(param_struct->RBY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RBY2", -23.5  , tam::types::param::ParameterType::DOUBLE, "Variation of slope Fy reduction with alpha", &(param_struct->RBY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RBY3", 0.001  , tam::types::param::ParameterType::DOUBLE, "Shift term for alpha in slope Fy reduction", &(param_struct->RBY3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RCY1", 1.0    , tam::types::param::ParameterType::DOUBLE, "Shape factor for combined Fy reduction", &(param_struct->RCY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "REY1", 0.0    , tam::types::param::ParameterType::DOUBLE, "Curvature factor of combined Fy", &(param_struct->REY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "REY2", 0.0    , tam::types::param::ParameterType::DOUBLE, "Curvature factor of combined Fy with load", &(param_struct->REY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RHY1", -0.02  , tam::types::param::ParameterType::DOUBLE, "Shift factor for combined Fy reduction", &(param_struct->RHY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RHY2", 0.0    , tam::types::param::ParameterType::DOUBLE, "Shift factor for combined Fy reduction with load", &(param_struct->RHY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RVY1", 0.16   , tam::types::param::ParameterType::DOUBLE, "Kappa induced side force Svyk/Muy*Fz at Fznom", &(param_struct->RVY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RVY2", 0.03   , tam::types::param::ParameterType::DOUBLE, "Variation of Svyk/Muy*Fz with load", &(param_struct->RVY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RVY3", 27.5   , tam::types::param::ParameterType::DOUBLE, "Variation of Svyk/Muy*Fz with camber", &(param_struct->RVY3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RVY4", -29.7  , tam::types::param::ParameterType::DOUBLE, "Variation of Svyk/Muy*Fz with alpha", &(param_struct->RVY4)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RVY5", 0.03   , tam::types::param::ParameterType::DOUBLE, "Variation of Svyk/Muy*Fz with kappa", &(param_struct->RVY5)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RVY6", 0.0    , tam::types::param::ParameterType::DOUBLE, "Variation of Svyk/Muy*Fz with atan(kappa)", &(param_struct->RVY6)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PTY1", 3.8    , tam::types::param::ParameterType::DOUBLE, "Peak value of relaxation length SigAlp0/R0", &(param_struct->PTY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PTY2", 4.7    , tam::types::param::ParameterType::DOUBLE, "Value of Fz/Fznom where SigAlp0 is extreme", &(param_struct->PTY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PEY5", 0.0    , tam::types::param::ParameterType::DOUBLE, "Lateral curvature variation with camber squared", &(param_struct->PEY5)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKY4", 2.0    , tam::types::param::ParameterType::DOUBLE, "Lateral force stiffness, KFy curvature", &(param_struct->PKY4)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKY5", 0.0    , tam::types::param::ParameterType::DOUBLE, "Variation of peak stiffness with squared camber", &(param_struct->PKY5)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKY6", 0.0    , tam::types::param::ParameterType::DOUBLE, "Lateral force, Fy, camber stiffness factor", &(param_struct->PKY6)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PKY7", 0.0    , tam::types::param::ParameterType::DOUBLE, "Camber stiffness vertical load dependency", &(param_struct->PKY7)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PPY1", 0.0    , tam::types::param::ParameterType::DOUBLE, "Cornering stiffness variation with inflation pressure", &(param_struct->PPY1)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PPY2", 0.0    , tam::types::param::ParameterType::DOUBLE, "Cornering stiffness variation with inflation pressure induced nominal load dependency", &(param_struct->PPY2)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PPY3", 0.0    , tam::types::param::ParameterType::DOUBLE, "Linear inflation pressure on peak lateral friction", &(param_struct->PPY3)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PPY4", 0.0    , tam::types::param::ParameterType::DOUBLE, "Quadratic inflation pressure on peak lateral friction", &(param_struct->PPY4)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "PPY5", 0.0    , tam::types::param::ParameterType::DOUBLE, "Inflation pressure effect on camber stiffness", &(param_struct->PPY5)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix + "RBY4", 0.0    , tam::types::param::ParameterType::DOUBLE, "Lateral force, Fy, combined stiffness variation from camber", &(param_struct->RBY4)); // NOLINT
  // clang-format on

  // clang-format off
  // Scaling factors
  param_manager->declare_parameter_non_owning(prefix +  "LFZO" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of nominal (rated) load", &(param_struct->LFZO)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LCX"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fx shape factor", &(param_struct->LCX)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LMUX" , 0.97 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fx peak friction coefficient", &(param_struct->LMUX)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LEX"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fx curvature factor", &(param_struct->LEX)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LKX"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fx slip stiffness", &(param_struct->LKX)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LHX"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fx horizontal shift", &(param_struct->LHX)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LVX"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fx vertical shift", &(param_struct->LVX)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LGAX" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of camber for Fx", &(param_struct->LGAX)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LCY"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fy shape factor", &(param_struct->LCY)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LMUY" , 0.97 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fy peak friction coefficient", &(param_struct->LMUY)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LEY"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fy curvature factor", &(param_struct->LEY)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LKY"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fy cornering stiffness", &(param_struct->LKY)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LKYG" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fy camber force stiffness", &(param_struct->LKYG)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LHY"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fy horizontal shift", &(param_struct->LHY)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LVY"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Fy vertical shift", &(param_struct->LVY)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LGAY" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of camber for Fy", &(param_struct->LGAY)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LTR"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Peak of pneumatic trail", &(param_struct->LTR)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LRES" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor for offset of residual torque", &(param_struct->LRES)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LGAZ" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of camber for Mz", &(param_struct->LGAZ)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LXAL" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of alpha influence on Fx", &(param_struct->LXAL)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LYKA" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of alpha influence on Fx", &(param_struct->LYKA)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LVYKA", 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of kappa induced Fy", &(param_struct->LVYKA)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LS"   , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Moment arm of Fx", &(param_struct->LS)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LSGKP", 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Relaxation length of Fx", &(param_struct->LSGKP)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LSGAL", 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Relaxation length of Fy", &(param_struct->LSGAL)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LGYR" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of gyroscopic torque", &(param_struct->LGYR)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LMX"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of overturning couple", &(param_struct->LMX)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LVMX" , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of Mx vertical shift", &(param_struct->LVMX)); // NOLINT
  param_manager->declare_parameter_non_owning(prefix +  "LMY"  , 1.0 , tam::types::param::ParameterType::DOUBLE, "Scale factor of rolling resistance torque", &(param_struct->LMY)); // NOLINT
  // clang-format on

  // Other params
  param_manager->declare_parameter_non_owning(
    prefix + "FNOMIN", 3000.0, tam::types::param::ParameterType::DOUBLE, "Nominal wheel load",
    &(param_struct->FNOMIN));  // NOLINT
}
}  // namespace integration
}  // namespace tam::sim::helpers::tire_models
