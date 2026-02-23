// Copyright 2026 Simon Sagmeister
#include "ocd_aerodynamics_models_cpp/default.hpp"

#include <cmath>

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "tum_helpers_cpp/numerical.hpp"
namespace tam::ocd::aerodynamics
{
types::AeroModelOutput DefaultAerodynamicsModel::evaluate(types::AeroModelInput const & u)
{
  // vx, vy - long/lat velocity in m/s, theta - pitch angle in rad
  auto sign = [](double x) {
    if (x < 0) {
      return -1;
    } else {
      return 1;
    }
  };

  types::AeroModelOutput out;

  // Lift cofficient multiplier interpolation
  double delta_c_l = tam::helpers::numerical::interp(
    u.vx_mps, p_.c_l_velocity_scaling__vel_mps, p_.c_l_velocity_scaling__delta_c_l);
  // Aero center translation interpolation
  double aero_center_translation_m = tam::helpers::numerical::interp(
    u.pitch_angle_rad, p_.pitch_aero_center_translation__pitch_rad,
    p_.pitch_aero_center_translation__aero_center_translation_m);

  out.force_cog_N.x =
    -0.5 * p_.c_d * p_.air_density_kgpm3 * p_.A_m2 * std::pow(u.vx_mps, 2) * sign(u.vx_mps);
  // To be precise the area of calculating the drag here is different for lateral movement
  // but this is neglected here
  out.force_cog_N.y =
    -0.5 * p_.c_d * p_.air_density_kgpm3 * p_.A_m2 * std::pow(u.vy_mps, 2) * sign(u.vy_mps);
  out.force_cog_N.z =
    0.5 * (p_.c_l + delta_c_l) * p_.air_density_kgpm3 * p_.A_m2 * std::pow(u.vx_mps, 2);

  // Aero Balance
  out.torque_Nm.y = -out.force_cog_N.z * (p_.d_aero_center_cog_m + aero_center_translation_m);

  // Zero for everything else
  out.torque_Nm.x = 0;
  out.torque_Nm.z = 0;

  return out;
}
void DefaultAerodynamicsModel::declare_parameters(
  tam::pmg::ParamReferenceManager * param_manager, std::string name_prefix)
{
  param_manager->declare_parameter(
    name_prefix + "air_density_kgpm3", &(p_.air_density_kgpm3), 1.225,
    tam::pmg::ParameterType::DOUBLE, "");

  // Base parameters
  param_manager->declare_parameter(
    name_prefix + "A_m2", &(p_.A_m2), 1.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c_d", &(p_.c_d), 1.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c_l", &(p_.c_l), -1.5, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "d_aero_center_cog_m", &(p_.d_aero_center_cog_m), 0.0,
    tam::pmg::ParameterType::DOUBLE, "");

  // Velocity scaling of cl
  param_manager->declare_parameter(
    name_prefix + "c_l_velocity_scaling.vel_mps", &(p_.c_l_velocity_scaling__vel_mps),
    std::vector<double>{0, 100}, tam::pmg::ParameterType::DOUBLE_ARRAY, "");

  param_manager->declare_parameter(
    name_prefix + "c_l_velocity_scaling.delta_c_l", &(p_.c_l_velocity_scaling__delta_c_l),
    std::vector<double>{0, 0}, tam::pmg::ParameterType::DOUBLE_ARRAY, "");

  // Pitch correction since pitch moves the center of aero forces
  param_manager->declare_parameter(
    name_prefix + "pitch_aero_center_translation.pitch_rad",
    &(p_.pitch_aero_center_translation__pitch_rad), std::vector<double>{-M_PI, M_PI},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");

  param_manager->declare_parameter(
    name_prefix + "pitch_aero_center_translation.aero_center_translation_m",
    &(p_.pitch_aero_center_translation__aero_center_translation_m), std::vector<double>{0, 0},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");
}
}  // namespace tam::ocd::aerodynamics
