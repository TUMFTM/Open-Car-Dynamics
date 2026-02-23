// Copyright 2026 Simon Sagmeister
#include "ocd_aerodynamics_models_cpp/ride_height.hpp"

#include <cmath>

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "tum_helpers_cpp/numerical.hpp"
namespace tam::ocd::aerodynamics
{
types::AeroModelOutput RideHeightAerodynamicsModel::evaluate(types::AeroModelInput const & u)
{
  auto sign = [](double x) {
    if (x < 0) {
      return -1;
    } else {
      return 1;
    }
  };

  types::AeroModelOutput out;

  double l_r = p_.l - p_.l_f;

  // Assumption of Aero centre placed on the axles
  double delta_z_front = u.z_m - p_.l_f * u.pitch_angle_rad;
  double delta_z_rear = u.z_m + l_r * u.pitch_angle_rad;

  // current RH = static RH - displacement - cog shift
  double current_rh_front = -(p_.static_ride_height_front_m - delta_z_front);
  double current_rh_rear = -(p_.static_ride_height_rear_m - delta_z_rear);

  // Lift Coefficient as 3rd order surfaces
  double c_l_front = p_.c0 + p_.c1_F * current_rh_front + p_.c2_F * std::pow(current_rh_front, 2) +
                     p_.c3_F * std::pow(current_rh_front, 3) + p_.c1_R * current_rh_rear +
                     p_.c2_R * std::pow(current_rh_rear, 2) +
                     p_.c3_R * std::pow(current_rh_rear, 3) +
                     p_.c4 * current_rh_front * current_rh_rear +
                     p_.c5 * std::pow(current_rh_front, 2) * current_rh_rear +
                     p_.c6 * current_rh_front * std::pow(current_rh_rear, 2);

  double c_l_rear = p_.a0 + p_.a1_F * current_rh_front + p_.a2_F * std::pow(current_rh_front, 2) +
                    p_.a3_F * std::pow(current_rh_front, 3) + p_.a1_R * current_rh_rear +
                    p_.a2_R * std::pow(current_rh_rear, 2) +
                    p_.a3_R * std::pow(current_rh_rear, 3) +
                    p_.a4 * current_rh_front * current_rh_rear +
                    p_.a5 * std::pow(current_rh_front, 2) * current_rh_rear +
                    p_.a6 * current_rh_front * std::pow(current_rh_rear, 2);

  // Clipping the lift values
  c_l_front = std::clamp(c_l_front, 0.0, 5.0);
  c_l_rear = std::clamp(c_l_rear, 0.0, 5.0);

  // Output forces
  out.force_cog_N.x =
    -0.5 * p_.c_d * p_.air_density_kgpm3 * p_.A_m2 * std::pow(u.vx_mps, 2) * sign(u.vx_mps);
  out.force_cog_N.y =
    -0.5 * p_.c_d * p_.air_density_kgpm3 * p_.A_m2 * std::pow(u.vy_mps, 2) * sign(u.vy_mps);
  // Minus for Fz since lift coefficient are positive
  out.force_cog_N.z =
    -0.5 * (c_l_front + c_l_rear) * p_.air_density_kgpm3 * p_.A_m2 * std::pow(u.vx_mps, 2);

  // Front and Rear Split
  double f_l_front = -0.5 * c_l_front * p_.air_density_kgpm3 * p_.A_m2 * std::pow(u.vx_mps, 2);
  double f_l_rear = -0.5 * c_l_rear * p_.air_density_kgpm3 * p_.A_m2 * std::pow(u.vx_mps, 2);

  // Output Torques
  out.torque_Nm.y = (f_l_rear * l_r) - (f_l_front * p_.l_f);
  out.torque_Nm.x = 0;
  out.torque_Nm.z = 0;

  imr_.ride_height_front_m = current_rh_front;
  imr_.ride_height_rear_m = current_rh_rear;

  return out;
}
void RideHeightAerodynamicsModel::declare_parameters(
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
    name_prefix + "l_f", &(p_.l_f), 1.6, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "l", &(p_.l), 3.0, tam::pmg::ParameterType::DOUBLE, "");

  // Ride Heights
  param_manager->declare_parameter(
    name_prefix + "static_ride_height_front_m", &(p_.static_ride_height_front_m), 0.0,
    tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "static_ride_height_rear_m", &(p_.static_ride_height_rear_m), 0.0,
    tam::pmg::ParameterType::DOUBLE, "");

  // Surface Coefficients
  param_manager->declare_parameter(
    name_prefix + "c0", &(p_.c0), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c1_F", &(p_.c1_F), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c2_F", &(p_.c2_F), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c3_F", &(p_.c3_F), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c1_R", &(p_.c1_R), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c2_R", &(p_.c2_R), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c3_R", &(p_.c3_R), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c4", &(p_.c4), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c5", &(p_.c5), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "c6", &(p_.c6), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a0", &(p_.a0), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a1_F", &(p_.a1_F), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a2_F", &(p_.a2_F), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a3_F", &(p_.a3_F), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a1_R", &(p_.a1_R), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a2_R", &(p_.a2_R), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a3_R", &(p_.a3_R), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a4", &(p_.a4), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a5", &(p_.a5), 0.0, tam::pmg::ParameterType::DOUBLE, "");
  param_manager->declare_parameter(
    name_prefix + "a6", &(p_.a6), 0.0, tam::pmg::ParameterType::DOUBLE, "");
}
}  // namespace tam::ocd::aerodynamics
