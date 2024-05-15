// Copyright 2023 Simon Sagmeister
#include "vehicle_dynamics_helpers_cpp/aerodynamics.hpp"
namespace tam::sim::helpers::aerodynamics
{
AeroModelOutput aero_model(double vx, double vy, Parameters p)
{
  AeroModelOutput out;
  auto sign = [](double x) {
    if (x < 0) {
      return -1;
    } else {
      return 1;
    }
  };

  out.force_cog_N.x = -0.5 * p.c_d * air_density_kgpm3 * p.A_m2 * std::pow(vx, 2) * sign(vx);
  // To be precise the area of calculating the drag here is different for lateral movement
  // but this is neglected here
  out.force_cog_N.y = -0.5 * p.c_d * air_density_kgpm3 * p.A_m2 * std::pow(vy, 2) * sign(vy);
  out.force_cog_N.z = 0.5 * p.c_l * air_density_kgpm3 * p.A_m2 * std::pow(vx, 2);

  // Aero Balance
  out.torque_Nm.y = -out.force_cog_N.z * p.d_aero_center_cog_m;

  // Zero for everything else
  out.torque_Nm.x = 0;
  out.torque_Nm.z = 0;

  return out;
}
}  // namespace tam::sim::helpers::aerodynamics
