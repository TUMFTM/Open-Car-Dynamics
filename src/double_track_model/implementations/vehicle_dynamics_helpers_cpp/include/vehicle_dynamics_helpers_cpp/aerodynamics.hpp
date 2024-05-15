// Copyright 2023 Simon Sagmeister
#pragma once

#include "tum_types_cpp/common.hpp"
namespace tam::sim::helpers::aerodynamics
{

constexpr double air_density_kgpm3 = 1.225;
struct AeroModelOutput
{
  // Aerodynamic forces acting on the cog of the vehicle
  tam::types::common::Vector3D<double> force_cog_N;
  // Aerodynamic torque on the vehicle body
  tam::types::common::Vector3D<double> torque_Nm;
};
struct Parameters
{
  double c_d;   // Drag coefficient
  double c_l;   // Lift coefficient
  double A_m2;  // Area of cross section
  // Distance from the center of aero forces to the cog
  // Positive values indicate a front heavy aero balance
  double d_aero_center_cog_m;
};
AeroModelOutput aero_model(double vx, double vy, Parameters p);
}  // namespace tam::sim::helpers::aerodynamics
