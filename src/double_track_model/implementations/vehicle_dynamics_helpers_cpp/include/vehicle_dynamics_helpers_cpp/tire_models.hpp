// Copyright 2023 Simon Sagmeister
#pragma once
#include <math.h>

#include <param_manager_cpp/param_manager_non_owning.hpp>
#include <string>
#include <tum_types_cpp/common.hpp>
namespace tam::sim::helpers::tire_models
{
struct TireModelOutput
{
  double longitudinal_force_N;     // Tire force in longitudinal direction
  double lateral_force_N;          // Tire force in lateral direction
  double self_aligning_moment_Nm;  // Self aligning moment
};
struct TireModelInput
{
  double longitudinal_slip;  // Slip ratio in longitudinal direction
  double slip_angle_rad;     // Slip angle at the respective tire
  double F_z_N;              // Normal force in the tire road contact patch
};
// Linear
// =========================================================
class Linear
{
public:
  struct Parameters
  {
    double C_sx;
    double C_alpha;
  };
  TireModelOutput operator()(TireModelInput const & input, Parameters const & parameters) const;
};
namespace integration
{
void _declare_tire_model_parameters(
  std::string prefix, Linear::Parameters * param_struct,
  tam::core::ParamManagerNonOwning * param_manager);
}  // namespace integration
// MF Simple
// =========================================================
class MF_Simple
{
public:
  struct Parameters
  {
    struct Coefficients
    {
      double B, C, D, E;
    };
    Coefficients lateral, longitudinal;
  };
  TireModelOutput operator()(TireModelInput const & input, Parameters const & parameters) const;
};
namespace integration
{
void _declare_tire_model_parameters(
  std::string prefix, MF_Simple::Parameters * param_struct,
  tam::core::ParamManagerNonOwning * param_manager);
}  // namespace integration
//
// MF Simple Extended
// =========================================================
class MF_Simple_Extended
{
public:
  struct Parameters
  {
    MF_Simple::Parameters p_mf_simple;
    double FNOMIN;
    double PDY2;
  };
  TireModelOutput operator()(TireModelInput const & input, Parameters const & parameters) const;

private:
  MF_Simple mf_simple;
};
namespace integration
{
void _declare_tire_model_parameters(
  std::string prefix, MF_Simple_Extended::Parameters * param_struct,
  tam::core::ParamManagerNonOwning * param_manager);
}  // namespace integration
// MF 52
// =========================================================
class MF52
{
public:
  struct Parameters
  {
    // Long Parameters
    double PCX1, PDX1, PDX2, PDX3, PEX1, PEX2, PEX3, PEX4, PKX1, PKX2, PKX3, PHX1, PHX2, PVX1, PVX2,
      RBX1, RBX2, RBX3, RCX1, REX1, REX2, RHX1, PTX1, PTX2, PTX3;
    // Lateral Parameters
    double PCY1, PDY1, PDY2, PDY3, PEY1, PEY2, PEY3, PEY4, PKY1, PKY2, PKY3, PHY1, PHY2, PHY3, PVY1,
      PVY2, PVY3, PVY4, RBY1, RBY2, RBY3, RCY1, REY1, REY2, RHY1, RHY2, RVY1, RVY2, RVY3, RVY4,
      RVY5, RVY6, PTY1, PTY2, PEY5, PKY4, PKY5, PKY6, PKY7, PPY1, PPY2, PPY3, PPY4, PPY5, RBY4;
    // Scaling Factors
    double LFZO, LCX, LMUX, LEX, LKX, LHX, LVX, LGAX, LCY, LMUY, LEY, LKY, LKYG, LHY, LVY, LGAY,
      LTR, LRES, LGAZ, LXAL, LYKA, LVYKA, LS, LSGKP, LSGAL, LGYR, LMX, LVMX, LMY;
    // Other Params
    double FNOMIN;
  };
  TireModelOutput operator()(TireModelInput const & input, Parameters const & parameters) const;

private:
  double long_force_pure_slip(double F_z, double kappa, Parameters parameters) const;
  double lat_force_pure_slip(double F_z, double alpha, double gamma, Parameters parameters) const;
  double long_force_combined_slip(
    double F_z, double kappa, double alpha, double long_force_pure_slip, double gamma,
    Parameters parameters) const;
  double lat_force_combined_slip(
    double F_z, double kappa_long, double alpha, double lat_force_pure_slip, double gamma,
    Parameters paramaters) const;
};
namespace integration
{
void _declare_tire_model_parameters(
  std::string prefix, MF52::Parameters * param_struct,
  tam::core::ParamManagerNonOwning * param_manager);
}  // namespace integration
// Overall integration functions
// =========================================================
template <typename T>
void declare_tire_model_parameters(
  std::string prefix, tam::types::common::DataPerWheel<T> * param_struct,
  tam::core::ParamManagerNonOwning * param_manager)
{
  integration::_declare_tire_model_parameters(
    prefix + "tire.front_left.", &(param_struct->front_left), param_manager);
  integration::_declare_tire_model_parameters(
    prefix + "tire.front_right.", &(param_struct->front_right), param_manager);
  integration::_declare_tire_model_parameters(
    prefix + "tire.rear_left.", &(param_struct->rear_left), param_manager);
  integration::_declare_tire_model_parameters(
    prefix + "tire.rear_right.", &(param_struct->rear_right), param_manager);
}
}  // namespace tam::sim::helpers::tire_models
