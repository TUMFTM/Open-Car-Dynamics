// Copyright 2026 Simon Sagmeister
#pragma once
#include <math.h>

#include <string>
#include <tum_types_cpp/common.hpp>

#include "ocd_tire_model_base_cpp/base_class.hpp"
#include "ocd_tire_model_base_cpp/concept.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
namespace tam::ocd::tire_models
{
class MF52 : public tam::ocd::interfaces::TireModelBase
{
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
  } p_;

public:
  types::TireModelOutput evaluate(types::TireModelInput const & input) override;
  void declare_parameters(
    tam::pmg::ParamReferenceManager * param_manager, std::string name_prefix) override;
  void register_log_signals(tam::tsl::ReferenceLogger *, std::string = "") const override {}

private:
  double long_force_pure_slip(double F_z, double kappa) const;
  double lat_force_pure_slip(double F_z, double alpha, double gamma) const;
  double long_force_combined_slip(
    double F_z, double kappa, double alpha, double long_force_pure_slip, double gamma) const;
  double lat_force_combined_slip(
    double F_z, double kappa_long, double alpha, double lat_force_pure_slip, double gamma) const;
};
// Check that the class fulfills the concept
// This checks if the base class was properly implemented without having to create an instance
static_assert(
  tam::ocd::interfaces::concepts::TireModel<MF52>, "MF52 does not fulfill the TireModel concept");
}  // namespace tam::ocd::tire_models
