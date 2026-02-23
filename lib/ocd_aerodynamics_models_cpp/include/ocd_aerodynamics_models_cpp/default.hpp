// Copyright 2026 Simon Sagmeister
#pragma once

#include <limits>
#include <memory>
#include <string>
#include <tum_types_cpp/common.hpp>
#include <vector>

#include "ocd_aerodynamics_model_base_cpp/base_class.hpp"
#include "ocd_aerodynamics_model_base_cpp/concept.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
namespace tam::ocd::aerodynamics
{
class DefaultAerodynamicsModel : public interfaces::AerodynamicsModelBase
{
public:
  struct Parameters
  {
    double air_density_kgpm3;  // Air density

    double c_d;   // Drag coefficient
    double c_l;   // Lift coefficient
    double A_m2;  // Area of cross section
    // Distance from the center of aero forces to the cog
    // Positive values indicate a front heavy aero balance
    double d_aero_center_cog_m;

    // Make lift coefficient velocity dependent
    std::vector<double> c_l_velocity_scaling__vel_mps;
    std::vector<double> c_l_velocity_scaling__delta_c_l;

    // Pitch correction since pitch moves the center of aero forces
    std::vector<double> pitch_aero_center_translation__pitch_rad;
    std::vector<double> pitch_aero_center_translation__aero_center_translation_m;
  } p_;
  types::AeroModelOutput evaluate(types::AeroModelInput const & input) override;
  void declare_parameters(
    tam::pmg::ParamReferenceManager * param_manager,
    std::string name_prefix = "aerodynamics.") override;
  void register_log_signals(
    tam::tsl::ReferenceLogger *, std::string = "aerodynamics") const override
  {
  }
};
// Check that the class fulfills the concept
// This checks if the base class was properly implemented without having to create an instance
static_assert(
  tam::ocd::interfaces::concepts::AerodynamicsModel<DefaultAerodynamicsModel>,
  "DefaultAerodynamicsModel does not fulfill the AerodynamicsModel concept");
}  // namespace tam::ocd::aerodynamics
