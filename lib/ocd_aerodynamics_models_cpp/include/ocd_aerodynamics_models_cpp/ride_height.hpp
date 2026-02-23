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
class RideHeightAerodynamicsModel : public interfaces::AerodynamicsModelBase
{
public:
  struct Parameters
  {
    double air_density_kgpm3;  // Air density
    // Aerodynamic geometry & properties
    double c_d;   // Drag coefficient
    double A_m2;  // Area of cross section

    double l_f;  // Alias front wheelbase
    double l;    // Alias Wheelbase

    // Static RH are initialized to 0
    double static_ride_height_front_m;  // static ride heights
    double static_ride_height_rear_m;

    // Surface Coefficients
    // Front Lift Coefficients
    double c0;
    double c1_F;
    double c2_F;
    double c3_F;
    double c1_R;
    double c2_R;
    double c3_R;
    double c4;
    double c5;
    double c6;
    // Rear Lift Coefficients
    double a0;
    double a1_F;
    double a2_F;
    double a3_F;
    double a1_R;
    double a2_R;
    double a3_R;
    double a4;
    double a5;
    double a6;
  } p_;
  struct
  {
    double ride_height_front_m;
    double ride_height_rear_m;
  } imr_;
  types::AeroModelOutput evaluate(types::AeroModelInput const & input) override;
  void declare_parameters(
    tam::pmg::ParamReferenceManager * param_manager,
    std::string name_prefix = "aerodynamics.") override;
  void register_log_signals(
    tam::tsl::ReferenceLogger * logger, std::string name_prefix = "aerodynamics") const override
  {
    logger->log(name_prefix + "ride_height_front_m", &imr_.ride_height_front_m);
    logger->log(name_prefix + "ride_height_rear_m", &imr_.ride_height_rear_m);
  }
};
// Check that the class fulfills the concept
// This checks if the base class was properly implemented without having to create an instance
static_assert(
  tam::ocd::interfaces::concepts::AerodynamicsModel<RideHeightAerodynamicsModel>,
  "RideHeightAerodynamicsModel does not fulfill the AerodynamicsModel concept");
}  // namespace tam::ocd::aerodynamics
