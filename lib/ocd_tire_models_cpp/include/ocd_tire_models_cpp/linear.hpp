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
// Linear
// =========================================================
class Linear : public tam::ocd::interfaces::TireModelBase
{
  struct Parameters
  {
    double C_sx;
    double C_alpha;
  } p_;

public:
  types::TireModelOutput evaluate(types::TireModelInput const & input) override;
  void declare_parameters(
    tam::pmg::ParamReferenceManager * param_manager, std::string name_prefix) override;
  void register_log_signals(tam::tsl::ReferenceLogger *, std::string = "") const override {}
};
// Check that the class fulfills the concept
// This checks if the base class was properly implemented without having to create an instance
static_assert(
  tam::ocd::interfaces::concepts::TireModel<Linear>,
  "Linear does not fulfill the TireModel concept");
}  // namespace tam::ocd::tire_models
