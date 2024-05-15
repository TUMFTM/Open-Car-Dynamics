// Copyright 2023 Simon Sagmeister
#pragma once
#include <map>
#include <string>
#include <vector>

#include "param_manager_cpp/param_manager_base.hpp"
#include "tum_sim_types_cpp/types.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::interfaces
{
class VehicleModelBase
{
public:
  VehicleModelBase() = default;
  virtual ~VehicleModelBase() = default;

  // Model step function
  virtual void step() = 0;

  // Setter
  virtual void set_driver_input(const tam::types::DriverInput & input) = 0;
  virtual void set_external_influences(const tam::types::ExternalInfluences & input) = 0;

  // Vehicle dynamic state:
  virtual tam::types::VehicleModelOutput get_output() const = 0;

  // Debug Output
  virtual tam::types::common::TUMDebugContainer::SharedPtr get_debug_out() const = 0;

  // Parameter handling
  virtual ParamManagerBase * get_param_manager() = 0;

  virtual void reset() = 0;
};
}  // namespace tam::interfaces
