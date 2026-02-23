// Copyright 2026 Simon Sagmeister
#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ocd_types_cpp/types.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::ocd::interfaces
{
/// Base class for an aerodynamics model. Should be stateless
class AerodynamicsModelBase
{
public:
  AerodynamicsModelBase() = default;
  virtual ~AerodynamicsModelBase() = default;

  virtual types::AeroModelOutput evaluate(types::AeroModelInput const & input) = 0;

  /// Use reference manager and logger for all entities. So no need to store the ptr to
  /// the manager or logger somewhere
  virtual void declare_parameters(
    tam::pmg::ParamReferenceManager * param_manager, std::string name_prefix) = 0;
  virtual void register_log_signals(
    tam::tsl::ReferenceLogger * logger, std::string name_prefix) const = 0;
};
}  // namespace tam::ocd::interfaces
