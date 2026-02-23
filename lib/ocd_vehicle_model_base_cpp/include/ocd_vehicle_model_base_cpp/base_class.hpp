// Copyright 2026 Simon Sagmeister
#pragma once
#include <map>
#include <string>
#include <vector>

#include "ocd_types_cpp/types.hpp"
#include "ocd_vehicle_model_base_cpp/conditional_auxiliary_input.hpp"
#include "param_management_cpp/base.hpp"
#include "tsl_logger_cpp/base.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::ocd::interfaces
{
template <
  typename DrivetrainDriverInputT, typename DrivetrainFeedbackT,
  typename SteeringActuatorDriverInputT, typename SteeringActuatorFeedbackT,
  typename DrivetrainAuxiliaryInputT, typename SteeringActuatorAuxiliaryInputT>
class VehicleModelBase
// Inherit from helper structs that add functions only if the auxiliary input types are not void
: public auxiliary::conditional_compilation::AuxiliaryInputHelperDT_VEH<DrivetrainAuxiliaryInputT>,
  public auxiliary::conditional_compilation::AuxiliaryInputHelperSA_VEH<
    SteeringActuatorAuxiliaryInputT>
{
public:
  // Declare the types transpararently so they can be accessed from outside
  using DrivetrainDriverInputType = DrivetrainDriverInputT;
  using DrivetrainFeedbackType = DrivetrainFeedbackT;
  using SteeringActuatorDriverInputType = SteeringActuatorDriverInputT;
  using SteeringActuatorFeedbackType = SteeringActuatorFeedbackT;
  using DrivetrainAuxiliaryInputType = DrivetrainAuxiliaryInputT;
  using SteeringActuatorAuxiliaryInputType = SteeringActuatorAuxiliaryInputT;

public:
  VehicleModelBase() = default;
  virtual ~VehicleModelBase() = default;

  // Model step function
  virtual void step() = 0;

  // drivetrain block
  virtual void set_drivetrain_input(const DrivetrainDriverInputT & input) = 0;
  virtual DrivetrainFeedbackT get_drivetrain_feedback() const = 0;

  // steering actuator block
  virtual void set_steering_input(const SteeringActuatorDriverInputT & input) = 0;
  virtual SteeringActuatorFeedbackT get_steering_actuator_feedback() const = 0;
  // vehicle dynamics block
  virtual void set_external_influences(const types::ExternalInfluences & input) = 0;
  // clang-format off
  // virtual void set_auxiliary_input_steering_actuator(const SteeringActuatorAuxiliaryInputT & value) = 0; | INHERITED FROM HELPER // NOLINT
  // virtual void set_auxiliary_input_drivetrain(const DrivetrainAuxiliaryInputT & value) = 0; | INHERITED FROM HELPER  // NOLINT
  // clang-format on

  // output block
  virtual types::VehicleModelOutput get_vehicle_model_output() const = 0;

  // Debug Output
  virtual tam::tsl::LoggerAccessInterface::SharedPtr get_logger() const = 0;

  // Parameter handling
  virtual pmg::MgmtInterface::SharedPtr get_param_manager() = 0;

  virtual void reset() = 0;

  // Functions to set auxiliary inputs are inherited from the respective base classes
  // if the input types are not void.
};
}  // namespace tam::ocd::interfaces
