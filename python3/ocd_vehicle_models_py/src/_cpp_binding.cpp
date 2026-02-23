// Copyright 2026 Simon Sagmeister
#include <pybind11/pybind11.h>

#include "ocd_vehicle_models_py/bind_macro.hpp"

// Create this code using the helper script under tools of this package
namespace py = pybind11;
PYBIND11_MODULE(_ocd_vehicle_models_py, m)
{
  BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF52__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__Linear__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple_Extended__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF52__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__Linear__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple_Extended__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF52__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__Linear__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple_Extended__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF52__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__Linear__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainWheelTorqueModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple_Extended__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF52__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__Linear__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple_Extended__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF52__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__Linear__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple_Extended__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF52__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__Linear__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple_Extended__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF52__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__Linear__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainFxModel__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple_Extended__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainFxModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF52__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__Linear__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple_Extended__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF52__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__Linear__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsSingleTrackModel__MF_Simple_Extended__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF52__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__Linear__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple_Extended__DefaultAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::DefaultAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF52__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF52,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__Linear__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::Linear,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);

BIND_OCD_VEHICLE_MODEL(
  m,
  OCD_Vehicle_DrivetrainModel_RWD_LSD__PT1SteeringActuatorModel__VehicleDynamicsDoubleTrackModel__MF_Simple_Extended__RideHeightAerodynamicsModel, // NOLINT
  tam::ocd::drivetrain::DrivetrainModel_RWD_LSD,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel,
  tam::ocd::tire_models::MF_Simple_Extended,
  tam::ocd::aerodynamics::RideHeightAerodynamicsModel);
};
