# Copyright 2026 Simon Sagmeister
from enum import Enum


class DrivetrainType(Enum):
    """Available drivetrain models."""

    WHEEL_TORQUE = "DrivetrainWheelTorqueModel"
    FX = "DrivetrainFxModel"
    RWD_LSD = "DrivetrainModel_RWD_LSD"


class SteeringActuatorType(Enum):
    """Available steering actuator models."""

    PT1 = "PT1SteeringActuatorModel"


class VehicleDynamicsType(Enum):
    """Available vehicle dynamics models."""

    SINGLE_TRACK = "VehicleDynamicsSingleTrackModel"
    DOUBLE_TRACK = "VehicleDynamicsDoubleTrackModel"


class AerodynamicsType(Enum):
    """Available aerodynamics models."""

    DEFAULT = "DefaultAerodynamicsModel"
    RIDE_HEIGHT = "RideHeightAerodynamicsModel"


class TireType(Enum):
    """Available tire models."""

    MF52 = "MF52"
    LINEAR = "Linear"
    MF_SIMPLE = "MF_Simple"
    MF_SIMPLE_EXTENDED = "MF_Simple_Extended"
