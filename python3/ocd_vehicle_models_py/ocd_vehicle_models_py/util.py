# Copyright 2026 Simon Sagmeister
from .submodel_choices import (
    DrivetrainType,
    SteeringActuatorType,
    VehicleDynamicsType,
    TireType,
    AerodynamicsType,
)


def get_cpp_class_name(
    drivetrain: DrivetrainType,
    steering: SteeringActuatorType,
    vdm: VehicleDynamicsType,
    tire: TireType,
    aero: AerodynamicsType,
):
    return f"OCD_Vehicle_{drivetrain.value}__{steering.value}__{vdm.value}__{tire.value}__{aero.value}"


def print_supported_vehicle_variants() -> None:
    """Print the supported options for each submodel type in a readable format.

    Example output:
      Drivetrain:
        - RWD (rwd)
        - FWD (fwd)
    """
    categories = [
        ("Drivetrain", DrivetrainType),
        ("Steering actuator", SteeringActuatorType),
        ("Vehicle dynamics", VehicleDynamicsType),
        ("Tire", TireType),
        ("Aerodynamics", AerodynamicsType),
    ]

    print("Supported vehicle variant options:")
    for title, enum in categories:
        print(f"{title}:")
        for member in enum:
            print(f"  - {member.name} ({member.value})")
        print()
