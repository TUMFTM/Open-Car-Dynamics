from ocd_vehicle_models_py.submodel_choices import *
from ocd_vehicle_models_py.util import get_cpp_class_name


def main():

    for drivetrain in DrivetrainType:
        for steering in SteeringActuatorType:
            for vdm in VehicleDynamicsType:
                for aero in AerodynamicsType:
                    for tire in TireType:
                        name = get_cpp_class_name(
                            drivetrain, steering, vdm, tire, aero
                        )
                        print("BIND_OCD_VEHICLE_MODEL(")
                        print("  m,")
                        print(f"  {name}, // NOLINT")
                        print(f"  tam::ocd::drivetrain::{drivetrain.value},")
                        print(f"  tam::ocd::steering_actuator::{steering.value},")
                        print(f"  tam::ocd::vehicle_dynamics::{vdm.value},")
                        print(f"  tam::ocd::tire_models::{tire.value},")
                        print(f"  tam::ocd::aerodynamics::{aero.value});\n")


if __name__ == "__main__":
    main()
