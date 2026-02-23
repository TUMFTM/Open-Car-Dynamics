from . import _ocd_vehicle_models_py as _cpp
from .submodel_choices import (
    DrivetrainType,
    SteeringActuatorType,
    VehicleDynamicsType,
    TireType,
    AerodynamicsType,
)
from .util import get_cpp_class_name


class VehicleFactory:

    @staticmethod
    def create_vehicle(
        drivetrain: DrivetrainType = DrivetrainType.FX,
        steering: SteeringActuatorType = SteeringActuatorType.PT1,
        vdm: VehicleDynamicsType = VehicleDynamicsType.DOUBLE_TRACK,
        tire: TireType = TireType.MF52,
        aero: AerodynamicsType = AerodynamicsType.DEFAULT,
    ):
        class_name = get_cpp_class_name(
            drivetrain,
            steering,
            vdm,
            tire,
            aero,
        )
        vehicle_class = getattr(_cpp, class_name)
        return vehicle_class()
