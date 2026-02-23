from ocd_vehicle_models_py import VehicleFactory
from ocd_vehicle_models_py import (
    DrivetrainType,
    SteeringActuatorType,
    VehicleDynamicsType,
    TireType,
    AerodynamicsType,
)

veh = VehicleFactory.create_vehicle(
        drivetrain=DrivetrainType.RWD_LSD,
        steering=SteeringActuatorType.PT1,
        vdm=VehicleDynamicsType.DOUBLE_TRACK,
        tire=TireType.MF52,
        aero=AerodynamicsType.DEFAULT,
)

driver_input = veh.construct_drivetrain_input() # Get the correct input struct for the drivetrain model
external_influences = veh.construct_external_influences() # Get the correct input struct for external influences (e.g. road slope, wind, etc.) - this is optional and can be left at default values if not needed
steering_input = veh.construct_steering_input() # Get the correct input struct for the steering model - this is optional and can be left at default values if not needed


# Load params 
# --------------------
# Get the params for your model or alternatively load them from a json file
param_dict = veh.get_param_manager().get_parameters_as_dict()
# You can modify the param dict as needed, for example:
# param_dict["some_param"] = new_value
# Then set the modified dict back to the param manager
veh.get_param_manager().set_parameters_from_dict(param_dict)


# Modify the inputs as needed 
# ----------
# ....
# ----------

# Set the inputs 
veh.set_drivetrain_input(driver_input)
veh.set_external_influences(external_influences)
veh.set_steering_input(steering_input)

# Once all the inputs have been set, call step() to run the simulation for one time step
veh.step()

# Repeat this loop as many times as needed, modifying the inputs and calling step() to advance the simulation

# Get the outputs
drivetrain_feedback = veh.get_drivetrain_feedback()
steering_actuator_feedback = veh.get_steering_actuator_feedback()
vehicle_model_output = veh.get_vehicle_model_output()
# Get the internal log signals for the model as dictionary
log_signals = veh.get_logger().get_data()