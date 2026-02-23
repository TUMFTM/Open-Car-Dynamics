#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>

#include "ocd_types_cpp/types.hpp"

namespace py = pybind11;
namespace typ = tam::ocd::types;
PYBIND11_MODULE(_ocd_types_cpp_py, m)
{
  py::class_<typ::VehicleDynamicsModelOutput>(m, "VehicleDynamicsModelOutput")
    .def(py::init())
    .def_readwrite("position_m", &typ::VehicleDynamicsModelOutput::position_m)
    .def_readwrite("velocity_mps", &typ::VehicleDynamicsModelOutput::velocity_mps)
    .def_readwrite("acceleration_mps2", &typ::VehicleDynamicsModelOutput::acceleration_mps2)
    .def_readwrite("orientation_rad", &typ::VehicleDynamicsModelOutput::orientation_rad)
    .def_readwrite(
      "angular_velocity_radps", &typ::VehicleDynamicsModelOutput::angular_velocity_radps)
    .def_readwrite(
      "angular_acceleration_radps2", &typ::VehicleDynamicsModelOutput::angular_acceleration_radps2)
    .def_readwrite(
      "tire_longitudinal_slip_per_wheel",
      &typ::VehicleDynamicsModelOutput::tire_longitudinal_slip_per_wheel)
    .def_readwrite(
      "tire_slip_angle_per_wheel_rad",
      &typ::VehicleDynamicsModelOutput::tire_slip_angle_per_wheel_rad)
    .def_readwrite(
      "longitudinal_tire_force_tire_frame_per_wheel_N",
      &typ::VehicleDynamicsModelOutput::longitudinal_tire_force_tire_frame_per_wheel_N)
    .def_readwrite(
      "lateral_tire_force_tire_frame_per_wheel_N",
      &typ::VehicleDynamicsModelOutput::lateral_tire_force_tire_frame_per_wheel_N)
    .def_readwrite(
      "vertical_tire_force_per_wheel_N",
      &typ::VehicleDynamicsModelOutput::vertical_tire_force_per_wheel_N);
  py::class_<typ::ExternalInfluences>(m, "ExternalInfluences")
    .def(py::init())
    .def_readwrite("external_force_N", &typ::ExternalInfluences::external_force_N)
    .def_readwrite("external_torque_Nm", &typ::ExternalInfluences::external_torque_Nm)
    .def_readwrite("z_height_road_m", &typ::ExternalInfluences::z_height_road_m)
    .def_readwrite("lambda_mue", &typ::ExternalInfluences::lambda_mue);
  py::class_<typ::VehicleModelOutput>(m, "VehicleModelOutput")
    .def(py::init())
    .def_readwrite("vehicle_dynamics_output", &typ::VehicleModelOutput::vehicle_dynamics_output)
    .def_readwrite("wheel_speeds_radps", &typ::VehicleModelOutput::wheel_speeds_radps)
    .def_readwrite(
      "drivetrain_load_torque_per_wheel_Nm",
      &typ::VehicleModelOutput::drivetrain_load_torque_per_wheel_Nm)
    .def_readwrite(
      "steering_angle_per_wheel_rad", &typ::VehicleModelOutput::steering_angle_per_wheel_rad)
    .def_readwrite(
      "steering_load_torque_per_wheel_Nm",
      &typ::VehicleModelOutput::steering_load_torque_per_wheel_Nm);
};
