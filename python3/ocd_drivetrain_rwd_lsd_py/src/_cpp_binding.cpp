// Copyright 2026 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>

#include "ocd_drivetrain_rwd_lsd_cpp/drivetrain_rwd_lsd_model.hpp"

namespace py = pybind11;
using dt = tam::ocd::drivetrain::DrivetrainModel_RWD_LSD;
PYBIND11_MODULE(_ocd_drivetrain_rwd_lsd_cpp_py, m)
{
  py::class_<dt::DriverInputType>(m, "DriverInput")
    .def(py::init())
    .def_readwrite(
      "transmission_output_torque_Nm", &dt::DriverInputType::transmission_output_torque_Nm)
    .def_readwrite(
      "current_engine_inertia_at_wheels_kgm2",
      &dt::DriverInputType::current_engine_inertia_at_wheels_kgm2)
    .def_readwrite("brake_torque_per_wheel_Nm", &dt::DriverInputType::brake_torque_per_wheel_Nm);
};
