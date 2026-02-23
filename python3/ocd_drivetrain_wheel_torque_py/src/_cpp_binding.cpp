// Copyright 2026 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>

#include "ocd_drivetrain_wheel_torque_cpp/drivetrain_direct_torque_model.hpp"

namespace py = pybind11;
using dt = tam::ocd::drivetrain::DrivetrainWheelTorqueModel;
PYBIND11_MODULE(_ocd_drivetrain_wheel_torque_cpp_py, m)
{
  py::class_<dt::DriverInputType>(m, "DriverInput")
    .def(py::init())
    .def_readwrite(
      "drivetrain_input_torque_per_wheel_Nm",
      &dt::DriverInputType::drivetrain_input_torque_per_wheel_Nm);
  py::class_<dt::FeedbackType>(m, "Feedback")
    .def(py::init())
    .def_readwrite(
      "drivetrain_input_torque_per_wheel_Nm",
      &dt::FeedbackType::drivetrain_input_torque_per_wheel_Nm);
};
