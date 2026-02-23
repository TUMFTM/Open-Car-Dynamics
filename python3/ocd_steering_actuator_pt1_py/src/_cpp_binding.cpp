// Copyright 2026 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>

#include "ocd_steering_actuator_pt1_cpp/steering_actuator_pt1_model.hpp"

namespace py = pybind11;
using sa = tam::ocd::steering_actuator::PT1SteeringActuatorModel;
PYBIND11_MODULE(_ocd_steering_actuator_pt1_cpp_py, m)
{
  py::class_<sa::DriverInputType>(m, "DriverInput")
    .def(py::init())
    .def_readwrite(
      "steering_angle_rad",
      &sa::DriverInputType::steering_angle_rad);
  py::class_<sa::FeedbackType>(m, "Feedback")
    .def(py::init())
    .def_readwrite(
      "steering_angle_rad",
      &sa::FeedbackType::steering_angle_rad);
};
