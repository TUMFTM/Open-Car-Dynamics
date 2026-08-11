# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.1.1] - 2026-08-11
### Added
- Configurable output rate for `VehicleModelNode` via the new ROS 2 parameter `output_rate_hz` (default 100 Hz), replacing the previously hard-coded 10 ms output timer. Non-positive values are rejected with an error log and fall back to 100 Hz.
- Time-skew monitoring and compensation in the model update callback: the node tracks the accumulated skew between its clock and the integrated simulation time, and performs a second integration step within a cycle to catch up whenever the skew exceeds one nominal step.
- Additional logged diagnostic signals for the model update callback: `node/time_between_callbacks_us`, `node/accumulated_skew_us`, `node/num_steps`, `node/double_step_count`, and `node/filtered_double_step_ratio`.

### Changed
- The communication handler delay timers are now advanced with the nominal step size once per executed integration step, instead of once per callback.

## [2.1.0] - 2026-07-10
### Added
- Support for supplying environmental wind conditions to the vehicle.
- Parametric external influence aggregator (`ocd_external_influence_aggregator_cpp`).
- New message package `ocd_interfaces` for publishing and subscribing to the external influence interfaces.
- `COLCON_IGNORE` for the plain CMake build directory, so the repository can be built inside a ROS 2 workspace.

### Changed
- The `ExternalInfluence` message now uses a wrench message directly, simplifying the combination of multiple influence sources.
- Cleaned up the initialization of non-parameter variables.

### Fixed
- Logging and variable initialization for the drivetrains.
- Submodule paths for the `tam__common` dependencies in the CMake build.

## [2.0.0] - 2026-02-23
### Added
- Use templating to combine the models at compile time, enabling building vehicles from a set of submodels.
- Configurable input/feedback types for models.
- Templated tire and aerodynamics models for improved double-track dynamics.
- Consideration of camber and toe in double-track modeling.
- Single-track vehicle dynamics model.
- Ability to build the library using plain CMake.
- Generic ROS 2 bindings utilizing communications handlers.
- Additional drivetrain implementations.
- Python 3 bindings.
- Vehicle Factory to configure vehicles from Python.
- Loading parameters from JSON for the Python bindings.

### Changed
- Complete rework of the library (breaking changes / major refactor).
- Updated README.
- Improved computational efficiency (≈10x speedup).
- Switched to the public `tam__common` dependency instead of copying packages.
- Major version bump to 2.0.0.

## [1.0.0] - Initial Release
### Added
- Initial release of Open Car Dynamics.
