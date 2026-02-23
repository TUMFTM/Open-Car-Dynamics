# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
