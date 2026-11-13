# Experimental MATLAB binding

> [!WARNING]
> This binding remains experimental and is not part of the supported Open Car Dynamics API. It has been successfully built and run on Ubuntu 22.04 with MATLAB R2026a, but other systems and MATLAB releases have not been validated. If you try it, please report successes, build failures, MATLAB-version details, and proposed improvements in the [Open Car Dynamics issue tracker](https://github.com/TUMFTM/Open-Car-Dynamics/issues). Community feedback is explicitly welcome.
>
> The initial implementation of parts of this interface was AI-assisted.

This directory contains a deliberately small MEX interface with full model-factory coverage. It exchanges only MATLAB scalars, vectors, strings, cells, and structs; C++ objects, the parameter manager, and the logger are not exposed.

The factory exposes the same complete Cartesian product as the Python `VehicleFactory`:

| Component | Available models |
| --- | --- |
| Drivetrain | force input, per-wheel torque, RWD with LSD |
| Steering actuator | PT1 |
| Vehicle dynamics | single track, double track |
| Tire | MF52, linear, simple Magic Formula, extended simple Magic Formula |
| Aerodynamics | default, ride-height dependent |

All `3 × 1 × 2 × 4 × 2 = 48` combinations are constructible and are compiled and stepped by the MATLAB-independent smoke test. The models start with the library's generic defaults. Scalar and vector parameters can be listed, read, and changed by name through a simplified command interface.

## Tested configuration

| Operating system | MATLAB | Result |
| --- | --- | --- |
| Ubuntu 22.04 | R2026a | MEX module built and the binding was run successfully |

This is currently the only reported MATLAB runtime test. It does not establish compatibility with other MATLAB releases, compilers, or operating systems.

## Requirements

- MATLAB with MEX development files and a C++ compiler supported by that MATLAB release (runtime-tested with R2026a only);
- CMake 3.18 or newer;
- a C++20 compiler;
- Eigen3 and Boost; and
- all Open Car Dynamics git submodules (`git submodule update --init --recursive`).

The build uses CMake's standard `FindMatlab` module and builds the standalone Open Car Dynamics C++ library in the same build tree. ROS 2 and colcon are not required.

## Build

From the Open Car Dynamics repository root:

```bash
cmake -S experimental/matlab -B build/matlab -DCMAKE_BUILD_TYPE=Release
cmake --build build/matlab --target ocd_matlab --parallel
```

If CMake cannot locate MATLAB, provide its installation root:

```bash
cmake -S experimental/matlab -B build/matlab \
  -DCMAKE_BUILD_TYPE=Release \
  -DMatlab_ROOT_DIR=/path/to/MATLAB
```

The MEX file is written to `build/matlab/matlab`. Add that directory to the MATLAB path:

```matlab
addpath('/absolute/path/to/open-car-dynamics/build/matlab/matlab');
```

The command reference is maintained in [`src/ocd_matlab.m`](src/ocd_matlab.m).

On Linux, start MATLAB from an environment in which the generated Open Car Dynamics shared library can be found. The CMake build normally records a build-tree runtime path. If your system strips it, add `build/matlab/open_car_dynamics` to `LD_LIBRARY_PATH` before starting MATLAB.

## Interface

The MEX module keeps one model instance for the current MATLAB process:

```matlab
choices = ocd_matlab('models');
assert(choices.combination_count == 48);

config = struct( ...
    'drivetrain', 'fx', ...
    'steering', 'pt1', ...
    'vehicle_dynamics', 'double_track', ...
    'tire', 'mf52', ...
    'aerodynamics', 'default');

% Create at 20 m/s with a 1 ms integration step.
ocd_matlab('create', config, 20.0, 0.001);

info = ocd_matlab('info');

% The selected force-input drivetrain takes a scalar force in newtons.
output = ocd_matlab('step', 500.0, 0.01);

% Destroy the model instance and unload the MEX module.
clear ocd_matlab
```

`create` requires all five text fields shown above. The initial velocity and time-step arguments are optional and default to `0 m/s` and `0.0008 s`. `reset` preserves the selected composition and synchronizes all wheel speeds with its requested initial longitudinal velocity.

### Drivetrain inputs

The second argument to `step` depends only on the selected drivetrain:

| Drivetrain | MATLAB input |
| --- | --- |
| `fx` | Real scalar longitudinal force in N; positive drives and negative brakes |
| `wheel_torque` | Four-element vector of drivetrain torque per wheel in Nm |
| `rwd_lsd` | Scalar struct described below |

The RWD-LSD struct accepts `transmission_output_torque_Nm`, `current_engine_inertia_at_wheels_kgm2`, and the four-element `brake_torque_per_wheel_Nm`. Its fields are optional and default to zero.

### External influences and parameters

External-influence fields are optional. Omitted values use their default (zero, except `lambda_mue`, which defaults to one):

```matlab
external = struct('wind_mps', [5, 0, 0], ...
                  'lambda_mue', [1, 1, 0.9, 0.9]);
ocd_matlab('set_external', external);
ocd_matlab('set_external');  % Restore defaults.
```

The binding exposes current scalar and vector model parameters by name without exposing the C++ parameter-manager object:

```matlab
parameter_names = ocd_matlab('list_parameters');
mass_kg = ocd_matlab('get_parameter', ...
    'vehicle_dynamics_double_track.mass_vehicle_kg');
ocd_matlab('set_parameter', ...
    'vehicle_dynamics_double_track.mass_vehicle_kg', 900.0);
```

### Outputs

`step` returns a scalar struct containing time, vehicle motion, wheel speeds, steering angles, tire slips, and tire forces. Three-component vectors use `[x, y, z]`. Per-wheel vectors always use:

```text
[front_left, front_right, rear_left, rear_right]
```

Commands, choices, and parameter names accept MATLAB character vectors and scalar strings. Numeric inputs must be finite, real doubles with the documented element counts.

See [examples/simple_simulation.m](examples/simple_simulation.m) for a complete minimal loop.

## Validation without MATLAB

The adapter and gateway can be built without a MATLAB installation. The test constructs and steps all 48 combinations, exercises all three drivetrain input contracts, applies external influences, round-trips scalar and vector parameters, validates initial wheel-speed synchronization and output conversions, and runs the default model for one simulated second. The exact gateway also compiles against a minimal MEX API declaration with warnings treated as errors:

```bash
cmake -S experimental/matlab -B build/matlab-check \
  -DOCD_MATLAB_BUILD_MEX=OFF \
  -DOCD_MATLAB_BUILD_TESTS=ON
cmake --build build/matlab-check --parallel
ctest --test-dir build/matlab-check --output-on-failure
```

This MATLAB-independent test cannot validate MATLAB's loader, ABI, or actual MEX runtime behavior. Those parts have been exercised successfully on the tested configuration above, but broader runtime and compiler coverage is still needed before the binding can be considered supported.

