%OCD_MATLAB Experimental Open Car Dynamics MEX interface.
%
%   choices = ocd_matlab('models')
%   ocd_matlab('create', config)
%   ocd_matlab('create', config, initial_velocity_mps, time_step_s)
%   ocd_matlab('reset', initial_velocity_mps, time_step_s)
%   output = ocd_matlab('step', drivetrain_input, steering_angle_rad)
%   ocd_matlab('set_external', external_influences)
%   names = ocd_matlab('list_parameters')
%   value = ocd_matlab('get_parameter', name)
%   ocd_matlab('set_parameter', name, value)
%   info = ocd_matlab('info')
%
% CONFIG is a scalar struct with these text fields:
%   drivetrain       'fx', 'wheel_torque', or 'rwd_lsd'
%   steering          'pt1'
%   vehicle_dynamics  'single_track' or 'double_track'
%   tire              'mf52', 'linear', 'mf_simple', or
%                     'mf_simple_extended'
%   aerodynamics      'default' or 'ride_height'
%
% Every Cartesian combination is supported (48 vehicle models). Drivetrain
% input depends on the selected drivetrain:
%   fx            real double scalar longitudinal force [N]
%   wheel_torque  four-element wheel-torque vector [Nm]
%   rwd_lsd       struct with transmission_output_torque_Nm,
%                 current_engine_inertia_at_wheels_kgm2, and
%                 brake_torque_per_wheel_Nm (all fields optional, default 0)
%
% External-influence struct fields are optional: external_force_N (3),
% external_torque_Nm (3), wind_mps (3), z_height_road_m (4), and lambda_mue
% (4). Calling SET_EXTERNAL without a struct restores defaults.
%
% Scalar and vector model parameters can be read and changed by name without
% exposing the C++ parameter-manager object. Per-wheel vectors use the order
% [front_left, front_right, rear_left, rear_right].
%
% The module owns one model instance. Clear it to destroy that instance:
%
%   clear ocd_matlab
%
% This interface is experimental and has not yet been tested in MATLAB.
% See experimental/matlab/README.md for examples and current limitations.

