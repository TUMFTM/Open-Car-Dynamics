% Minimal experimental Open Car Dynamics simulation.

choices = ocd_matlab('models');
assert(choices.combination_count == 48);

config = struct( ...
    'drivetrain', 'fx', ...
    'steering', 'pt1', ...
    'vehicle_dynamics', 'double_track', ...
    'tire', 'mf52', ...
    'aerodynamics', 'default');

initial_velocity_mps = 20.0;
time_step_s = 0.001;
ocd_matlab('create', config, initial_velocity_mps, time_step_s);

duration_s = 1.0;
number_of_steps = round(duration_s / time_step_s);
speed_mps = zeros(number_of_steps, 1);

for index = 1:number_of_steps
    longitudinal_force_N = 500.0;
    steering_angle_rad = 0.01;
    output = ocd_matlab('step', longitudinal_force_N, steering_angle_rad);
    speed_mps(index) = output.velocity_mps(1);
end

time_s = (1:number_of_steps)' * time_step_s;
plot(time_s, speed_mps);
xlabel('Time [s]');
ylabel('Longitudinal speed [m/s]');
grid on;

