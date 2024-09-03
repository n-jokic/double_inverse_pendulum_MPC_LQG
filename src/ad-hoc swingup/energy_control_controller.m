%% Simulation setup:
controller = @energy_control;
system_model = @inverted_pendulum;
measurement_model = @inverted_pendulum_measurement;


t = 0:dt:3;
M = 1;

reference_cutoff_angle = 0;
controller_cutoff = inf;
x0 = [0.0; 0;pi; 0; 0];
state_reference = zeros(length(t), length(x0));
control_reference = zeros(length(t), length(u0));
u_limit = [u_min, u_max];

u0 = 5;
disturbance = zeros(length(t), length(x0)); 
noise = [0.01, 0.01, 0.01, 0.01, 0.01]*0;
%% Simulation:
[states, control, states_measured, state_reference, control_reference] = ...
simulate_system(system_model, controller, x0, u0, state_reference, ...
control_reference, disturbance, noise, t, M, ...
u_limit, reference_cutoff_angle, controller_cutoff);
%% Plotting: 
%plot_simulation_result(x, u, disturbance, t, [-5; 5]);
x_ref = states;
u_ref = control;

if SAVE_TRAJECTORY
loc = fullfile(cd, 'ad-hoc swingup');
save(fullfile(loc, 'EnergyControl_trajectory'), 'x_ref', 'u_ref');
end