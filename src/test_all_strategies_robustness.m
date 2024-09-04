%% Model and simulation setup
close all;

%% Model params:

system_model = @inverted_pendulum;
u_limit = [u_min, u_max];
t = 0 : dt : 3;
t1 = 2; 
t2 = 3;
M = 4;
reference_cutoff_angle = 16/180*pi; %angle when reference gets set to 0
controller_cutoff = inf;

disturbance = zeros(length(t), length(x0));
disturbance(t==2.1, 3) = 10/180*pi;
disturbance(t==2.4, 3) = -10/180*pi;

noise = [0.001/10, 0.005, 0.001/10, 0.005, 0.001/2];


%% MPC_L2 + gain_scheduling
controller = @gain_scheduling_lqr;
trajectory_file = 'MPC_trajectory_norm2.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise, controller_cutoff);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" "dist"], ...
        [fig2.Name "_" "dist"]}, PATH)
end


