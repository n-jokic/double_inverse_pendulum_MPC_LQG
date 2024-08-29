%% Model and simulation setup
close all;

system_model = @inverted_pendulum;
u_limit = [u_min, u_max];
t = 0 : dt : 3.5;
t1 = 0; 
t2 = 3.5;
M = 2;
reference_cutoff_angle = 5/180*pi; %angle when reference gets set to 0

disturbance = zeros(length(t), length(x0));
disturbance(t==2.5, 3) = 10/180*pi;
disturbance(t==3, 3) = -10/180*pi;

noise = [0.001/10, 0.005, 0.001/10, 0.005, 0.001/2];


%% MPC_L1 + gain_scheduling
controller = @gain_scheduling_lqr;
trajectory_file = 'MPC_trajectory_norm1.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4)], ...
        [fig2.Name "_" trajectory_file(1:end-4)]}, PATH)
end


%% MPC_L2 + gain_scheduling
controller = @gain_scheduling_lqr;
trajectory_file = 'MPC_trajectory_norm2.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4)], ...
        [fig2.Name "_" trajectory_file(1:end-4)]}, PATH)
end



%% Energy control + gain_scheduling
controller = @gain_scheduling_lqr;
trajectory_file = 'EnergyControl_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4)], ...
        [fig2.Name "_" trajectory_file(1:end-4)]}, PATH)
end



%% EnergyShaping_trajectory + gain_scheduling
controller = @gain_scheduling_lqr;
trajectory_file = 'EnergyShaping_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4)], ...
        [fig2.Name "_" trajectory_file(1:end-4)]}, PATH)
end



%% ExpControl_trajectory + gain_scheduling
controller = @gain_scheduling_lqr;
trajectory_file = 'ExpControl_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4)], ...
        [fig2.Name "_" trajectory_file(1:end-4)]}, PATH)
end



%% Energy control + global_lqr
controller = @gain_scheduling_lqr;
trajectory_file = 'EnergyControl_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4)], ...
        [fig2.Name "_" trajectory_file(1:end-4)]}, PATH)
end



%% EnergyShaping_trajectory + global_lqr
controller = @global_LQR;
trajectory_file = 'EnergyShaping_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4)], ...
        [fig2.Name "_" trajectory_file(1:end-4)]}, PATH)
end



%% ExpControl_trajectory + global_lqr
controller = @global_LQR;
trajectory_file = 'ExpControl_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4)], ...
        [fig2.Name "_" trajectory_file(1:end-4)]}, PATH)
end



