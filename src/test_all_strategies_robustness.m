%% Model and simulation setup
close all;

%% Model params:
Le1 = 229/1000; % Lenght of the horizontal arm [m]
J0 = 87.04/1000; % Moment of inertia at the base joint of the horizontal arm and pendulum [kg*m^2]
Ka1 = 1.6/1000; % Friction coeficient between base and the horizontal arm [N*m*s]
m2 = 311/1000; % Mass of the pendulum [kg]
Lcm2 = 406/1000; % Distance from axis of rotation to centre of mass of the pendulum[m]
J2 = 28.39/1000; % Moment of inertia at the joint of the pendulum[kg*m^2]
Ka2 = 0.139/1000; % Friction coeficient between the horizontal arm and the pendulum [N*m*s]
Lb = 3.3/1000; % Electric impendance of the motor (imaginary part) [H]
R = 2.272; % Electric internal resistance of the motor [ohm]
Kt = 0.699; % Counter-electromotive force term, coupling the angular speed and current of the motor [V*s]
Kf = 3.379; % Torque produced by the motor per current unit [V*s]

system_model = @inverted_pendulum;
u_limit = [u_min, u_max];
t = 0 : dt : 3.5;
t1 = 0; 
t2 = 3.5;
M = 2;
reference_cutoff_angle = 16/180*pi; %angle when reference gets set to 0
controller_cutoff = inf;

disturbance = zeros(length(t), length(x0));
disturbance(t==2.5, 3) = 10/180*pi;
disturbance(t==3, 3) = -10/180*pi;

noise = [0.001/10, 0.005, 0.001/10, 0.005, 0.001/2]/5;


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
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4) "_" "rob"], ...
        [fig2.Name "_" trajectory_file(1:end-4) "_" "rob"]}, PATH)
end



%% Energy control + gain_scheduling
controller = @gain_scheduling_lqr;
trajectory_file = 'EnergyControl_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise, controller_cutoff);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4) "_" "rob"], ...
        [fig2.Name "_" trajectory_file(1:end-4) "_" "rob"]}, PATH)
end



%% EnergyShaping_trajectory + gain_scheduling
controller = @gain_scheduling_lqr;
trajectory_file = 'EnergyShaping_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise, controller_cutoff);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4) "_" "rob"], ...
        [fig2.Name "_" trajectory_file(1:end-4) "_" "rob"]}, PATH)
end



%% ExpControl_trajectory + gain_scheduling
controller = @gain_scheduling_lqr;
trajectory_file = 'ExpControl_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise, controller_cutoff);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4) "_" "rob"], ...
        [fig2.Name "_" trajectory_file(1:end-4) "_" "rob"]}, PATH)
end

%%
controller_cutoff = reference_cutoff_angle;


%% Energy control + global_lqr
controller = @gain_scheduling_lqr;
trajectory_file = 'EnergyControl_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise, controller_cutoff);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4) "_" "rob"], ...
        [fig2.Name "_" trajectory_file(1:end-4) "_" "rob"]}, PATH)
end



%% EnergyShaping_trajectory + global_lqr
controller = @global_LQR;
trajectory_file = 'EnergyShaping_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise, controller_cutoff);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4) "_" "rob"], ...
        [fig2.Name "_" trajectory_file(1:end-4) "_" "rob"]}, PATH)
end



%% ExpControl_trajectory + global_lqr
controller = @global_LQR;
trajectory_file = 'ExpControl_trajectory.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise, controller_cutoff);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4) "_" "rob"], ...
        [fig2.Name "_" trajectory_file(1:end-4) "_" "rob"]}, PATH)
end

%% MPC_L2 + global_LQR
controller = @global_LQR;
trajectory_file = 'MPC_trajectory_norm2.mat';

[states, control, states_measured, state_reference, control_reference] = ... 
test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise, controller_cutoff);

[fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit);

if SAVE_PLOTS
    save_plots([fig1, fig2], {[fig1.Name "_" trajectory_file(1:end-4) "_" "rob"], ...
        [fig2.Name "_" trajectory_file(1:end-4) "_" "rob"]}, PATH)
end



