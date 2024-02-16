%% Simulation setup:
controller = @global_LQR;
system_model = @inverted_pendulum;
measurement_model = @inverted_pendulum_measurement;
%load MPC_trajectory_norm1.mat u_ref x_ref;
load EnergyShaping_trajectory.mat u_ref x_ref;
% load EnergyControl_trajectory.mat u_ref x_ref;
%load ExpControl_trajectory.mat u_ref x_ref;

global closed0
closed0 = 0;
u_limit = [u_min, u_max];
t = 0:dt:4;
M = 4;
u_size = size(u_ref);
x_size = size(x_ref);

control_reference = zeros(length(t), u_size(2));
state_reference = zeros(length(t), x_size(2));

t0 = 1.262; % Shading
% t0 = 1.888; % Energy
%t0 = 2.1; % Exp control

control_reference(t<t0, :) = u_ref(t<t0, :);
state_reference(t<t0, :) = x_ref(t<t0, :);


x0 = [0.0; 0;pi; 0; 0];
u0 = 0;
disturbance = zeros(length(t), length(x0));

% Disturbance is impulse in nature
% disturbance(t==2, 3) = 20/180*pi;
disturbance(t==2.5, 3) = 20/180*pi;
disturbance(t==3.5, 3) = -20/180*pi;

noise = [0.01, 0.01*5, 0.01/10, 0.01*5, 0.01/10]/10;
%% Simulation:
[states, control, states_measured] = ...
simulate_system(system_model, controller, x0, u0, state_reference, ...
control_reference, disturbance, noise, t, M, u_limit);
disp(find(abs(states(:,3)) <= 3*pi/180,1)*dt);
%% Plotting: 
t1 = 0; t2 = 4;
plot_simulation_result(states, states_measured, ...
    control, state_reference, control_reference, disturbance, t, t1, t2, [-5; 5]);
