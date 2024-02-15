

%% Simulation setup:
controller = @gain_scheduling_lqr_pendulum;
system_model = @inverted_pendulum;
measurement_model = @inverted_pendulum_measurement;
load MPC_trajectory_norm1.mat u_ref x_ref;


t = 0:dt:5;
M = 2;
u_size = size(u_ref);
x_size = size(x_ref);
control_reference = zeros(length(t), u_size(2));
state_reference = zeros(length(t), x_size(2));

control_reference(t<1.6, :) = u_ref(t<1.6, :);
state_reference(t<1.6, :) = x_ref(t<1.6, :);



x0 = [0.0; 0;pi; 0; 0];
u0 = 0;
disturbance = repmat(u0, 1, length(t))'; 
noise = [0.01, 0.01, 0.01, 0.01, 0.01];
%% Simulation:
[x, u] = ...
simulate_system(system_model, controller, x0, u0, state_reference, ...
control_reference, disturbance, noise, t, M);
disp(find(abs(x(:,3)) <= 3*pi/180,1)*dt);
%% Plotting: 
plot_simulation_result(x, u, disturbance, t, [-5; 5]);