%% Simulation setup:
controller = @energy_shaping;
system_model = @inverted_pendulum;
measurement_model = @inverted_pendulum_measurement;


t = 0:dt:1.3;
M = 4;


x0 = [0.0; 0;pi; 0; 0];
u0 = 0;
disturbance = repmat(u0, 1, length(t))'; 
noise = [0.01, 0.01, 0.01, 0.01, 0.01];
%% Simulation:
[x, u] = ...
simulate_trajectory(system_model, controller, x0, u0, disturbance, noise, t, M);
disp(find(abs(x(:,3)) <= 3*pi/180,1)*dt);
%% Plotting: 
% plot_simulation_result(x, u, disturbance, t, [-5; 5]);
x_ref = x;
u_ref = u;
save('EnergyShaping_trajectory', 'x_ref', 'u_ref');