%% Simulation setup:
controller = @test;
system_model = @inverted_pendulum;
measurement_model = @inverted_pendulum_measurement;


t = 0:dt:10;
M = 10;


x0 = [0.1; 0; 0.1; 0; 0];
u0 = 0;
disturbance = repmat(u0, 1, length(t))'; 
noise = [0.01, 0.01, 0.01, 0.01, 0.01];
%% Simulation:
[x, u] = ...
simulate_system(system_model, controller, x0, u0, disturbance, noise, t, M);

%% Plotting: 
plot_simulation_result(x, u, disturbance, t, [-5; 5]);