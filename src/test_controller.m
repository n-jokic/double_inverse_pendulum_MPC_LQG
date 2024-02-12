%% Controller setup
mpc = MPC_swing_up(2, [-5, 5], 10, dt, 100);
mpc.Rw = 0;

%% Simulation setup:
controller = @mpc.runMPC;
system_model = @inverted_pendulum;
measurement_model = @inverted_pendulum_measurement;


t = 0:dt:0.5;
M = 10;


x0 = [0; 0; pi; 0; 0];
u0 = 0;
disturbance = repmat(u0, 1, length(t))'; 
noise = [0.001, 0.001, 0.001, 0.001, 0.001];
%% Simulation:
[x, u] = ...
simulate_system(system_model, controller, x0, u0, disturbance, noise, t, M);

%% Plotting: 
plot_simulation_result(x, u, disturbance, t, [-5; 5]);
