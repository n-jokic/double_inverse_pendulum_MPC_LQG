%% norm 2
rk4_steps = 2;
control_signal_rate = 0.001;
N_horizon = 2000;
norm = 2;
mpc = MPC_swing_up(norm, [u_min, u_max], rk4_steps, ...
    control_signal_rate, N_horizon);

mpc.runMPC([0, 0, pi, 0, 0]), mpc.plot_trajectories();

x_ref = mpc.x_mpc;
u_ref = mpc.u_mpc;

save('MPC_trajectory_norm2', 'x_ref', 'u_ref');

%% norm 1

norm = 1;

mpc = MPC_swing_up(norm, [u_min, u_max], rk4_steps, ...
    control_signal_rate, N_horizon);

mpc.runMPC([0, 0, pi, 0, 0]), mpc.plot_trajectories();

x_ref = mpc.x_mpc;
u_ref = mpc.u_mpc;

save('MPC_trajectory_norm1', 'x_ref', 'u_ref');