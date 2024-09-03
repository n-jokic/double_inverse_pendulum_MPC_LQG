%% MPC setup
rk4_steps = 1;
control_signal_rate = 0.001;
N_horizon = 1500;

mpc = MPC_swing_up([u_min, u_max], rk4_steps, ...
    control_signal_rate, N_horizon);

%% 2 norm
mpc.gamma_control =1;
moc.gamma_states = 0.0;
mpc.gamma_final = 5;

u0 = randn(mpc.N*mpc.n_controls, 1)*2; 

scale = [pi/2 0 0 0 0;
         0 5 0 0 0;
         0 0 pi/2 0 0;
         0 0  0   5 0;
         0 0 0    0 2
         ];
X0 = repmat(randn(1, mpc.n_states)*scale, 1,mpc.N+1)';
mpc.x0  = [X0; u0];

%mpc.gamma_final = N_horizon/200;
%%
mpc.runMPC([0, 0, pi, 0, 0], [0, 0]), mpc.plot_trajectories();

x_ref = mpc.x_mpc; 
u_ref = mpc.u_mpc;

if SAVE_TRAJECTORY
    loc = fullfile(cd, 'MPC swingup');
    name = fullfile(loc, 'MPC_trajectory_norm2');
    save('MPC_trajectory_norm2', 'x_ref', 'u_ref');
end
%% 1 norm
%mpc.gamma_control = 4;
%mpc.gamma_final = .5;
%mpc.u_norm = 1;
%mpc.runMPC([0, 0, pi, 0, 0], [0, 0]), mpc.plot_trajectories();

%x_ref = mpc.x_mpc;
%u_ref = mpc.u_mpc;

%if SAVE_TRAJECTORY
%    loc = fullfile(cd, 'MPC swingup');
%    name = fullfile(loc, 'MPC_trajectory_norm1');
%    save(name, 'x_ref', 'u_ref');
%end