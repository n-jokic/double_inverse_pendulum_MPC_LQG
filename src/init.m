clear; close all; clc;
rng default
%% Casadi setup:
home_directory = string(java.lang.System.getProperty("user.home"));
casadi_folder = 'casadi-3.6.4-windows64-matlab2018b';
casad_path = fullfile(home_directory, casadi_folder);
addpath(casad_path);

%% SAVE FLAG
SAVE_PARAM = 0;
SAVE_PLOTS = 1;
SAVE_TRAJECTORY = 0;

PATH = "..\latex\slike";

%% Adding folders to path:
folder_path = fullfile(cd, 'ad-hoc swingup');
addpath(folder_path);

folder_path = fullfile(cd, 'linearization');
addpath(folder_path);

folder_path = fullfile(cd, 'LQR');
addpath(folder_path);

folder_path = fullfile(cd, 'model');
addpath(folder_path);

folder_path = fullfile(cd, 'MPC swingup');
addpath(folder_path);

folder_path = fullfile(cd, 'simulation');
addpath(folder_path);

%% Model params:
Le1 = 227/1000; % Lenght of the horizontal arm [m]
J0 = 86.98/1000; % Moment of inertia at the base joint of the horizontal arm and pendulum [kg*m^2]
Ka1 = 1/1000; % Friction coeficient between base and the horizontal arm [N*m*s]
m2 = 309/1000; % Mass of the pendulum [kg]
Lcm2 = 404/1000; % Distance from axis of rotation to centre of mass of the pendulum[m]
J2 = 28.37/1000; % Moment of inertia at the joint of the pendulum[kg*m^2]
Ka2 = 0.136/1000; % Friction coeficient between the horizontal arm and the pendulum [N*m*s]
Lb = 3/1000; % Electric impendance of the motor (imaginary part) [H]
R = 2.266; % Electric internal resistance of the motor [ohm]
Kt = 0.696; % Counter-electromotive force term, coupling the angular speed and current of the motor [V*s]
Kf = 3.377; % Torque produced by the motor per current unit [V*s]
g = 9.81; % [m/s^2]

if SAVE_PARAM
    save('parameters', 'Le1', 'J0', 'Ka1', 'm2', 'Lcm2', 'J2', 'Ka2',...
    'Lb', 'R', 'Kt', 'Kf', 'g');
end

u_min = -5;
u_max = 5;
%% Simulation params:
dt = 0.001;  % Sampling time [s]
t_end = 10;  % Simuation duration [s]
time = 0:dt:t_end;  % time axis [s]

% Initial states
x0 = [0;0;0;0;0];

% Initial control
u0 = 0;