clear all; close all; clc;

% energy_control / energy_shaping / exponential pendium position

%% Simulation params:
t_end = 2.7;  % Simuation duration [s]
% t = 0:dt:t_end;  % time axis [s]

d_tu = 250/10^6;
dt = d_tu;%0.01;
t = 0:d_tu:t_end;

% Initial states
x0 = [0;0;pi;0;0];

update = dt/d_tu;

X = zeros(length(t), length(x0));
X(1, :) = x0;
U = zeros(length(t)-1,1);

%%

for i = 2:length(t)
    
    % Calculate control using energy_control
     u = exp_pendium_position(X(i-1, :));
     U(i-1) = u;
    
    if mod(i,update) == 0
        % RK4 integration using inverted_pendulum

        X(i, :) = rk4_integration(X(i-1, :)', u, dt)';
        

    else
        X(i, :) = X(i-1, :);
    end
    
    
    
    X(i, 1) = wrapToPi(X(i,1));
    X(i, 3) = wrapToPi(X(i,3));
end
%%
% Plot the results
figure;
subplot(2, 1, 1);
plot(t, X(:, 3)*180/pi, 'LineWidth', 2); % Pendulum angle
xlabel('Time [s]');
ylabel('Pendulum Angle [deg]');
grid on;

subplot(2, 1, 2);
plot(t, X(:, 1)*180/pi, 'LineWidth', 2); % Horizontal arm angle
xlabel('Time [s]');
ylabel('Horizontal Arm Angle [deg]');
grid on;

figure, plot(t(1:end-1), U, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('input [V]');
grid on;
%% greska od 3 stepena

find(abs(X(:,3)) <= 3*pi/180, 1)*d_tu