function [states, control, states_measured, state_reference, ...
    control_reference] = test_controller(controller, ...
    system_model, trajectory_file, u_limit, ...
    t, M, reference_cutoff_angle, disturbance, noise)


%Simulation setup:

data = load(trajectory_file);
u_ref = data.u_ref;
x_ref = data.x_ref;

u_size = size(u_ref);
x_size = size(x_ref);
control_reference = zeros(length(t), u_size(2));
state_reference = zeros(length(t), x_size(2));

control_reference(1:length(u_ref), :) = u_ref;
state_reference(1:length(x_ref), :) = x_ref;



x0 = [0.0; 0; pi; 0; 0];
u0 = 0;

% Simulation:
[states, control, states_measured, state_reference, control_reference] = ...
simulate_system(system_model, controller, x0, u0, state_reference, ...
control_reference, disturbance, noise, t, M, ...
u_limit, reference_cutoff_angle);

disp('Steady state for 3rd state reached after: ');
idx = find(abs(states(:,3)) <= 3*pi/180,1);
disp(find(abs(states(:,3)) <= 3*pi/180,1)*(t(2)-t(1)));

disp('Control 2norm: ');
disp(sqrt(um(control.^2)));

end

