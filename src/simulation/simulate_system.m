function [states, control, states_measured, state_reference, ...
    control_reference] = simulate_system( ...
    system_dynamics, controller, x0, u0, ...
    state_reference, control_reference, disturbance, ...
    sigma_noise, t, M, u_limit, reference_cutoff_angle, ...
    controller_cutoff)
control = zeros(length(t), length(u0));
states = zeros(length(t)+1, length(x0));
states_measured = zeros(length(t)+1, length(x0));
dt = t(2) - t(1);

states(2, :) = rk4_integration(x0, u0, system_dynamics, dt, M);

control(1, :) = u0;
states(1, :) = x0;
states_measured(1, :) = x0;

reached_cutoff = 0;
cont_off = 0;

for i = 2 : length(t)
    noise_sample = mvnrnd([0; 0; 0; 0; 0], sigma_noise)';
    states_measured(i, :) = states(i, :)'+noise_sample;

    if abs(states_measured(i, 3)) < reference_cutoff_angle ...
            && reached_cutoff~=1
        reached_cutoff = 1;
        display(['Controlled turned on: ' num2str(i*dt)]);
    end

    if abs(states_measured(i, 3)) >  controller_cutoff 
        cont_off = 1;
        

    end

    control_reference(i, :) = (1-reached_cutoff)*control_reference(i, :);
    state_reference(i, :) = (1-reached_cutoff)*state_reference(i, :);

    control(i, :) = (1-cont_off)*controller(states_measured(i, :)', ...
        state_reference(i, :)') + control_reference(i, :);

    control(i, :) = max(min(control(i, :), u_limit(2)), u_limit(1));

    states(i+1, :) = rk4_integration(states(i, :)'+disturbance(i, :)', ...
        control(i, :), system_dynamics, dt, M);
end


end

