function [x, u] = simulate_system(system_dynamics, controller, x0, u0, ...
    disturbance, sigma_noise,t, M)
u = zeros(length(t), length(u0));
x = zeros(length(t)+1, length(x0));
dt = t(2) - t(1);

noise_sample = mvnrnd(zeros(size(x0)), sigma_noise)';
x(2, :) = rk4_integration(x0+noise_sample, u0, system_dynamics, dt, M);

u(1, :) = u0;
x(1, :) = x0;

for i = 2 : length(t)
    noise_sample = mvnrnd([0; 0; 0; 0; 0], sigma_noise)';
    u(i, :) = controller(x(i, :)'+noise_sample) + disturbance(i, :);
    x(i+1, :) = rk4_integration(x(i, :)', u(i, :), ...
                                system_dynamics, dt, M);
end

end

