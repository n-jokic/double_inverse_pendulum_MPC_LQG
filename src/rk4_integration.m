function x_next = rk4_integration(x, u, system_dynamics, dt, M)
    x_next = x;
    time_step = dt/M;

    for i = 1:M
        k1 = time_step * system_dynamics(x_next, u);
        k2 = time_step * system_dynamics(x_next + k1/2, u);
        k3 = time_step * system_dynamics(x_next + k2/2, u);
        k4 = time_step * system_dynamics(x_next + k3, u);

        x_next = x + (k1 + 2*k2 + 2*k3 + k4)/6;
    end
end
