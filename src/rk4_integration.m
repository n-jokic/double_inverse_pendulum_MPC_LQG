function x_next = rk4_integration(x, u, system_dynamics, dt)
    k1 = dt * system_dynamics(x, u);
    k2 = dt * system_dynamics(x + k1/2, u);
    k3 = dt * system_dynamics(x + k2/2, u);
    k4 = dt * system_dynamics(x + k3, u);

    x_next = x + (k1 + 2*k2 + 2*k3 + k4)/6;
end
