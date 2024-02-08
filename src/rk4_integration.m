function x_next = rk4_integration(x, u, dt)
    k1 = dt * inverted_pendulum(x, u);
    k2 = dt * inverted_pendulum(x + k1/2, u);
    k3 = dt * inverted_pendulum(x + k2/2, u);
    k4 = dt * inverted_pendulum(x + k3, u);

    x_next = x + (k1 + 2*k2 + 2*k3 + k4)/6;
end
