function y = inverted_pendulum_measurement(state_vector)

% x(1) - Horizontal arm angle x(3) - pendulum angle
y = [state_vector(1); state_vector(2); state_vector(3); state_vector(4);
    state_vector(5)];
end

