function y = inverted_pendulum_measurement(state_vector)

% x(1) - Horizontal arm angle x(3) - pendulum angle
y = [state_vector(1); state_vector(3)];
end

