function control = gain_scheduling_lqr(state_measurement, ...
    reference)
persistent controllers alpha;

if isempty(controllers)
    controllers = {};
    alpha = [0, 36, 72, 108, 144, 180]/180*pi;

    for i = 1 : length(alpha)
    controllers{i} = make_lqr(@inverted_pendulum, ...
        @inverted_pendulum_measurement, [0; 0; alpha(i); 0; 0], 0);
    end
end

% reduction to interval from -pi to pi: 
alpha_measured = mod(state_measurement(3), 2*pi);
alpha_measured = alpha_measured - 2*pi*(alpha_measured>pi);

[~, min_idx] = min(abs(alpha-abs(alpha_measured)));

% linearization point for LQR
% closest_angle = sign(alpha_measured)*alpha(min_idx);
closest_controller = controllers{min_idx};

control = closest_controller*(reference - state_measurement);


end

