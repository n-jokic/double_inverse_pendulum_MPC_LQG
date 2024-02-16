function control = global_LQR(state_measurement, ...
    reference)
persistent controller
turn_off_controller = 0;

if isempty(controller)
    controller =  make_lqr(@inverted_pendulum, ...
        @inverted_pendulum_measurement, [0; 0; 0; 0; 0], 0);
end

% reduction to interval from -pi to pi: 
alpha_measured = mod(state_measurement(3), 2*pi);
alpha_measured = alpha_measured - 2*pi*(alpha_measured>pi);

if(abs(alpha_measured) > 18*pi/180) % swing up
    turn_off_controller = 1;
end

control = (1-turn_off_controller)*controller*(reference - state_measurement);


end

