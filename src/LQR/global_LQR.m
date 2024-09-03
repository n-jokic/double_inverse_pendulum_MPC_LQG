function control = global_LQR(state_measurement, ...
    reference)
persistent controller

if isempty(controller)
    controller =  make_lqr(@inverted_pendulum, ...
        @inverted_pendulum_measurement, [0; 0; 0; 0; 0], 0);
end

controller =  make_lqr(@inverted_pendulum, ...
        @inverted_pendulum_measurement, [0; 0; 0; 0; 0], 0);


control = controller*(reference - state_measurement);


end

