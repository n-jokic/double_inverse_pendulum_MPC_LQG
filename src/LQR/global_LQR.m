function control = global_LQR(state_measurement, ...
    reference)

global closed0
load lqr.mat K

controllers0 = K;
% reduction to interval from -pi to pi: 
alpha_measured = mod(state_measurement(3), 2*pi);
alpha_measured = alpha_measured - 2*pi*(alpha_measured>pi);

if(alpha_measured > 1*pi/180 || alpha_measured < -1*pi/180) % swing up
    controller = [0,0,0,0,0];
else
    controller = controllers0; 
    closed0 = 1;
end

if (closed0==1)
    controller = controllers0;
end

control = controller*(reference - state_measurement);


end

