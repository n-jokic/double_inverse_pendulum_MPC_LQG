function K = make_lqr(system_dynamics, ...
    measurement_model, x_linearization_point, ...
    u_linearization_point)


[A, B, ~, ~] = linearize(system_dynamics, ...
    measurement_model, x_linearization_point, u_linearization_point);

Q = [1/pi 0 0 0 0;
     0 1/5 0 0 0;
     0 0 1/pi 0 0;
     0 0 0 1/5 0;
     0 0 0 0 1/4
     ];

R = 1/5;

K = lqr(A,B,Q,R);
end

