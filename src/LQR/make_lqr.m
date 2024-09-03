function K = make_lqr(system_dynamics, ...
    measurement_model, x_linearization_point, ...
    u_linearization_point)


[A, B, C, D] = linearize(system_dynamics, ...
    measurement_model, x_linearization_point, u_linearization_point);

continious_system = ss(A,B,C,D);
%discrete_system = c2d(continious_system, dt);


Q = [(1/(pi))^2 0 0 0 0;
     0 (1/2/pi)^2 0 0 0;
     0 0 0*(0.1/(pi))^2 0 0;
     0 0 0 (1/2/pi)^2 0;
     0 0 0 0 (1/4)^2
     ];

R = 1/4;

N = [1;
    0;
    1;
    0;
    0];

K = lqr(A, B, Q, R);
end

