x0 = [0; 0; 0; 0; 0];
u0 = 0;

[A, B, C, D] = linearize(@inverted_pendulum, ...
    @inverted_pendulum_measurement, x0, u0);

%% LQR controller
Q = [1/pi 0 0 0 0;
     0 1/pi/2 0 0 0;
     0 0 1/pi 0 0;
     0 0 0 1/pi/2 0;
     0 0 0 0 1/2
     ];

R = 1/5;

K = lqr(A,B,Q,R);
save('lqr', 'K');