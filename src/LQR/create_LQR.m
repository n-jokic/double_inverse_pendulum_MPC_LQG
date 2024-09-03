x0 = [0; 0; 0; 0; 0];
u0 = 0;

[A, B, C, D] = linearize(@inverted_pendulum, ...
    @inverted_pendulum_measurement, x0, u0);

%% LQR controller
Q = [1/pi 0 0 0 0;
     0 1/10 0 0 0;
     0 0 1/pi 0 0;
     0 0 0 1/10 0;
     0 0 0 0 1/4
     ]*10;

R = 1/6;

K = lqr(A,B,Q,R);
save('lqr', 'K');