function xdot = system_dynamics(x, u)
    %parameters
    load parameters.mat J0 J2 Ka1 Ka2 Kf Lcm2 Le1 m2 g Kt R Lb
    
    %states
    
    x1 = x(1); % aplha - horizontal arm angle
    x2 = x(2); % alpha_dot - horizontal arm angular speed
    x3 = x(3); % beta - Pendulum angle
    x4 = x(4); % beta_dot - Pendulum angular speed
    x5 = x(5); % i - motor current

    x1_dot = x2;
    
    x2_dot = (-J2*(Ka1*x2 - Kf*x5 + x4*(Lcm2*Le1*m2*x4 + 2*J2*x2*cos(x3))*sin(x3))...
        + Lcm2*Le1*m2*cos(x3)*(-Ka2*x4 + (g*Lcm2*m2+J2*x2^2*cos(x3))*sin(x3)))/...
        (-Lcm2^2*Le1^2*m2^2*cos(x3)^2 + J2*(J0+J2*sin(x3)^2));
    
    x3_dot = x4;
    
    x4_dot = (Ka2*x4 - g*Lcm2*m2*sin(x3)*(J0+J2*sin(x3)^2)+...
        cos(x3)*((-J0*J2*x2^2 + Lcm2^2*Le1^2*m2^2*x4^2)*sin(x3)-...
        J2^2*x2^2*sin(x3)^3 + Lcm2*Le1*m2*(Ka1*x2 - Kf*x5 + J2*x2*x4*sin(2*x3))))/...
        (Lcm2^2*Le1^2*m2^2*cos(x3)^2 - J2*(J0 + J2*sin(x3)^2));
    
    x5_dot = (-Kt*x2 - R*x5 + u)/Lb;
    
    xdot = [x1_dot;x2_dot; x3_dot; x4_dot; x5_dot];


end
