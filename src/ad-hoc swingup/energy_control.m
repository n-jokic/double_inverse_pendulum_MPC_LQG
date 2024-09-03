function u = energy_control(x, r)
    % u = sat[kv(E-E0)]sign(beta_dot*cos(beta))
    
    load model_parameters.mat J0 J2 Lcm2 Le1 m2 g 
    
    
    kv = 0.68*12;
    E0 = 0;
    
    l2 = Lcm2; % Distance from axis of rotation to centre of mass of the pendulum[m]
    L2 = 0.6;%Lcm2; Lenght of the pendulum arm [m]
    
    % Arm2
    w0 = sqrt(m2*l2*g/J2);
    E = g*m2*l2/2*(cos(x(3)) - 1 +(x(4)/w0)^2);
    E0 = g*m2*l2;

    u = -kv*(E-E0);
    
    u = u * sign(x(4)*cos(x(3)));       
    
end