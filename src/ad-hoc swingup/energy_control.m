function u = energy_control(x, r)
    % u = sat[kv(E-E0)]sign(beta_dot*cos(beta))
    
    load parameters.mat J0 J2 Lcm2 Le1 m2 g 
    
    
    kv = 0.68;
    E0 = 0;
    
    l2 = Lcm2; % Distance from axis of rotation to centre of mass of the pendulum[m]
    L2 = 0.6;%Lcm2; Lenght of the pendulum arm [m]
    
    L1  = 0.1;%Le1; Distance from axis of rotation to centre of mass of the horizontal arm [m]
    l1 = Le1; % Lenght of the horizontal arm [m]
    
    m1 = m2/5;
    % Arm1
    Ep1 = 0;
    Ek1 = 1/2*x(2)^2*(m1*l1^2 + J0);
    
    % Arm2
    Ep2 = g*m2*l2*(1-cos(x(3)));
    Ek2 = 1/2*x(1)^2*(m2*L2^2 + (m2*l2^2 + J2)*sin(x(3))^2) + ...
        1/2*x(4)^2*(J2 + m2*l2^2) + m2*L1*l2*cos(x(3))*x(2)*x(4);
    
    E = Ek1 + Ep1 + Ek2 + Ep2;
    u = kv*(E-E0);
    
    u = u * sign(x(4)*cos(x(3)));       
    
end