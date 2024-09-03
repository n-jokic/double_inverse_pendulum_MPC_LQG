function u = exp_pendium_position(x, r)
    % u = sat(kv*|beta^n|)*sign(beta_dot*cos(beta))
  
    
    
    n = 3.2 ; % 1.65
    kv = 0.12; % 0.665
    
    u = kv*abs(x(3)^n);
    
    u = u * sign(x(4)*cos(x(3)));       
    
    
end