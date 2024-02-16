function u = exp_pendium_position(x, r)
    % u = sat(kv*|beta^n|)*sign(beta_dot*cos(beta))
  
    
    
    n = 1.38 ; % 1.65
    kv = 0.765; % 0.665
    
    u = kv*abs(x(3)^n);
    
    u = u * sign(x(4)*cos(x(3)));       
    
    
end