function u = energy_shaping(x, r)
    % u =  k1 (x2 + k2 cos(x3)x4)

    
    k1 = 2; % k1 > 0 0.5
    k2 = 4; % k2 > 0  0.9
    
    u = k1*x(2) + k2*cos(x(3))*x(4);
    
  
    
    
end