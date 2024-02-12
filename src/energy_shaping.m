function u = energy_shaping(x)
    % u =  k1 (x2 + k2 cos(x3)x4)
    umax = 5;
    umin = -5;
    
    k1 = 2; % k1 > 0
    k2 = 5.0; % k2 > 0
    
    u = k1*(x(2) + k2*cos(x(3))*x(4));
    
    if u > umax
        u = umax;
    else
        if u < umin
            u = umin;
        end
    end
    
    
end