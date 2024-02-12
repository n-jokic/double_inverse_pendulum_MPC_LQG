function u = exp_pendium_position(x)
    % u = sat(kv*|beta^n|)*sign(beta_dot*cos(beta))
    
    umax = 5;
    umin = -5;
    
    kv = 0.665*2;
    n = 2.15;
    
    u = kv*abs(x(3)^n);
    
     if u > umax
        u = umax;
    else
        if u < umin
            u = umin;
        end
    end
    
    u = u * sign(x(4)*cos(x(3)));       
    
    
end