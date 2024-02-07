function [bool_value, system_eigenvalues] = is_stable(A, B, C, D)
    syms s 

    G = C*(s*eye(5) - A)^-1*B;
    G = simplify(G);
   

    [~, den] = ss2tf(double(A), double(B), double(C), double(D));
    system_eigenvalues = roots(den);
    bool_value = all(real(poles) <= 0);
    
end


