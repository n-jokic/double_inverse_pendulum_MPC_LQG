function [bool_value, G, system_eigenvalues] = is_stable(A, B, C, D)
    syms s 

    G = C*(s*eye(5) - A)^-1*B;
    G = simplify(G);
   

    [~, den] = ss2tf(double(A), double(B), double(C), double(D));
    system_eigenvalues = roots(den);
    bool_value = all(real(system_eigenvalues) <= 0);

end


