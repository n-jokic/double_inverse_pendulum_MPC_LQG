function [A, B, C, D] = linearize(system_dyn, x_linearization_point, ...
    u_linearization_point, observation_matrix)

    n_x = length(x_linearization_point);
    n_u = length(u_linearization_point);

    x = sym('x', [1 n_x])';
    u = sym('x', [1 n_u])';
    y = observation_matrix*x;
    
    f  = system_dyn(x, u);
    subs([x; u], [x_linearization_point; u_linearization_point] );

    A = jacobian(f, x);
    A = double(subs(A, [x; u], ...
        [x_linearization_point; u_linearization_point] ...
        ));

    B = jacobian(f, u);
    B = double(subs(B, [x; u], ...
        [x_linearization_point; u_linearization_point] ...
        ));

    C = jacobian(y, x);
    C = double(subs(C, [x; u], ...
        [x_linearization_point; u_linearization_point] ...
        ));

    D = jacobian(y, u);
    D = double(subs(D, [x; u], ...
        [x_linearization_point; u_linearization_point] ...
        ));
end

