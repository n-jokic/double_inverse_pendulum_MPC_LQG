function [A, B, C, D] = linearize(system_dynamics, measurement_model, ...
    x_linearization_point, ...
    u_linearization_point)

    n_x = length(x_linearization_point);
    n_u = length(u_linearization_point);
    
    x = sym('x', [1 n_x]);
    u = sym('u', [1 n_u]);

    if n_x > 1
        x = transpose(x);
    end

    if n_u > 1
        u = transpose(u);
    end
    y = measurement_model(x);
    
    f  = system_dynamics(x, u);
   

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

