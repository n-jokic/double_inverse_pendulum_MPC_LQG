function [A, B, C, D] = linearize(x_linearization_point, u_linearization_point)

    n_x = length(x_linearization_point);
%     n_u = length(u_linearization_point);

    x = reshape(sym('x', [1 n_x]),n_x,1);
    u = sym('u');
    y = inverted_pendulum_measurement(x);
    
    f  = inverted_pendulum(x, u);
   % subs([x; u], [x_linearization_point; u_linearization_point] );

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

