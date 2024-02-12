function [A, B, C, D, G, controllable, observable, stable] = ...
    linear_dynamics_analysis(system_dynamics, measurement_model, ...
    x_linearization_point, ...
    u_linearization_point, ....
    print_info)

if nargin == 4
    print_info = true;
end

true_false = ["false"; "true"];

[A, B, C, D] = linearize(system_dynamics, measurement_model, ...
    x_linearization_point, ...
    u_linearization_point);

if print_info
    disp('--------');
    disp(['linearization around x = ' ...
        num2str(round(x_linearization_point', 3)) ...
        ', u = ' num2str(round(u_linearization_point, 3))]);
    disp('A: ');
    disp(A);
    disp('B: ');
    disp(B);
    disp('C: ');
    disp(C);
    disp('D: ');
    disp(D);
    disp('--------');
end
[stable, G, system_eigenvalues] = is_stable(A, B, C, D);

if print_info
    disp('System is stable: ');
    disp(true_false(stable+1));
    disp('eigenvalues: ');
    disp(system_eigenvalues);
    disp('--------');
end

controllable = is_controllable(A, B);
if print_info
    disp('System is controlable: ');
    disp(true_false(controllable+1));
    disp('--------');
end

observable = is_observable(A, C);

if print_info
    disp('System is observable: ');
    disp(true_false(observable+1));
    disp('--------');
end

end

