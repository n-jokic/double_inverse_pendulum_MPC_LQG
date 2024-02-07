function bool_value = is_observable(A, C)
    n = size(A, 1);
    observability_matrix = zeros(n*size(C,1), n);
    
    % O = [C | CA | CA^2 | ... |CA^(n-1)]
    for i = 1:n
        observability_matrix((i-1)*size(C,1)+1:i*size(C,1), :) = C * A^(i-1);
    end

    % observability matrix rank calculation
    rank_observability = rank(observability_matrix);

   
    bool_value = rank_observability == n;
end

