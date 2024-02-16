function bool_value = is_controllable(A, B)

    n = size(A, 1);

    controllability_matrix = zeros(n, n*size(B,2));
    
    % C = [B | AB | A^2B | ... | A^n-1B]
    for i = 1:n
        controllability_matrix(:, (i-1)*size(B,2)+1:i*size(B,2)) ...
            = A^(i-1) * B;
    end

    controllability_rank = rank(controllability_matrix);
    
    bool_value = controllability_rank == n;
    
end

