function A_inv = tridiagonal_inverse(A)
    % Check if A is a valid tridiagonal matrix
    [m, n] = size(A);
    if m ~= n || ~isequal(A, tril(A, -1) + diag(diag(A)) + triu(A, 1))
        error('Input matrix is not a valid tridiagonal matrix.');
    end

    % Extract the diagonal and off-diagonal elements
    d = diag(A);
    a = diag(A, -1);
    b = diag(A, 1);

    % Determine the size of the matrix
    n = length(d);

    % Initialize variables for the Thomas algorithm
    alpha = zeros(n, 1);
    beta = zeros(n, 1);

    % Forward sweep
    alpha(1) = d(1);
    for i = 2:n
        beta(i) = b(i-1) / alpha(i-1);
        alpha(i) = d(i) - beta(i) * a(i-1);
    end

    % Backward substitution
    A_inv = eye(n);
    for i = n:-1:2
        A_inv(i, i-1) = -beta(i);
        A_inv(i-1, i) = 1;
    end

    % Final step for the last element
    A_inv(n, n) = 1 / alpha(n);

    % Invert the diagonal elements
    A_inv = diag(1./d) * A_inv;

end