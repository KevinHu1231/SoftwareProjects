% Define your tridiagonal matrix A
A = [1 -0.5 0;
     -0.5 1 -0.5;
     0 -0.5 1];

% Calculate the inverse
A_inv = tridiagonal_inverse(A)