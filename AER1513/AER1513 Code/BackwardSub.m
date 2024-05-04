function y = BackwardSub(a,b)
% The function solves a system of linear equations ax=b 
% where a is upper triangular by using backward substitution.
% Input variables:
% a  The matrix of coefficients.
% b  A column vector of constants.
% Output variable:
% y  A colum vector with the solution.

n = length(b);
y(n,1) = b(n)/a(n,n);
for i = n-1:-1:1
    y(i,1)=(b(i)-a(i,i+1:n)*y(i+1:n,1))./a(i,i);
end