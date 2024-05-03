function dxdt = nonlinear_system(t, x)

    f_nonlinear = [x(1)^2 + x(2)^2; 0];

    A_linear = [0, 1; -1, -2];

    dxdt = A_linear * x + f_nonlinear;
end
