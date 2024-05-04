syms x1 x2 x3 xi1 xi2 z
x3_Sol = solve(xi2 == 2*cos(z)*cos(x3) + 2*sin(z)*sin(x3), x3)
co = cos(z_Sol)
si = sin(z_Sol)
z_dot = simplify(x1*si - x2*co)