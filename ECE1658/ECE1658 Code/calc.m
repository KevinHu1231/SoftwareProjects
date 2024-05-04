syms x1 x2 x
v1 = (-2*x^2+1)/(x^2+1)^2;
v2 = (4*x^2)/(x^2+1)^2;
subexp = x1/(1-x2);
v1_sub = simplify(subs(v1,x,subexp))
v2_sub = simplify(subs(v2,x,subexp))