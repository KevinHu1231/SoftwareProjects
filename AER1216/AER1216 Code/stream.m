[x1, x2] = meshgrid(-3:0.2:3, -3:0.2:3);
dx1 = zeros(size(x1));
dx2 = zeros(size(x2));

for i = 1:numel(x1)
    x = [x1(i); x2(i)];
    dx = nonlinear_system(0, x);
    dx1(i) = dx(1);
    dx2(i) = dx(2);
end

figure;
streamslice(x1, x2, dx1, dx2);
xlabel('x_1');
ylabel('x_2');
title('Streamlines of Nonlinear System with Linear Portion');