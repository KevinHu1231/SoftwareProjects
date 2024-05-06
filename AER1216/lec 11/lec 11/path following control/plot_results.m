close all
figure(1)
% actual trajectory
plot3(Xe.data(:, 1), Xe.data(:, 2), Xe.data(:, 3), 'LineWidth', 2)
hold on
% reference trajectory
Xs = P;
Xt = P + 30 * n;
plot3([Xs(1), Xt(1)], [Xs(2), Xt(2)], [Xs(3), Xt(3)], '--r', 'LineWidth', 2)

grid on 
axis equal
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')

figure(2)

Vabs = sqrt( Ve.data(:, 1).^2  + Ve.data(:, 2).^2 + Ve.data(:, 3).^2);

plot(Ve.Time, Vabs, 'LineWidth', 2)
grid on 
xlabel('t(s)')
ylabel('||v||(m/s)')