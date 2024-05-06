c1 = 2;
c2 = 3;

x1_0 = pi/4;
x2_0 = pi/6;
x3_0 = 0;
x4_0 = 0;

a1 = 1;
a2 = 1;

tspan = [0, 10];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);


p22 = 1;
p12 = p22*c2;

% System dynamics
f = @(t, x) [x(2); -(a1/a2)*sin(x(1)) + (1/a2)*(x(3)*sin(x(1))-x(4)*(c1*(x(1)-pi) + c2*x(2))); 
    -2*p12*(x(1)-pi)*sin(x(1))-2*p22*x(2)*sin(x(1)); 
    2*p12*(x(1)-pi)^2*c1 + 2*p12*(x(1)-pi)*c1*x(2) + 2*p22*x(2)*c1*(x(1)-pi) + 2*p22*x(2)^2*c2];

% Simulation
[t, y] = ode45(f, tspan, [x1_0; x2_0; x3_0; x4_0], options);

% Plotting
figure;

subplot(2, 2, 1);
plot(t, y(:, 1), 'LineWidth', 2);
xlabel('Time');
ylabel('x1');
title('Pendulum Angle');

subplot(2, 2, 2);
plot(t, y(:, 2), 'LineWidth', 2);
xlabel('Time');
ylabel('x2');
title('Angular Velocity');

% The estimated parameter over time
a1_hat = y(:, 3);
a1_trues = a1 * ones(length(y));

a2_hat = y(:, 4);
a2_trues = a2 * ones(length(y));

subplot(2, 2, 3);
plot(t, a1_hat, 'LineWidth', 2);
hold on
plot(t, a1_trues, 'LineWidth', 2);
xlabel('Time');
ylabel('Estimated a1');
title('a1 Estimation');

subplot(2, 2, 4);
plot(t, a2_hat, 'LineWidth', 2);
hold on
plot(t, a2_trues, 'LineWidth', 2);
xlabel('Time');
ylabel('Estimated a2');
title('a2 Estimation');