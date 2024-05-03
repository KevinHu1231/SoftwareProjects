c1 = 2;
c2 = 3;


x1_0 = pi/3;
x2_0 = pi/3;
x3_0 = 3;
theta_true = 5;

tspan = [0, 10];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

p22 = 1;
p12 = p22*c2;

% System dynamics
f = @(t, x) [x(2); sin(x(1))*(x(3)-theta_true) - c1*(x(1)-pi) - c2*x(2); -2*p12*(x(1)-pi)*sin(x(1))-2*p22*x(2)*sin(x(1))];

% Simulation
[t, y] = ode45(f, tspan, [x1_0; x2_0; x3_0], options);

% Plotting
figure;

subplot(3, 1, 1);
plot(t, y(:, 1), 'LineWidth', 2);
xlabel('Time');
ylabel('x1');
title('Pendulum Angle');

subplot(3, 1, 2);
plot(t, y(:, 2), 'LineWidth', 2);
xlabel('Time');
ylabel('x2');
title('Angular Velocity');

% The estimated parameter over time
theta_hat = y(:, 3);
theta_trues = theta_true * ones(length(y)); 

subplot(3, 1, 3);
plot(t, theta_hat, 'LineWidth', 2);
hold on
plot(t, theta_trues, 'LineWidth', 2);
xlabel('Time');
ylabel('Estimated theta');
title('Parameter Estimation');