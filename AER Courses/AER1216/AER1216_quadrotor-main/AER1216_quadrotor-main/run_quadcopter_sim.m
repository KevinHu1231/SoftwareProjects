clc
clear
clear all

sim("multirotor_full_system.slx");
t = ans.position.Time;

x = ans.position.Data(:, 1);
y = ans.position.Data(:, 2);
z = ans.position.Data(:, 3);

u = ans.velocity.Data(:, 1);
v = ans.velocity.Data(:, 2);
w = ans.velocity.Data(:, 3);

phi = ans.attitude.Data(:, 1);
theta = ans.attitude.Data(:, 2);
psi = ans.attitude.Data(:, 3);

phi_c = ans.control.Data(:, 1);
theta_c = ans.control.Data(:, 2);
yaw_rate_c = ans.control.Data(:, 3);
w_c = ans.control.Data(:, 4);

figure
subplot(4, 1, 1)
plot(t, phi_c)
legend("phi_c")
grid on

subplot(4, 1, 2)
plot(t, theta_c)
legend("theta_c")
grid on

subplot(4, 1, 3)
plot(t, yaw_rate_c)
legend("yaw rate_c")
grid on

subplot(4, 1, 4)
plot(t, w_c)
legend("w_c")
grid on

figure
subplot(3, 3, 1)
plot(t, x)
legend("x")
grid on

subplot(3, 3, 2)
plot(t, y)
legend("y")
grid on

subplot(3, 3, 3)
plot(t, z)
legend("z")
grid on

subplot(3, 3, 4)
plot(t, u)
legend("u")
grid on

subplot(3, 3, 5)
plot(t, v)
legend("v")
grid on

subplot(3, 3, 6)
plot(t, w)
legend("w")
grid on

subplot(3, 3, 7)
plot(t, theta)
legend("theta")
grid on

subplot(3, 3, 8)
plot(t, phi)
legend("phi")
grid on

subplot(3, 3, 9)
plot(t, psi)
legend("psi")
grid on

% phi_c_calc = ans.phi_c.Data(:, 1);
% theta_c_calc = ans.theta_c.Data(:, 1);

% figure
% subplot(2, 1, 1)
% plot(t, phi_c_calc)
% legend("phi_c calc")
% grid on
% 
% subplot(2, 1, 2)
% plot(t, theta_c_calc)
% legend("theta_c calc")
% grid on