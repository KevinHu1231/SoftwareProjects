m = 30;
s = 1;
g = 9.81;
rho_inf = 1.23;
L = m*g;
u = linspace(10,50,81);
q_inf = 0.5*rho_inf*u.^2;
C_L = (L/s)./q_inf;
C_D = 0.02 + 0.05*C_L.^2;
L_D = C_L./C_D;

figure
plot(u,C_L);
title("UAV Flight Velocity and Corresponding Coefficient of Lift")
xlabel("Flight Velocity (m/s)");
ylabel("Coefficient of Lift");

figure
plot(u,C_D);
title("UAV Flight Velocity and Corresponding Coefficient of Drag")
xlabel("Flight Velocity (m/s)");
ylabel("Coefficient of Drag");

figure
plot(u,L_D);
title("UAV Flight Velocity and Corresponding Lift-to-Drag Ratio")
xlabel("Flight Velocity (m/s)");
ylabel("Lift-to-Drag Ratio");
