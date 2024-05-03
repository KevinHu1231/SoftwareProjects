omega = 5/10;
T = 2*pi/omega;
T_steps = ceil(T*1000);

r_10_v_5_x = r_10_v_5(1:T_steps,1);
r_10_v_5_y = r_10_v_5(1:T_steps,2);
r_10_v_5_z = r_10_v_5(1:T_steps,3);
norm_r_10_v_5 = sqrt(r_10_v_5_x.^2 + r_10_v_5_y.^2 + r_10_v_5_z.^2);
avg_norm_r_10_v_5 = mean(norm_r_10_v_5);

circle_r_10_v_5 = linspace(0,length(r_10_v_5_x)-1,length(r_10_v_5_x))/(length(r_10_v_5_x)-1);

omega = 1/10;
T = 2*pi/omega;
T_steps = ceil(T*1000);

r_10_v_1_x = r_10_v_1(1:T_steps,1);
r_10_v_1_y = r_10_v_1(1:T_steps,2);
r_10_v_1_z = r_10_v_1(1:T_steps,3);
norm_r_10_v_1 = sqrt(r_10_v_1_x.^2 + r_10_v_1_y.^2 + r_10_v_1_z.^2);
avg_norm_r_10_v_1 = mean(norm_r_10_v_1);
circle_r_10_v_1 = linspace(0,length(r_10_v_1_x)-1,length(r_10_v_1_x))/(length(r_10_v_1_x)-1);

omega = 2/10;
T = 2*pi/omega;
T_steps = ceil(T*1000);
r_10_v_2_x = r_10_v_2(1:T_steps,1);
r_10_v_2_y = r_10_v_2(1:T_steps,2);
r_10_v_2_z = r_10_v_2(1:T_steps,3);
norm_r_10_v_2 = sqrt(r_10_v_2_x.^2 + r_10_v_2_y.^2 + r_10_v_2_z.^2);
avg_norm_r_10_v_2 = mean(norm_r_10_v_2);
circle_r_10_v_2 = linspace(0,length(r_10_v_2_x)-1,length(r_10_v_2_x))/(length(r_10_v_2_x)-1);

omega = 0.5/10;
T = 2*pi/omega;
T_steps = ceil(T*1000);

r_10_v_05_x = r_10_v_05_fixed(1:T_steps,1);
r_10_v_05_y = r_10_v_05_fixed(1:T_steps,2);
r_10_v_05_z = r_10_v_05_fixed(1:T_steps,3);
norm_r_10_v_05 = sqrt(r_10_v_05_x.^2 + r_10_v_05_y.^2 + r_10_v_05_z.^2);
avg_norm_r_10_v_05 = mean(norm_r_10_v_05);
circle_r_10_v_05 = linspace(0,length(r_10_v_05_x)-1,length(r_10_v_05_x))/(length(r_10_v_05_x)-1);

xs = [0.5,1,2,5];
ys = [avg_norm_r_10_v_05,avg_norm_r_10_v_1,avg_norm_r_10_v_2,avg_norm_r_10_v_5];

figure
plot(xs,ys)
title("Average Path Tracking Error Along One Revolution of a Circle for Various Velocities with R = 10")
xlabel("Velocity (m/s)")
ylabel("Average Path Tracking Error ||r|| (m)")

% figure
% plot(circle_r_10_v_05,norm_r_10_v_05)
% hold on
% plot(circle_r_10_v_1,norm_r_10_v_1)
% hold on
% plot(circle_r_10_v_2,norm_r_10_v_2)
% hold on
% plot(circle_r_10_v_5,norm_r_10_v_5)
% title("Path Tracking Errors Along One Revolution of a Circle for Various Velocities with R = 10")
% xlabel("Position Along Circle (Revolution)")
% ylabel("Path Tracking Error ||r|| (m)")
% legend("Velocity = 0.5 m/s", "Velocity = 1 m/s", "Velocity = 2 m/s", "Velocity = 5 m/s")


omega = 1/10;
T = 2*pi/omega;
T_steps = ceil(T*1000);

r_10_v_1_x = r_10_v_1(1:T_steps,1);
r_10_v_1_y = r_10_v_1(1:T_steps,2);
r_10_v_1_z = r_10_v_1(1:T_steps,3);
norm_r_10_v_1 = sqrt(r_10_v_1_x.^2 + r_10_v_1_y.^2 + r_10_v_1_z.^2);
avg_norm_r_10_v_1 = mean(norm_r_10_v_1);
circle_r_10_v_1 = linspace(0,length(r_10_v_1_x)-1,length(r_10_v_1_x))/(length(r_10_v_1_x)-1);

omega = 1/1;
T = 2*pi/omega;
T_steps = ceil(T*1000);

r_1_v_1_x = r_1_v_1(1:T_steps,1);
r_1_v_1_y = r_1_v_1(1:T_steps,2);
r_1_v_1_z = r_1_v_1(1:T_steps,3);
norm_r_1_v_1 = sqrt(r_1_v_1_x.^2 + r_1_v_1_y.^2 + r_1_v_1_z.^2);
avg_norm_r_1_v_1 = mean(norm_r_1_v_1);
circle_r_1_v_1 = linspace(0,length(r_1_v_1_x)-1,length(r_1_v_1_x))/(length(r_1_v_1_x)-1);

omega = 1/2;
T = 2*pi/omega;
T_steps = ceil(T*1000);

r_2_v_1_x = r_2_v_1(1:T_steps,1);
r_2_v_1_y = r_2_v_1(1:T_steps,2);
r_2_v_1_z = r_2_v_1(1:T_steps,3);
norm_r_2_v_1 = sqrt(r_2_v_1_x.^2 + r_2_v_1_y.^2 + r_2_v_1_z.^2);
avg_norm_r_2_v_1 = mean(norm_r_2_v_1);
circle_r_2_v_1 = linspace(0,length(r_2_v_1_x)-1,length(r_2_v_1_x))/(length(r_2_v_1_x)-1);

omega = 1/20;
T = 2*pi/omega;
T_steps = ceil(T*1000);

r_20_v_1_x = r_20_v_1_fixed(1:T_steps,1);
r_20_v_1_y = r_20_v_1_fixed(1:T_steps,2);
r_20_v_1_z = r_20_v_1_fixed(1:T_steps,3);
norm_r_20_v_1 = sqrt(r_20_v_1_x.^2 + r_20_v_1_y.^2 + r_20_v_1_z.^2);
avg_norm_r_20_v_1 = mean(norm_r_20_v_1);
circle_r_20_v_1 = linspace(0,length(r_20_v_1_x)-1,length(r_20_v_1_x))/(length(r_20_v_1_x)-1);

xs = [1,2,10,20];
ys = [avg_norm_r_1_v_1,avg_norm_r_2_v_1,avg_norm_r_10_v_1,avg_norm_r_20_v_1];

figure
plot(xs,ys)
title("Average Path Tracking Error Along One Revolution of a Circle for Various Radii with v = 1 m/s")
xlabel("Radius (m)")
ylabel("Average Path Tracking Error ||r|| (m)")

% figure
% plot(circle_r_1_v_1,norm_r_1_v_1)
% hold on
% plot(circle_r_2_v_1,norm_r_2_v_1)
% hold on
% plot(circle_r_10_v_1,norm_r_10_v_1)
% hold on
% plot(circle_r_20_v_1,norm_r_20_v_1)
% title("Path Tracking Errors Along One Revolution of a Circle for Various Radii with v = 1 m/s")
% xlabel("Position Along Circle (Revolution)")
% ylabel("Path Tracking Error ||r|| (m)")
% legend("Radius = 1 m", "Radius = 2 m", "Radius = 10 m", "Radius = 20 m")








