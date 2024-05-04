%% 3b
% Initialize parameters for A and B matrices
M = 1.0731;
m = 0.2300;
l = 0.3302;
g = 9.8;
alpha_1 = -7.7443;
alpha_2 = 1.7265;
%eq = [y_bar 0 0 0];

% Initialize A and B matrices
A = [0 0 1 0; 0 0 0 1; 0 m*g/M alpha_1/M 0; 0 (M+m)*g/(l*M) alpha_1/(l*M) 0];
B = [0; 0; alpha_2/M; alpha_2/(l*M)];
C = [1 0 0 0; 0 1 0 0];
D = [0; 0];

%% 3c

% Calculate K based on desired controller eigenvalues
poles = [-2,-4,-10,-20];
K = -place(A,B,poles);

% K =
% 
%    33.5077  -98.0920   34.6425  -17.3463

%% 3d

% Calculate L based on desired observer eigenvalues
poles_ob = [-40,-40,-50,-50];
L = (-place(A.',-C.',poles_ob)).';

% L =
% 
% 
% 
%     82.8   0.0
%    -21.9   90.0
%     1402.6 2.1
%    -1809.3 2036.0

%% 3f
%% y

% Plot for y and yhat for controller and observer
figure(1)
plot(t,y(:,1))
hold on
%yhatybar = y(:,2)+y(:,3);
%plot(t,yhatybar)
plot(t,y(:,2))
hold on
plot(t,y(:,3))
title("Y and Observer Estimate as a Function of Time")
xlabel("Time (s)")
ylabel("Position (mm)")
legend("y","yhat","yd")

%% theta

% Plot for theta and thetahat for controller and observer
figure(2)
plot(t,theta(:,1))
hold on
plot(t,theta(:,2))
title("Theta and Observer Estimate as a Function of Time")
xlabel("Time (s)")
ylabel("Angle (degrees)")
legend("theta","theta hat")

%% y_dot

% Plot for ydot and ydot hat for controller and observer
figure(3)
plot(t,y_dot(:,1))
hold on
plot(t,y_dot(:,2))
title("Y Dot and Observer Estimate as a Function of Time")
xlabel("Time (s)")
ylabel("Velocity (mm/s)")
legend("ydot","ydot hat")

%% theta_dot

% Plot for thetadot and thetadot hat for controller and observer
figure(4)
plot(t,theta_dot(:,1))
hold on
plot(t,theta_dot(:,2))
title("Theta Dot and Observer Estimate as a Function of Time")
xlabel("Time (s)")
ylabel("Angular Velocity (degrees/s)")
legend("thetadot","thetadot hat")

%% 3g

% Initialize Q and R
q1 = 1;
q2 = 4;
r = 0.0005;
Q = [q1 0 0 0; 0 q2 0 0; 0 0 0 0; 0 0 0 0];
R = r;

% Calculate K for optimal controller
K = -1*lqr(A,B,Q,R)

% K =
% 
%    44.7214 -136.7657   39.1388  -18.6723

%% y
figure(1)
plot(t,y(:,1))
hold on
plot(t,y(:,2))
hold on
plot(t,y(:,3))
title("Y and Observer Estimate as a Function of Time for Optimal Controller")
xlabel("Time (s)")
ylabel("Position (mm)")
legend("y","yhat","yd")

%% theta
figure(2)
plot(t,theta(:,1))
hold on
plot(t,theta(:,2))
title("Theta and Observer Estimate as a Function of Time for Optimal Controller")
xlabel("Time (s)")
ylabel("Angle (degrees)")
legend("theta","thetahat")

%% y simulated
figure(1)
plot(t,y(:,1))
title("Y (Simulated) as a Function of Time for Optimal Controller")
xlabel("Time (s)")
ylabel("Position (mm)")
legend("y")

%% theta simulated
figure(2)
plot(t,theta(:,1))
title("Theta (Simulated) as a Function of Time for Optimal Controller")
xlabel("Time (s)")
ylabel("Angle (degrees)")
legend("theta")
