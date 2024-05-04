clear all; clc;

%% Section 4.1 Robot structure
% Define DH parameters
DH = [0 0.76 0 pi/2;
    0 -0.2365 0.4323 0;
    0 0 0 pi/2;
    pi/4 0.4318 0 -pi/2;
    -pi/3 0 0 pi/2;
    0 0.20 0 0];

% Initialize the robot object
myrobot = mypuma560(DH);

%% Section 4.2 Plot sample joint space trajectory
% Instantiate joint variables
theta1 = linspace(0, pi, 200)
theta2 = linspace(0, pi/2, 200)
theta3 = linspace(0, pi, 200)
theta4 = linspace(pi/4, 3*pi/4, 200)
theta5 = linspace(-pi/3, pi/3, 200)
theta6 = linspace(0, 2*pi, 200)
q = [theta1.', theta2.', theta3.', theta4.', theta5.', theta6.']

% Plot the trajectory
plot(myrobot, q)

%% Section 4.3 Forward Kinematics

% Use forward kinematics to find the end effector positions
o = zeros(200,3)
for i=1:200
    H = forward(q(i,:), myrobot)
    o(i,:) = H(1:3,4)
end

% Plot end effector positions and check it matches
plot3(o(:,1),o(:,2),o(:,3), 'r')
hold on;
plot(myrobot,q)

%% Section 4.4 Inverse Kinematics

% Check that inverse kinematics works
H = [cos(pi/4) -sin(pi/4) 0 0.20; sin(pi/4) cos(pi/4) 0 0.23; 0 0 1 0.15; 0 0 0 1]
inverse(H, myrobot)

% Create vector of desired position and orientations
d1 = linspace(0.10,0.30,100);
d2 = linspace(0.23,0.30,100);
d3 = linspace(0.15,1.00,100);
d = [d1; d2; d3].';
R = [cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1];

% Inverse computation loop
q = zeros(100,6);
for i = 1:100
    H = eye(4);
    H(1:3, 1:3) = R;
    H(1:3, 4) = d(i,:);
    q_i = inverse(H, myrobot);
    q(i,:) = q_i;
end

% Get end effector pose from joint variables
o = zeros(100,3);
for i=1:100
    H = forward(q(i,:), myrobot);
    o(i,:) = H(1:3,4);
end

% Plot the trajectory
plot3(o(:,1),o(:,2),o(:,3), 'r')
hold on;
plot(myrobot,q)

