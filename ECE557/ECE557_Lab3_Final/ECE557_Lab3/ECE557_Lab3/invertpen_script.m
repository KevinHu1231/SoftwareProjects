%% Section 3.1
%% 3.1.2
syms m g M l
A_lin = [0 1 0 0; 0 0 m*g/M 0; 0 0 0 1; 0 0 (M+m)*g/(M*l) 0];
e = eig(A_lin)

%e =
% 
%                         0
%                         0
%-((g*(M + m))/(M*l))^(1/2)
% ((g*(M + m))/(M*l))^(1/2)

% Linearized control system is unstable due to negative third eigenvalue.

%% 3.1.4/3.1.5
M = 1.0731;
m = 0.2300;
l = 0.3302;
g = 9.8;
%[sizes, x0, states] = invertpen([],[],[],0);
% [A,B,C,D] = linmod('invertpen',[0; 0; 0; 0],0)
A = [0 1 0 0; 0 0 m*g/M 0; 0 0 0 1; 0 0 (M+m)*g/(M*l) 0];;
B = [0;1/M;0;1/l/M];
C=[1,0,0,0;0,0,1,0];
D=[0;0];

% States = [x1 x3 x2 x4]T

%% Section 3.2
%% 3.2.1
poles_1 = [-1,-2,-3,-4];
SF_controller_1 = place(A,B,poles_1)
K = -1*SF_controller_1;

% SF_controller_1 = -0.8678   25.4587   -1.8078    4.1403


% %% 3.2.2
% poles_2 = [-0.8,-1.2+1i,-1.2-1i,-2];
% SF_controller_2 = place(A,B,poles_2)
% K = -1*SF_controller_2;
% 
% % SF_controller_2 = -0.1412   16.6297   -0.3859    1.9700

poles_ob = [-18,-19+1i,-19-1i,-20];
FO_observer = place(A.',-C.',poles_ob).'
L = -1*FO_observer;
%% 3.2.4
q1 = 0.05;
q2 = 2000;
Q = [q1 0 0 0; 0 0 0 0; 0 0 q2 0; 0 0 0 0];
R = 10;

q1 = 0.05; R = 5e-4; q2 = 5;

Q = [q1 0 0 0; 0 0 0 0; 0 0 q2 0; 0 0 0 0];
K = -1*lqr(A,B,Q,R)
% 
% % q2 = 5; R = 0.5; q1 = 0.005
% % K = -0.1000   27.6408   -0.5509    4.6051

% q2 = 5; R = 0.5; q1 = 0.1
% K = -0.4472   29.6951   -1.2429    4.9864

% q2 = 5; R = 0.5; q1 = 15
% K = -5.4772   41.5732   -5.6741    7.1821

% q1 = 0.05; R = 0.5; q2 = 1
% K = -0.3162   28.7703   -1.0162    4.8427

% q1 = 0.05; R = 0.5; q2 = 200
% K = -0.3162   39.2541   -1.3073    5.6990

% q1 = 0.05; R = 0.5; q2 = 2000
% K = -0.3162   79.7095   -2.0785    8.1972

% q1 = 0.05; q2 = 5; R = 5*10^-5
% K = 1.0e+03 * [-0.0316    6.3688   -0.2025    0.1340]

% q1 = 0.05; q2 = 5; R = 5*10^-4
% K = 1.0e+03 * [-0.0100    2.0280   -0.0641    0.0591]

% q1 = 0.05; q2 = 5; R = 10
% K = -0.0707   33.1426   -0.5422    5.0237

% Extract data
% f1 = figure(1)
% subplot(2,2,1)
% plot(output(1,:),output(2,:))
% xlabel('Time (s)')
% ylabel('y')
% subplot(2,2,2)
% plot(output(1,:),output(3,:))
% xlabel('Time (s)')
% ylabel('y dot')
% subplot(2,2,3)
% plot(output(1,:),output(4,:))
% xlabel('Time (s)')
% ylabel('theta')
% subplot(2,2,4)
% plot(output(1,:),output(5,:))
% xlabel('Time (s)')
% ylabel('theta dot')




%% 3.2.2
poles_2 = [-0.8,-1.2+1i,-1.2-1i,-2];
SF_controller_2 = place(A,B,poles_2)

% SF_controller_2 = -0.1412   16.6297   -0.3859    1.9700

%% 3.2.3
poles_ob = [-18,-19+1i,-19-1i,-20];
FO_observer = place(A.',-C.',poles_ob).'
L = -1*FO_observer;

% FO_observer =

%  -37.9883    0.4124
%   -0.0003  -38.0117
% -361.7224    7.6908
%    0.0492 -396.3178

% %% 3.2.4
% q1 = 0.05;
% q2 = 2000;
% Q = [q1 0 0 0; 0 q2 0 0; 0 0 0 0; 0 0 0 0];
% R = 10;
% K = lqr(A,B,Q,R)
% 
% % q2 = 5; R = 0.5; q1 = 0.005
% % K = -0.1000   27.6408   -0.5509    4.6051

% q2 = 5; R = 0.5; q1 = 0.1
% K = -0.4472   29.6951   -1.2429    4.9864

% q2 = 5; R = 0.5; q1 = 15
% K = -5.4772   41.5732   -5.6741    7.1821

% q1 = 0.05; R = 0.5; q2 = 1
% K = -0.3162   28.7703   -1.0162    4.8427

% q1 = 0.05; R = 0.5; q2 = 200
% K = -0.3162   39.2541   -1.3073    5.6990

% q1 = 0.05; R = 0.5; q2 = 2000
% K = -0.3162   79.7095   -2.0785    8.1972

% q1 = 0.05; q2 = 5; R = 5*10^-5
% K = 1.0e+03 * [-0.0316    6.3688   -0.2025    0.1340]

% q1 = 0.05; q2 = 5; R = 5*10^-4
% K = 1.0e+03 * [-0.0100    2.0280   -0.0641    0.0591]

% q1 = 0.05; q2 = 5; R = 10
% K = -0.0707   33.1426   -0.5422    5.0237
% %% Section 4.1
% %% 4.1.1
% %% For SF_controller_1
% y0 = 0.5;
% theta0 = 0.175;
% y_dot0 = 0.1;
% theta_dot0 = 0;
% x_init = [y0, y_dot0, theta0, theta_dot0].';
% simulate(x_init, SF_controller_1, A, B);
% %% SF_controller_2
% simulate(x_init, SF_controller_2, A, B);
% %% 4.1.2
% L = FO_observer;

%% Section 4.2.1
K = -SF_controller_2
%% 

%Extract data
output = load('out.mat')
output_o = load('out_observe.mat')
output = output.output;
outputo = output_o.output_o;
f1 = figure(1)
subplot(2,2,1)
plot(output(1,:),output(2,:))
hold on;
plot(outputo(1,:),outputo(2,:))
title('Cart Position (y)')
xlabel('Time (s)')
ylabel('Position (m)')
legend('state','observation')
subplot(2,2,2)
plot(output(1,:),output(3,:))
hold on;
plot(outputo(1,:),outputo(2,:))
title('Cart Velocity (y dot)')
xlabel('Time (s)')
ylabel('Linear Velocity (m/s)')
legend('state','observation')
subplot(2,2,3)
plot(output(1,:),output(4,:))
hold on;
plot(outputo(1,:),outputo(2,:))
title('Pendulum Angle (theta)')
xlabel('Time (s)')
ylabel('Angle (rad)')
legend('state','observation')
subplot(2,2,4)
plot(output(1,:),output(5,:))
hold on;
plot(outputo(1,:),outputo(2,:))
title('Pendulum Velocity (theta dot)')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
legend('state','observation')
%% Section 4.2.4
K = [0.4472,1.2429,-29.6951,-4.9864]
