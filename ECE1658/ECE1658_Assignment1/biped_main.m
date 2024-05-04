%%%%%%%%%%%%%%%%%%%%%%%
% ECE1658: Assignment 1
% Annalisa Cognigni and Kevin Hu
clear all

% Link lengths
l1 = 0.5;
l2 = 0.5;
l3 = 0.3;

% Masses
m1 = 0.05;
m2 = 0.5;
m3 = 0.3;
m4 = 0.5;
m5 = 0.05;
m6 = 0.5; 

% Gravity
g = 9.81;

% Defining symbolic variables for angles (q), angular velocities (qdot) and
% forces (tau)
syms t q1 q2 q3 q4 q5 real
syms x1 x2 real
syms q1dot q2dot q3dot q4dot q5dot real
syms x1dot x2dot real
syms tau1 tau2 tau3 tau4 tau5 real

q = [q1; q2; q3; q4; q5];
qdot = [q1dot; q2dot; q3dot; q4dot; q5dot];
tau = [tau1; tau2; tau3; tau4; tau5];

qbar = [q; x1; x2];
qbardot = [qdot; x1dot; x2dot];

% Rotation matrices for frames 1 through 6
C12 = rotation_mat([1, 0, 0, 0, 0]*q);
C23 = rotation_mat([0, 1, 0, 0, 0]*q);
C34 = rotation_mat([0, 0, 1, 0, 0]*q);
C45 = rotation_mat([0, 0, 0, 1, 0]*q);
C36 = rotation_mat([0, 0, 0, 0, 1]*q);

% Positions of masses
r1 =  [x1; x2];
r2 = r1 + C12*[l1; 0];
r3 = r2 + C12*C23*[l2; 0];
r4 = r3 + C12*C23*C34*[l2; 0];
r5 = r4 + C12*C23*C34*C45*[l1; 0];
r6 = r3 + C12*C23*C36*[l3; 0];

% Velocities of masses
r1dot = jacobian(r1, qbar)*qbardot;
r2dot = jacobian(r2, qbar)*qbardot;
r3dot = jacobian(r3, qbar)*qbardot;
r4dot = jacobian(r4, qbar)*qbardot;
r5dot = jacobian(r5, qbar)*qbardot;
r6dot = jacobian(r6, qbar)*qbardot;

% Kinetic energy of system
K = 0.5*m1*(r1dot'*r1dot) + 0.5*m2*(r2dot'*r2dot) + 0.5*m3*(r3dot'*r3dot) + ...
    0.5*m4*(r4dot'*r4dot) + 0.5*m5*(r5dot'*r5dot) + 0.5*m6*(r6dot'*r6dot);

Dbar = hessian(K, qbardot);
D = Dbar(1:5, 1:5);

% Potential energy of system
P = m1*g*r1(2,1) + m2*g*r2(2,1) + m3*g*r3(2,1) ...
    + m4*g*r4(2,1) + m5*g*r5(2,1) + m6*g*r6(2,1);
G = jacobian(P,q)';

% Input Matrix
B = [0 0 0 0;
     1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

% Christoffel coefficients
C = zeros(5,5);
C = sym(C);

for k=1:5
    for j=1:5
        for i=1:5
            C(k,j) = C(k,j) + (0.5*(diff(D(k,j), q(i)) + diff(D(k,i), q(j)) - diff(D(i,j), q(k))))*qdot(i);
        end
    end
end

px = r5 - r1;

E = [jacobian(px, q) eye(2,2)];

% Function definitions
Dfun = matlabFunction(D, 'Vars', {[q1; q2; q3; q4; q5]});
Dbarfun = matlabFunction(Dbar, 'Vars', {[q1; q2; q3; q4; q5]});
Efun = matlabFunction(E, 'Vars', {[q1; q2; q3; q4; q5]});
Gfun = matlabFunction(G, 'Vars', {[q1; q2; q3; q4; q5]});
Cfun = matlabFunction(C, 'Vars', {[q1; q2; q3; q4; q5; q1dot; q2dot; q3dot; q4dot; q5dot]});

data.D = Dfun;
data.Dbar = Dbarfun;
data.E = Efun;
data.G = Gfun;
data.C = Cfun;
data.B = B;

% Reference angles
q2ref = pi/6;
q3ref = pi+pi/6;
q4ref = -pi/6;
q5ref = -pi/6;

qref = [q2ref;
        q3ref;
        q4ref;
        q5ref];

% % Solvng for gains
% syms lambda Kd Kp
% 
% a = solve(lambda^2 + Kd*lambda + Kp);
% solve(a==[-20; -20])

Kd = 40;
Kp = 400;

H = [zeros(4,1) eye(4,4)];

data.qref = qref;
data.H = H;
data.Kp = Kp;
data.Kd = Kd;

B_ann = [1 0 0 0 0];

% VHC condition is confirmed
th1 = sym('th1', 'real');
VHC_condition  = simplify(B_ann*data.D([th1; qref])*[1; 0; 0; 0; 0])
%% 

% Creation of the ground impact events function
ground_impact=matlabFunction(eval(px(2)),1,-1,...
    'File','ground_impact','Vars',{t,[q;qdot]},...
    'Outputs',{'value', 'isterminal','direction'});

ops = odeset('reltol',1e-8,'abstol',1e-8,'Events',@ground_impact); 

% Initial conditions for integration
q0=[pi/3;pi/4;pi-pi/6;-pi/3;-pi/6]; 
qdot0=zeros(5,1);

% Simulate Pre Impact
[t1,x1,te1,xe1]=ode45(@(t,x) biped(t,x,data),0:5e-2:10,[q0;qdot0],ops);

% Post Impact State
post_impact_state = impact_map(xe1',data);

% Simulate Post Impact
[t2,x2,te2,xe2]=ode45(@(t,x) biped(t,x,data),te1:5e-2:te1+10,post_impact_state,ops);

%% Animation of the simulation results
Axis=[-1 4 0 2];
axis equal;
Time=text(1,1.8,['time= ',num2str(t1(1)),' secs']);
axis(Axis);
q=q0;

q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
xdata=0;
ydata=0;
l=[l1 l2 l2 l1 l3];
Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
for j=1:4
    xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
    ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
end
xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
ydata=[ydata ydata(3)+l(5)*sin(Q(5))];

link1=line([xdata(1) xdata(2)],[ydata(1) ydata(2)],'color','red','linewidth',2);
link2=line([xdata(2) xdata(3)],[ydata(2) ydata(3)],'color','red','linewidth',2);
link3=line([xdata(3) xdata(4)],[ydata(3) ydata(4)],'linewidth',2);
link4=line([xdata(4) xdata(5)],[ydata(4) ydata(5)],'linewidth',2);
link5=line([xdata(3) xdata(6)],[ydata(3) ydata(6)],'linewidth',2);

fprintf('\n Animation is ready...\n')
ref=0; % This variable keeps track of the position of the stance foot accross multiple steps

animation_slowdown_factor=4; % >1 means slow down
for k=2:length(t1)
    t0=clock;
    drawnow;
    q=x1(k,1:5)';
    q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
    Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
    xdata=ref;
    ydata=0;
    for j=1:4
        xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
        ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
    end
    xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
    ydata=[ydata ydata(3)+l(5)*sin(Q(5))];
    set(link1,'xdata',[xdata(1) xdata(2)],'ydata',[ydata(1) ydata(2)]);
    set(link2,'xdata',[xdata(2) xdata(3)],'ydata',[ydata(2) ydata(3)]);
    set(link3,'xdata',[xdata(3) xdata(4)],'ydata',[ydata(3) ydata(4)]);
    set(link4,'xdata',[xdata(4) xdata(5)],'ydata',[ydata(4) ydata(5)]);
    set(link5,'xdata',[xdata(3) xdata(6)],'ydata',[ydata(3) ydata(6)]);
    set(Time,'String',['time= ',num2str(round(t1(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t1(k)-t1(k-1))
    end
end

ref=xdata(5);

for k=2:length(t2)
    t0=clock;
    drawnow;
    q=x2(k,1:5)';
    q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
    Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
    xdata=ref;
    ydata=0;
    for j=1:4
        xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
        ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
    end
    xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
    ydata=[ydata ydata(3)+l(5)*sin(Q(5))];
    set(link1,'xdata',[xdata(1) xdata(2)],'ydata',[ydata(1) ydata(2)]);
    set(link2,'xdata',[xdata(2) xdata(3)],'ydata',[ydata(2) ydata(3)]);
    set(link3,'xdata',[xdata(3) xdata(4)],'ydata',[ydata(3) ydata(4)]);
    set(link4,'xdata',[xdata(4) xdata(5)],'ydata',[ydata(4) ydata(5)]);
    set(link5,'xdata',[xdata(3) xdata(6)],'ydata',[ydata(3) ydata(6)]);
    set(Time,'String',['time= ',num2str(round(t2(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t2(k)-t2(k-1))
    end
end


%%
function C = rotation_mat(theta)
    C = [cos(theta) -sin(theta);
         sin(theta) cos(theta)];
end