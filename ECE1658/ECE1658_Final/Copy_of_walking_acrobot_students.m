%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% FINAL PROJECT
%% VHC for walking acrobot
%% PARTIAL CODE TO GET STUDENTS STARTED ON THE PROJECT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc

number_steps=25;
symbolic_computations=1;
testvhc=1;
optimize=0;
lencost=1;

%For polynomial
k_poly = 6;
%% Physical parameters
l=1;
lc=0.5;
m=1;
Iz=1/12*m*l^2;
g=9.81;
psi=deg2rad(2);
%% Control parameters
Kp=20^2;
Kd=20*2;

%% Symbolic computations
if symbolic_computations
    % Define symbolic variables
    fprintf('\n Initializing symbolic variables...\n')
    % syms l lc Iz m g real
    syms t q1 q2 x1 x2 q1dot q2dot x1dot x2dot tau real
    
    q=[q1;q2];x=[x1;x2]; qbar=[q;x];
    qdot=[q1dot;q2dot];  xdot=[x1dot;x2dot]; qbardot=[qdot;xdot];
    
    fprintf('\n Symbolic model computation...\n')
    
    % Define centres of mass of two links
    rc1=x+lc*[cos(q1);sin(q1)];
    rc2=x+l*[cos(q1);sin(q1)]+lc*[cos(q1+q2);sin(q1+q2)];
    
    % Compute time derivatives of centres of mass
    rc1dot=jacobian(rc1,qbar)*qbardot;
    rc2dot=jacobian(rc2,qbar)*qbardot;
    
    % Define the total kinetic energy of the robot
    K=1/2*m*(rc1dot'*rc1dot+rc2dot'*rc2dot)+1/2*Iz*(q1dot^2+(q1dot+q2dot)^2);
    K=simplify(K);
    
    % Extract the square symmetric matrix of the kinetic energy
    Dbar=simplify(hessian(K,qbardot));
    
    % Extract the matrix of the kinetic energy of the pinned robot
    D = Dbar(1:2,1:2);
    
    % Define the potential energy of the pinnedrobot
    P = m*g*(lc*sin(q1)+l*sin(q1)+lc*sin(q1+q2));
    %psi=deg2rad(incline_degrees);
    gvector=[cos(psi) -sin(psi);sin(psi) cos(psi)]*[0; -1];
    P=-(m*g*lc*[cos(q1);sin(q1)]+m*g*[l*cos(q1)+lc*cos(q1+q2);l*sin(q1)+lc*sin(q1+q2)])'*gvector;
    P=simplify(P);
    % Input matrix of pinned robot
    B=[0;1];
    
    % Computation of matrix C(q,qdot) of pinned robot
    C = sym(zeros(2,2));
    for i=1:2
        for j=1:2
            for k=1:2
                C(k,j)=C(k,j)+1/2*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)))*qdot(i);
            end
        end
    end
    
    % Computation of gradient of the potential for the pinned robot
    
    G = jacobian(P,q)';
    
    % Computation of qddot (pinned robot)
    
    %     qddot = simplify(inv(D)*(-C*qdot-G+B*tau));
    Dfun=matlabFunction(D,'Vars',{q});
    Gfun=matlabFunction(G,'Vars',{q});
    Cfun=matlabFunction(C,'Vars',{[q;qdot]});
    fprintf('\n Impact map computation...\n')
    
    %% Impact map
    
    % Relabelling map
    
    T=-eye(2)*q + [pi;2*pi];
    Delta2=[T;-eye(2)*qdot];
    
    % First impact map
    
    px=l*[cos(q1)+cos(q1+q2);sin(q1)+sin(q1+q2)];
    xo=sym(zeros(2,1));
    
    E=[jacobian(px,q), eye(2)];
    Deltaqdot=[eye(2),zeros(2,4)]*inv([Dbar -E';E zeros(2,2)])*...
        [Dbar*[eye(2);jacobian(xo,q)];zeros(2,2)];
    Delta1=eval([q;Deltaqdot*qdot]);
    Delta1=simplify(Delta1);
    
    % Composition of two maps
    Delta=simplify(subs(Delta2,[q;qdot],Delta1));
    Deltafun=matlabFunction(Delta,'Vars',{[q;qdot]});
    save('walking_acrobot_model','D','Dfun','Cfun','Gfun','Deltafun','B');
else
    fprintf('\nLoading robot model...\n')
    load('walking_acrobot_model');
end
%% HERE WRITE YOUR CODE FOR THE VHC DESIGN
% The outcome of this part should be a parameter vector a, whose components
% a_1,ldots,a_k define the polynomial phi_a(theta)=a_1 + a_2 theta + ... +
% a_k theta^(k-1)

fprintf('\nDetermining vhc...\n')
beta = 0.316637;
v1 = -0.894373;
v2 = 1.9;
q_minus = [(pi-beta)/2;pi+beta];
q_plus = [(beta+pi)/2;pi-beta];
q1_tilde = q_plus(1)-q_minus(1);

dTqminus = subs(jacobian(T,q),q,q_minus);
Deltaqdotqminus = subs(Deltaqdot,q,q_minus);
I = dTqminus*Deltaqdotqminus;

%% Save functions in data structure

I_sym = jacobian(T,q)*Deltaqdot;
Ifun = matlabFunction(I_sym,'Vars',{q});
data.I=Ifun;
B_perp = [1, 0];
data.B_perp = B_perp;

% This is the data structure to be passed to various functions
% You will need to add extra information to it.
data.Kp=Kp;
data.Kd=Kd;
data.D=Dfun;
data.C=Cfun;
data.G=Gfun;
data.B=B;

%% Solve linear system of equations

%Polynomial terms

A = zeros(k_poly,k_poly);
A(1,1) = 1;
A(2,:) = 1;
A(3,:) = 0.5.^(0:k_poly-1);
A(4,2) = 1;
A(5,:) = 0:k_poly-1;
A(6,:) = A(5,:);
A(6,2:end) = A(6,2:end).*A(3,1:(end-1));

f_v1 = eval(-(q1_tilde*(-I(2,1)*q1_tilde + I(2,2)*v1))/(-I(1,1)*q1_tilde + I(1,2)*v1));
b = [q_plus(2);q_minus(2);pi;f_v1;v1;v2];
a_vec = A\b;

%% 4.2 Optimization

if optimize
    fprintf('\n Optimizing vhc...\n')
    % Initial Condition
    a_vec = [a_vec;zeros(k-6,1)];
    x0 = [a_vec;beta;v1;v2];
    
    % Linear Equality Constraints
    Aeq = [A,zeros(k_poly,3)];
    beq = b;
    
    e = 10e-6;
    % Linear Inequality Constraints
    Aineq = zeros(2,length(x0));
    bineq = zeros(2,1);
    
    % Beta
    Aineq(1,(length(x0)-2)) = -2;
    % v1
    Aineq(1,(length(x0)-1)) = 1;
    % Beta
    Aineq(2,(length(x0)-2)) = 2;
    % v2
    Aineq(2,(length(x0))) = -1;
    bineq(2) = -e;
    
    % Generate random samples
    num_samples=100;
    data.samples=rand(num_samples,1);
    
    % Recreate functions that work with variable a, q+ and q~
    phi_a = @(theta,a) polyval(a,theta);
    phiprime_a = @(theta,a) polyval(polyder(a),theta);
    phipprime_a = @(theta,a) polyval(polyder(polyder(a)),theta);
    
    sigma_a = @(theta,a,q1plus,q1tilde) [q1plus-theta*q1tilde; phi_a(theta,a)];
    sigmaprime_a = @(theta,a,q1tilde) [-q1tilde + 0*theta; phiprime_a(theta,a)];
    sigmapprime_a = @(theta,a) [0+0*theta; phipprime_a(theta,a)];
    
    % Add to data structure to pass into functions
    data.phi_a = phi_a;
    data.phiprime_a = phiprime_a;
    data.phipprime_a = phipprime_a;
    
    data.sigma_a = sigma_a;
    data.sigmaprime_a = sigmaprime_a;
    data.sigmapprime_a = sigmapprime_a;
    
    % % Recreate functions that work with variable a, q+ and q~
    % psi_1_a = @(theta,a,q1plus,q1tilde) -(B_perp*Gfun(sigma_a(theta,a,q1plus,q1tilde))) / (B_perp*Dfun(sigma_a(theta,a,q1plus,q1tilde))*sigmaprime_a(theta,a,q1tilde));
    % psi_2_a = @(theta,a,q1plus,q1tilde) -(B_perp*(Dfun(sigma_a(theta,a,q1plus,q1tilde))*sigmapprime_a(theta,a,q1tilde) + Cfun([sigma_a(theta,a,q1plus,q1tilde);sigmaprime_a(theta,a,q1tilde)])*sigmaprime_a(theta,a,q1tilde))) / (B_perp*Dfun(sigma_a(theta,a,q1plus,q1tilde))*sigmaprime_a(theta,a,q1tilde));
    % psi_1_a = @(theta,a,q1plus,q1tilde) arrayfun(psi_1_a,theta);
    % psi_2_a = @(theta,a,q1plus,q1tilde) arrayfun(psi_2_a,theta);
    % 
    % % Add to data structure to pass into functions
    % data.psi_1_a = psi_1_a;
    % data.psi_2_a = psi_2_a;
    % 
    % Nonlinear Constraints
    nonlcon = @(x) nlcons(x,data,1);
    
    % Cost function
    if lencost 
        fun = @(x) lengthcost(x,data);
    else
        fun = @(x) flatcost(x);
    end

    % Bounds
    lb = [];
    ub = [];

    options = optimoptions('fmincon','Display','iter');
    x = fmincon(fun,x0,Aineq,bineq,Aeq,beq,lb,ub,nonlcon,options);
    
    a_vec = x(1:(length(x)-3));
    beta = x(length(x)-2);
    v1 = x(length(x)-1);
    v2 = x(length(x));

    q_minus = [(pi-beta)/2;pi+beta];
    q_plus = [(beta+pi)/2;pi-beta];
    q1_tilde = q_plus(1)-q_minus(1);

end

%% HERE WE DEFINE THE FUNCTION phi_a AND ITS DERIVATIVES
a=flip(a_vec);
phi=@(theta) polyval(a,theta);
phiprime=@(theta) polyval(polyder(a),theta);
phipprime=@(theta) polyval(polyder(polyder(a)),theta);

% Using phi and its derivatives, below you should define functions sigma,
% sigmaprime, sigmapprime.
sigma=@(theta) [q_plus(1) - theta*q1_tilde; phi(theta)];
sigmaprime=@(theta) [-q1_tilde + 0*theta; phiprime(theta)];
sigmapprime=@(theta) [0 + 0*theta; phipprime(theta)];

data.phi=phi;
data.phiprime=phiprime;
data.phipprime=phipprime;
data.sigma=sigma;
data.sigmaprime=sigmaprime;
data.sigmapprime=sigmapprime;

%% HERE WRITE CODE TO TEST WHETHER YOUR VHC WORKS
if testvhc
N = 100;
theta_vals = linspace(0,1,N+1);
step = 1/N;
qs = sigma(theta_vals);
sps = sigmaprime(theta_vals);
spps = sigmapprime(theta_vals);

%Plot Set W
W1_q1 = linspace(0,pi/2,1001);
W1_q1_l = -2*W1_q1 + 2*pi;
W1_q1_u = 3*pi*ones(1,1001);

figure
plot(W1_q1,W1_q1_l)
hold on
plot(W1_q1,W1_q1_u)
hold on

x2 = [W1_q1, fliplr(W1_q1)];
inBetween = [W1_q1_l, fliplr(W1_q1_u)];
fill(x2, inBetween, 'y');

W2_q1 = linspace(pi/2,pi,1001);
W2_q1_l = -pi*ones(1,1001);
W2_q1_u = -2*W2_q1 + 2*pi;

plot(W2_q1,W2_q1_l)
hold on
plot(W2_q1,W2_q1_u)
hold on

x2 = [W2_q1, fliplr(W2_q1)];
inBetween = [W2_q1_l, fliplr(W2_q1_u)];
fill(x2, inBetween, 'y');

%Plot Curve and Points
plot(qs(1,:),qs(2,:),'k');
hold on
plot(q_plus(1),q_plus(2),'r*')
hold on
plot(q_minus(1),q_minus(2),'b*')
hold on
plot(pi/2,pi,'g*')
hold on

%Calculate tangent and tangent Lines
qplus = sigma(0);
qplusp = sigmaprime(0);
qplus0 = qplus - qplusp;
qplus1 = qplus + qplusp;

qminus = sigma(1);
qminusp = sigmaprime(1);
qminus0 = qminus - qminusp;
qminus1 = qminus + qminusp;

qbar = sigma(0.5);
qbarp = sigmaprime(0.5);
qbar0 = qbar - qbarp;
qbar1 = qbar + qbarp;

% Plot tangent lines

plot([qplus0(1),qplus1(1)],[qplus0(2),qplus1(2)],'r')
hold on
plot([qbar0(1),qbar1(1)],[qbar0(2),qbar1(2)],'g')
hold on
plot([qminus0(1),qminus1(1)],[qminus0(2),qminus1(2)],'b')
hold on

% Check if regular VHC
transverses = zeros(N+1);
for i = 1:N+1
    D = Dfun(qs(:,i));
    transverse = B_perp*D*sps(:,i);
    transverses(i) = transverse;
end
figure
plot(theta_vals,transverses)

end

%% Define functions

psi_1 = @(theta) -(B_perp*Gfun(sigma(theta))) / (B_perp*Dfun(sigma(theta))*sigmaprime(theta));
psi_2 = @(theta) -(B_perp*(Dfun(sigma(theta))*sigmapprime(theta) + Cfun([sigma(theta);sigmaprime(theta)])*sigmaprime(theta))) / (B_perp*Dfun(sigma(theta))*sigmaprime(theta));
psi_1 = @(theta) arrayfun(psi_1,theta);
psi_2 = @(theta) arrayfun(psi_2,theta);

M = @(theta) exp(-2*integral(psi_2, 0, theta));
M = @(theta) arrayfun(M,theta);

V_integrand = @(theta) psi_1(theta)*M(theta);
V_integrand = @(theta) arrayfun(V_integrand,theta);

V = @(theta) -integral(V_integrand,0,theta);
V = @(theta) arrayfun(V,theta);

Mminus = M(1);
Vminus = V(1);
delta = dot(sigmaprime(0),I*sigmaprime(1)) / (sigmaprime(0).' * sigmaprime(0));

%%
if testvhc
Vs = V(theta_vals);
Vmax = max(Vs);

constraint_1 = delta^2/Mminus;
constraint_2 = ((Vminus*delta^2) / (Mminus - delta^2)) + Vmax;

if ((0 < constraint_1) && (constraint_1 < 1) && (constraint_2 < 0))
    limit_cycle = 'True';
else
    limit_cycle = 'False';
end
words = strcat('Limit Cycle: ', limit_cycle);
disp(words)
end

% VHC is confirmed to work for original design

%% Added code for defining h and related functions for controller
% Symbolic function of h and Jacobians
syms q_1(t) q_2(t) th
phi_sym = poly2sym(a, th);
phi_sym = subs(phi_sym,th,(q_plus(1)-q_1(t))/q1_tilde);

h = q_2(t) - phi_sym;
q_ = [q_1(t);q_2(t)];
dhq = jacobian(h,q_);
ddhq_dt = diff(dhq,t);

h = subs(h,[q_1(t),q_2(t)],[q1,q2]);
dhq = subs(dhq,[q_1(t),q_2(t)],[q1,q2]);

ddhq_dt = subs(ddhq_dt,diff(q_1(t),t),q1dot);
ddhq_dt = subs(ddhq_dt,diff(q_2(t),t),q2dot);
ddhq_dt = subs(ddhq_dt,q_2(t),q2);
ddhq_dt = subs(ddhq_dt,q_1(t),q1);

hfun=matlabFunction(h,'Vars',{q});
dhqfun=matlabFunction(dhq,'Vars',{q});
ddhq_dtfun=matlabFunction(ddhq_dt,'Vars',{[q;qdot]});
data.h = hfun;
data.dhq = dhqfun;
data.ddhq_dt = ddhq_dtfun;

%% NOW YOU CAN SIMULATE THE ROBOT. PLACE YOUR CONTROLLER INSIDE THE FUNCTION acrobot AT THE END OF THIS SCRIPT
ops= odeset('reltol',1e-7,'abstol',1e-7,'Events',@ground_impact);
dt=1/60; % 60 fps; time increment in simulations and animations

fprintf('\n Simulating...\n')
%% DEFINE THE INITIAL CONDITION [q0;qdot0];
q0 = sigma(0);
thetaadot = double(delta*sqrt((-2*Vminus) / (Mminus - delta^2)));
qdot0 = sigmaprime(0)*thetaadot;
T=[];
X=[];
Te=[];
Ie=[];
Xe=[];
post_impact_state=[q0;qdot0];
% Simulate number_steps steps
for step=1:number_steps
    fprintf('\n...step %d...\n',step);
    [t,x,te,xe,ie]=ode45(@(t,x) acrobot(t,x,data),0:dt:10,post_impact_state,ops);
    % Application of the impact map
    impact_state=xe(end,:)';
    post_impact_state=Deltafun(impact_state);
    T{step}=t;
    X{step}=x;
    Ie{step}=ie;
    Te{step}=te;
    Xe{step}=xe;
end

fprintf('\n Setting up animation...\n')
figure(3);

%% Animation of the simulation results
ref=0;time_passed=0;step=1;
Axis=[-1 4 0 2];
Time=text(-1+2,1.8,['time= ','0',' secs,',' step= ',num2str(step)]);
axis(Axis);
stance_leg=line([ref l*cos(q0(1))],[0 l*sin(q0(1))],'color','red','linewidth',2);
swing_leg=line([ref+l*cos(q0(1)) ref+l*cos(q0(1))+l*cos(q0(1)+q0(2))],...
    [l*sin(q0(1)) l*sin(q0(1))+l*sin(q0(1)+q0(2))],'linewidth',2);
fprintf('\n Animation is ready...\n')

% Define video writer
videoFile = 'simulation_animation_optimized.mp4'; % Specify the file name and format
videoWriter = VideoWriter(videoFile, 'MPEG-4'); % Use the 'MPEG-4' format

% Open the video writer
open(videoWriter);

animation_slowdown_factor=1; % >1 means slow down
for step=1:length(Ie)
    t=T{step};
    x=X{step};
    xe=Xe{step};
    xe=xe(end,:);
    for k=2:length(t)
        t0=clock;
        drawnow;
        q=x(k,1:2)';
        xdata1=[ref ref+l*cos(q(1))];
        xdata2=[ref+l*cos(q(1)) ref+l*cos(q(1))+l*cos(q(1)+q(2))];
        ydata1=[0 l*sin(q(1))];
        ydata2=[l*sin(q(1)) l*sin(q(1))+l*sin(q(1)+q(2))];
        set(stance_leg,'xdata',xdata1,'ydata',ydata1);
        set(swing_leg,'xdata',xdata2,'ydata',ydata2);
        set(Time,'String',['time= ',num2str(round(time_passed+t(k),1)),' secs,',' step= ',num2str(step)]);
        current_axis=gca;
        if ref>.95*current_axis.XLim(end)
            current_axis.XLim=[.95*ref .95*ref+5];
            Time.Position=[.95*ref+2 1.8 0];
            Axis=axis;
        else
            axis(Axis)
        end
        
        % Capture the current frame
        frame = getframe(gcf);
        writeVideo(videoWriter, frame);

        while etime(clock,t0)<animation_slowdown_factor*(t(k)-t(k-1))
        end
    end
    time_passed=time_passed+t(end);
    ref=ref+l*(cos(xe(1))+cos(xe(1)+xe(2)));
end

% Close the video writer
close(videoWriter);

fprintf('\n Animation saved as %s\n', videoFile);

%% FUNCTIONS
function xdot=acrobot(t,x,data)
q1=x(1);
q2=x(2);
q1dot=x(3);
q2dot=x(4);
q=x(1:2);
qdot=x(3:4);
Kp=data.Kp;
Kd=data.Kd;
D=data.D;
C=data.C;
G=data.G;
B=data.B;
phi=data.phi;
phiprime=data.phiprime;
phipprime=data.phipprime;
h=data.h;
dhq=data.dhq;
ddhq_dt=data.ddhq_dt;

% DEFINE YOUR CONTROLLER HERE
D_inv = inv(D(q));

tau=inv(dhq(q)*D_inv*B)*(-ddhq_dt([q;qdot])*qdot + dhq(q)*D_inv*(C([q;qdot])*qdot + G(q)) - Kp*sin(h(q)) - Kd*dhq(q)*qdot);
tau=double(tau);
qddot=D_inv*(-C(x)*qdot-G(q)+B*tau);
qddot=double(qddot);

xdot=[qdot;qddot];
end

function [value,isterminal,direction]=ground_impact(t,x)
q1=x(1);
q2=x(2);
% impact occurs when q2 = -2*q1+2*pi
value=q2+2*q1-2*pi;

% We exclude the scuffing point from the impact conditions
if abs(q1-pi/2)<0.01
    isterminal=0;
else
    isterminal=1;
end

% We distinguish between impact on S^+ or S^- by changing the way in which
% ode45 monitors the zero crossing
if abs(q1)<pi/2
    direction=-1;
else
    direction=1;
end
end

%% Added functions for generating nonlinear constraints for optimization
function f=lengthcost(x,data)
sigmaprime_a = data.sigmaprime_a;

beta = x(length(x)-2);
a = flip(x(1:(length(x)-3)));
q1tilde = beta;

integrand = @(theta) norm(sigmaprime_a(theta,a,q1tilde));
integrand = @(theta) arrayfun(integrand,theta);
f = integral(integrand,0,1);
%f = @(theta) arrayfun(f,theta);

end

function f=flatcost(x)
v1 = x(length(x)-1);
f = double(v1^2);
end

function [c, ceq]=nlcons(x,data,num)
I = data.I;
sigma_a = data.sigma_a;
sigmaprime_a = data.sigmaprime_a;
sigmapprime_a = data.sigmapprime_a;

B_perp = data.B_perp;
D = data.D;
C = data.C;
G = data.G;

samples = data.samples;

beta = x(length(x)-2);
v1 = x(length(x)-1);

qminus = [(pi-beta)/2;pi+beta];
qplus = [(beta+pi)/2;pi-beta];
q1tilde = qplus(1)-qminus(1);

I = double(I(qminus));

e = 10e-6;

epsilon = 10e-6;
a = flip(x(1:(length(x)-3)));
c = zeros(length(samples)+5);

% Recreate functions that work with variable a, q+ and q~
psi_1 = @(theta) -(B_perp*G(sigma_a(theta,a,qplus(1),q1tilde)))/(B_perp*D(sigma_a(theta,a,qplus(1),q1tilde))*sigmaprime_a(theta,a,q1tilde));
psi_2 = @(theta) -(B_perp*(D(sigma_a(theta,a,qplus(1),q1tilde))*sigmapprime_a(theta,a) + C([sigma_a(theta,a,qplus(1),q1tilde);sigmaprime_a(theta,a,q1tilde)])*sigmaprime_a(theta,a,q1tilde))) / (B_perp*D(sigma_a(theta,a,qplus(1),q1tilde))*sigmaprime_a(theta,a,q1tilde));
psi_1 = @(theta) arrayfun(psi_1,theta);
psi_2 = @(theta) arrayfun(psi_2,theta);

M = @(theta) exp(-2*integral(psi_2, 0, theta));
M = @(theta) arrayfun(M,theta);

V_integrand = @(theta) psi_1(theta)*M(theta);
V_integrand = @(theta) arrayfun(V_integrand,theta);

V = @(theta) -integral(V_integrand,0,theta);
V = @(theta) arrayfun(V,theta);

Mminus = M(1);
Vminus = V(1);
delta = dot(sigmaprime_a(0,a,q1tilde),I*sigmaprime_a(1,a,q1tilde)) / (sigmaprime_a(0,a,q1tilde).' * sigmaprime_a(0,a,q1tilde));

Vs = V(samples);
for i = 1:length(samples)
    q = sigma_a(samples(i),a,qplus(1),q1tilde);
    c_i = epsilon - (B_perp*D(q)*sigmaprime_a(samples(i),a,q1tilde))^2;
    c(i) = double(c_i);
end
Vmax = max(Vs);

c(length(samples)+1) = double(-delta^2/Mminus + e);
c(length(samples)+2) = double(delta^2/Mminus - 1 + e);
c(length(samples)+3) = double(( (Vminus*delta^2) / (Mminus - delta^2) ) + Vmax + e);

if num == 1
    c(length(samples)+4) = double(I(1,1)*q1tilde/I(1,2) - v1 + e); 
    c(length(samples)+5) = double((I(2,1) + 2*I(1,1))*q1tilde / (2*I(1,2) + I(2,2)) - v1);
else
    c(length(samples)+4) = double(v1 - I(1,1)*q1tilde/I(1,2) + epsilon); 
    c(length(samples)+5) = double(v1 - (I(2,1) + 2*I(1,1))*q1tilde / (2*I(1,2) + I(2,2)));
end

ceq = [];

end
