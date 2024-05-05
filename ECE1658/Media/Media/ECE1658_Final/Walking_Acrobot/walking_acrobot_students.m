%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% FINAL PROJECT
%% VHC for walking acrobot
%% PARTIAL CODE TO GET STUDENTS STARTED ON THE PROJECT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc

%% NOTE: ONLY ONE ANIMATION WILL BE GENERATED EACH RUN
%% TOGGLE OPTIMIZE VARIABLE TO CHANGE BETWEEN OPTIMIZED AND ORIGINAL VHC

number_steps=25;
symbolic_computations=1;
testvhc=1;

% Optimize = 0 to get original VHC
% Optimize = 1 to get optimized VHC

optimize=0;
lencost=0;

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

% Set quantities with values
beta = 0.316637;
v1 = -0.894373;
v2 = 1.9;
q_minus = [(pi-beta)/2;pi+beta];
q_plus = [(beta+pi)/2;pi-beta];
q1_tilde = q_plus(1)-q_minus(1);

% Calculate I

dTqminus = subs(jacobian(T,q),q,q_minus);
Deltaqdotqminus = subs(Deltaqdot,q,q_minus);
I = dTqminus*Deltaqdotqminus;

%% Save functions in data structure

% Save I, B_perp in function
I_sym = jacobian(T,q)*Deltaqdot;
Ifun = matlabFunction(I_sym,'Vars',{q});


data.I=Ifun;
B_perp = [1,0];
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

%Get matrix A
A = zeros(6,6);
A(1,1) = 1;
A(2,:) = 1;
A(3,:) = 0.5.^(0:5);
A(4,2) = 1;
A(5,:) = 0:5;
A(6,:) = A(5,:);
A(6,2:end) = A(6,2:end).*A(3,1:(end-1));

% Symbolic declarations for easier access during optimization

syms th q1plus q2plus q2minus q1tilde v_1 v_2
syms I_ [2 2]
syms ai [6 1]

% Get vector B
f_sym = -(q1tilde*(-I_(2,1)*q1tilde + I_(2,2)*v1))/(-I_(1,1)*q1tilde + I_(1,2)*v_1);
b_sym = [q2plus;q2minus;pi;f_sym;v_1;v_2];
b_sym_fun = matlabFunction(b_sym,'Vars',{v_1,v_2,I_,q2plus,q2minus,q1tilde});
data.b = b_sym_fun;

b = b_sym_fun(v1,v2,double(I),q_plus(2),q_minus(2),q1_tilde);

% Solve linear system for coefficients
a_vec = A\b;

%% 4.2 Optimization

if optimize
    fprintf('\n Optimizing vhc...\n')
    % Initial Condition
    a_vec = [a_vec;zeros(k_poly-6,1)];
    x0 = [a_vec;beta;v1;v2];
    
    e = 1e-6;
    % Linear Equality Constraints
    Aopt = [A,zeros(6,k_poly-6+3)];
    data.A = Aopt;

    Aeq = [];
    beq = [];

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
    num_samples=1001;
    data.samples=rand(num_samples,1);
    
    % Define Sigma and Potential Functions in terms of a, th, q1plus and q1tilde
    phi_a_sym = poly2sym(ai,th);
    phiprime_a_sym = diff(phi_a_sym,th);
    phipprime_a_sym = diff(phiprime_a_sym,th);

    sigma_a_sym = [q1plus - th*q1tilde; phi_a_sym];
    sigmaprime_a_sym = diff(sigma_a_sym,th);
    sigmapprime_a_sym = diff(sigmaprime_a_sym,th);
    
    sigma_a_sym_fun = matlabFunction(sigma_a_sym,'Vars',{ai,th,q1plus,q1tilde});
    sigmaprime_a_sym_fun = matlabFunction(sigmaprime_a_sym,'Vars',{ai,th,q1plus,q1tilde});
    sigmapprime_a_sym_fun = matlabFunction(sigmapprime_a_sym,'Vars',{ai,th,q1plus,q1tilde});
    
    data.sigma_a=sigma_a_sym_fun;
    data.sigmaprime_a=sigmaprime_a_sym_fun;
    data.sigmapprime_a=sigmapprime_a_sym_fun;

    psi_1_a_sym = -(B_perp*Gfun(sigma_a_sym)) / (B_perp*Dfun(sigma_a_sym)*sigmaprime_a_sym);
    psi_2_a_sym = -(B_perp*(Dfun(sigma_a_sym)*sigmapprime_a_sym + Cfun([sigma_a_sym;sigmaprime_a_sym])*sigmaprime_a_sym)) / (B_perp*Dfun(sigma_a_sym)*sigmaprime_a_sym);
    psi_1_a_sym_fun = matlabFunction(psi_1_a_sym,'Vars',{ai,th,q1plus,q1tilde});
    psi_2_a_sym_fun = matlabFunction(psi_2_a_sym,'Vars',{ai,th,q1plus,q1tilde});
    
    data.psi_1_a = psi_1_a_sym_fun;
    data.psi_2_a = psi_2_a_sym_fun;

    % Nonlinear Constraints
    nonlcon = @(x) nlcons(x,data,2);

    % Cost function
    if lencost 
        fun = @(x) lengthcost(x,sigmaprime_a_sym_fun);
    else
        fun = @(x) flatcost(x);
    end

    % Bounds
    lb = [];
    ub = [];

    %Optimization

    %options = optimoptions("fmincon",'Display','iter',"Algorithm","sqp",'ConstraintTolerance',1e-14,'StepTolerance',1e-10,'OptimalityTolerance',10e-10,"MaxFunctionEvaluations",10000,"MaxIterations",100);
    options = optimoptions("fmincon",'Display','iter',"Algorithm","interior-point","EnableFeasibilityMode",true, "SubproblemAlgorithm","cg",'ConstraintTolerance',1e-14,'StepTolerance',1e-10,'OptimalityTolerance',1e-10,"MaxFunctionEvaluations",10000,"MaxIterations",1000);
    [x,fval,exitflag,output] = fmincon(fun,x0,Aineq,bineq,Aeq,beq,lb,ub,nonlcon,options);
    
    % Save best feasible point
    if exitflag == -2
        if ~isempty(output.bestfeasible)
            x = output.bestfeasible.x;
            disp("Converged to Infeasible Point, Best Feasible Point Returned")
        else
            disp("Optimization Failed")
        end
    else
        disp("Optimization Succeeded")
    end

    % Set optimization quantities
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

% Define symbolic functions
phi_sym = poly2sym(a,th);
phiprime_sym = diff(phi_sym,th);
phipprime_sym = diff(phiprime_sym,th);
sigma_sym = [q_plus(1) - th*q1_tilde; phi_sym];
sigmaprime_sym = diff(sigma_sym,th);
sigmapprime_sym = diff(sigmaprime_sym,th);


%% HERE WRITE CODE TO TEST WHETHER YOUR VHC WORKS
if testvhc
N_points = 101;
theta_vals = linspace(0,1,N_points);
qs = sigma(theta_vals);
sps = sigmaprime(theta_vals);

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
transverses = zeros(N_points,1);
for i = 1:N_points
    D = Dfun(qs(:,i));
    transverse = B_perp*D*sps(:,i);
    transverses(i,1) = transverse;
end
figure
plot(theta_vals,transverses)

end

%% Define Potential Functions

psi_1_sym = -(B_perp*Gfun(sigma_sym)) / (B_perp*Dfun(sigma_sym)*sigmaprime_sym);
psi_2_sym = -(B_perp*(Dfun(sigma_sym)*sigmapprime_sym + Cfun([sigma_sym;sigmaprime_sym])*sigmaprime_sym)) / (B_perp*Dfun(sigma_sym)*sigmaprime_sym);
psi_1_sym_fun = matlabFunction(psi_1_sym,'Vars',{th});  
psi_2_sym_fun = matlabFunction(psi_2_sym,'Vars',{th});

data2.psi_1 = psi_1_sym_fun;
data2.psi_2 = psi_2_sym_fun;

% Integrate potential functions
ops = odeset('RelTol',1e-4,'AbsTol',1e-4);
N_points = 101;
[Theta,X] = ode45(@potential,linspace(0,1,N_points),[1;0],ops,data2);

% Plot potential function
figure
plot(Theta,X(:,1));
hold on
plot(Theta,X(:,2));

% Save potential function by curve fitting into spline
M = spline(Theta,X(:,1));
V = spline(Theta,X(:,2));
Mminus = X(N_points,1);
Vminus = X(N_points,2);

I = eval(I);
delta = dot(sigmaprime(0),I*sigmaprime(1)) / (sigmaprime(0).' * sigmaprime(0));

%% Test hybrid limit cycle constraint
if testvhc
    Vs = ppval(V,theta_vals);
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
syms q_1(t) q_2(t)
phi_sym = poly2sym(a, th);
phi_sym = subs(phi_sym,th,(q_plus(1)-q_1(t))/q1_tilde);

% Define symbolic functions used by controller
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

% Save as matlab functions
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
figure;

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
videoFile = 'simulation_animation_original.mp4'; % Specify the file name and format
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

% Define controller using equations from lecture

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

%% Potential function calculations
function xdot=potential(theta,x,data)
    M = x(1);
    Mdot = -2*M*data.psi_2(theta);
    Vdot = -data.psi_1(theta)*M;
    xdot = [Mdot;Vdot];
end

% Potential function for optimization
function xdot=potential_opt(theta,x,data)
    M = x(1);
    Mdot = -2*M*data.psi_2(data.a,theta,data.q1_plus,data.q1_tilde);
    Vdot = -data.psi_1(data.a,theta,data.q1_plus,data.q1_tilde)*M;
    xdot = [Mdot;Vdot];
end

%% Added functions for generating nonlinear constraints for optimization

% Integral for objective function
function xdot=length_integral(theta,x,data)
    xdot = norm(data.sigmaprime_a(data.a,theta,data.q1_plus,data.q1_tilde));
end

% Objective functions
function f=lengthcost(x,sigmaprime_a)
    beta = x(length(x)-2);
    a = flip(x(1:(length(x)-3)));
    q1_plus = (beta+pi)/2;
    q1_tilde = beta;
    
    data4.sigmaprime_a = sigmaprime_a;
    data4.a = a;
    data4.q1_plus = q1_plus;
    data4.q1_tilde = q1_tilde;
    
    ops = odeset('RelTol',1e-4,'AbsTol',1e-4);
    [Theta,X] = ode45(@length_integral,linspace(0,1,101),0,ops,data4);
    f = X(101);
end

function f=flatcost(x)
    v1 = x(length(x)-1);
    f = v1^2;
end

% Nonlinear constraints

function [c, ceq]=nlcons(x,data,num)

% Get data
    I = data.I;
    B_perp = data.B_perp;
    D = data.D;
    A = data.A;
    b = data.b;
    
    samples = data.samples;
   
    % Calculate quantities
    beta = x(length(x)-2);
    v1 = x(length(x)-1);
    v2 = x(length(x));
    
    q_minus = [(pi-beta)/2;pi+beta];
    q_plus = [(beta+pi)/2;pi-beta];
    q1_tilde = q_plus(1)-q_minus(1);
    
    sigma_a = data.sigma_a;
    sigmaprime_a = data.sigmaprime_a;
    
    I = I(q_minus);
    e = 10e-6;
    epsilon = 10e-3;
    
    a = flip(x(1:(length(x)-3)));
    c = zeros((length(samples)+5),1);
    
    data3.a = a;
    data3.q1_plus = q_plus(1);
    data3.q1_tilde = q1_tilde;
    data3.psi_1 = data.psi_1_a;
    data3.psi_2 = data.psi_2_a;
    
    %Integrate to get potential functions dependent on optimization parameters
    ops = odeset('RelTol',1e-4,'AbsTol',1e-4);
    [Theta,X] = ode45(@potential_opt,linspace(0,1,101),[1;0],ops,data3);
    
    % Save potential functions by curve fitting into splines
    V = spline(Theta,X(:,2));
    Mminus = X(101,1);
    Vminus = X(101,2);
    
    % Calculate quantities
    sigmaprime0 = sigmaprime_a(a,0,q_plus(1),q1_tilde);
    sigmaprime1 = sigmaprime_a(a,1,q_plus(1),q1_tilde);
    delta = dot(sigmaprime0,I*sigmaprime1) / (sigmaprime0.' * sigmaprime0);
    
    Vs = ppval(V,samples);
    for i = 1:length(samples)
        % Transversality constraints
        theta_val = samples(i,1);
        q_th = sigma_a(a,theta_val,q_plus(1),q1_tilde);
        c_i = epsilon - (B_perp*D(q_th)*sigmaprime_a(a,theta_val,q_plus(1),q1_tilde))^2;
        c(i,1) = c_i;
    end
    Vmax = max(Vs);
    
    %Hybrid limit cycle constraints
    c(length(samples)+1,1) = -delta^2/Mminus + e;
    c(length(samples)+2,1) = delta^2/Mminus - 1 + e;
    c(length(samples)+3,1) = (Vminus*delta^2) / (Mminus - delta^2) + Vmax + e;
    
    % Select one of these constraints
    if num == 1
        c(length(samples)+4,1) = (I(1,1)*q1_tilde/I(1,2)) - v1 + e;
        c(length(samples)+5,1) = ((I(2,1) + 2*I(1,1))*q1_tilde / (2*I(1,2) + I(2,2))) - v1;
    else
        c(length(samples)+4,1) = v1 - I(1,1)*q1_tilde/I(1,2) + e;
        c(length(samples)+5,1) = v1 - (I(2,1) + 2*I(1,1))*q1_tilde / (2*I(1,2) + I(2,2));
    end
    
    % Resolve equality constraint, now nonlinear since b is dependent on optimization parameters
    b = b(v1,v2,I,q_plus(2),q_minus(2),q1_tilde);
    ceq = A*x - b;

end
