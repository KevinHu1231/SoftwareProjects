%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% FINAL PROJECT
%% VHC for walking acrobot
%% PARTIAL CODE TO GET STUDENTS STARTED ON THE PROJECT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%close all
%clear all
%clc

number_steps=25;
symbolic_computations=1;

% 1 to turn on
testvhc=1;
optimize=1;
lencost=0;

%For polynomial
sigma_poly_k = 6;
u_poly_k = 3;

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
    syms q1 q2 x1 x2 q1dot q2dot x1dot x2dot tau real

    q=[q1;q2];x=[x1;x2]; qbar=[q;x];
    qdot=[q1dot;q2dot];  xdot=[x1dot;x2dot]; qbardot=[qdot;xdot];

    % Mathematical Quantities from paper
    D = [m*(2*lc^2+l^2+2*l*lc*cos(q2))+2*Iz, m*(lc^2+l*lc*cos(q2))+Iz ; m*(lc^2 + l*lc*cos(q2)) + Iz, m*lc^2+Iz];
    C = -m*l*lc*sin(q2)*[q2dot, q1dot+q2dot; -q1dot, 0];
    P = g*sin(q1)*(m*lc+m*l)+g*sin(q1+q2)*m*lc;
    G = jacobian(P,q)';

    ds = [(-l*m-lc*m)*sin(q1) - lc*m*sin(q1+q2), (l*m+lc*m)*cos(q1) + lc*m*cos(q1+q2); -lc*m*sin(q1+q2), lc*m*cos(q1+q2)];
    De = [D,ds;ds.',2*m*eye(2)];
    E = [-l*sin(q1+q2)-l*sin(q1), -l*sin(q1+q2), 1, 0; l*cos(q1+q2)+l*cos(q1), l*cos(q1+q2), 0, 1];
    B = [0;1];

    % Computation of qddot (pinned robot)

    %     qddot = simplify(inv(D)*(-C*qdot-G+B*tau));
    Dfun=matlabFunction(D,'Vars',{q});
    Gfun=matlabFunction(G,'Vars',{q});
    Cfun=matlabFunction(C,'Vars',{[q;qdot]});
    fprintf('\n Impact map computation...\n')

    %% Impact map

    R = -q + [pi;0];
    De_inv = simplify(inv(De));
    omega = simplify([eye(2),zeros(2,2)]*(eye(4) - De_inv*E.'*inv(E*De_inv*E.')*E)*[eye(2);zeros(2,2)]);
    omegafun = matlabFunction(omega,'Vars',{q});
    Delta = [R;-omega*qdot];
    Deltafun=matlabFunction(Delta,'Vars',{[q;qdot]});

    save('walking_acrobot_model','D','Dfun','Cfun','Gfun','Deltafun','B');
else
    fprintf('\nLoading robot model...\n')
    load('walking_acrobot_model');
end
%% HERE WRITE YOUR CODE FOR THE VHC DESIGN
fprintf('\nDetermining vhc...\n')

%Estimated Starting Parameters
beta = 0.5;
q_minus = [(pi-beta)/2;pi+beta];
q_plus = [(beta+pi)/2;pi-beta];
q1_tilde = q_plus(1)-q_minus(1);


%% Save functions in data structure

%Function I
I_sym = jacobian(R,q)*omega;
Ifun = matlabFunction(I_sym,'Vars',{q});

data.I=Ifun;

%From Paper
B_perp = [-1,0];
data.B_perp = B_perp;

% Define single-input planar control system
f_acro = B_perp.';
g_acro = inv(D)*B;
g_acro_fun = matlabFunction(g_acro,'Vars',{q});

data.f_acro = f_acro;
data.g_acro = g_acro_fun;

% This is the data structure to be passed to various functions
% You will need to add extra information to it.
data.Kp=Kp;
data.Kd=Kd;
data.D=Dfun;
data.C=Cfun;
data.G=Gfun;
data.B=B;

%% Symbolic Declarations

syms th q1plus q2plus q1minus q2minus q1tilde
syms ai [6 1] % Phi function polynomial coefficients
syms ui [3 1] % u function polynomial coefficients
syms t [2 1] % Tangent Vector

%% 4.2 Optimization

if optimize
    fprintf('\n Optimizing vhc...\n')

    % Linear Equality Constraints
    Aeq = [];
    beq = [];

    % Linear Inequality Constraints
    Aineq = [];
    bineq = [];

    % Generate random samples
    num_samples=1000;
    data.samples=rand(num_samples,1);


    % Define sigma, u and Potential Functions in terms of ai, ui, th, q1plus and q1tilde
    phi_a_sym = poly2sym(ai,th);
    phiprime_a_sym = diff(phi_a_sym,th);
    phipprime_a_sym = diff(phiprime_a_sym,th);

    u_sym = poly2sym(ui,th);
    uprime_sym = diff(u_sym,th);
    upprime_sym = diff(uprime_sym,th);

    u = matlabFunction(u_sym,'Vars',{ui,th});
    uprime = matlabFunction(uprime_sym,'Vars',{ui,th});
    upprime = matlabFunction(upprime_sym,'Vars',{ui,th});

    data.u = u;
    data.uprime = uprime;
    data.upprime = upprime;

    sigma_a_sym = [q1plus - th*q1tilde; phi_a_sym];
    sigmaprime_a_sym = diff(sigma_a_sym,th);
    sigmapprime_a_sym = diff(sigmaprime_a_sym,th);

    sigma_a_sym_fun = matlabFunction(sigma_a_sym,'Vars',{ai,th,q1plus,q1tilde});
    sigmaprime_a_sym_fun = matlabFunction(sigmaprime_a_sym,'Vars',{ai,th,q1plus,q1tilde});
    sigmapprime_a_sym_fun = matlabFunction(sigmapprime_a_sym,'Vars',{ai,th,q1plus,q1tilde});

    data.sigma_a=sigma_a_sym_fun;
    data.sigmaprime_a=sigmaprime_a_sym_fun;
    data.sigmapprime_a=sigmapprime_a_sym_fun;

    % Define psi_1 and psi_2 functions

    psi_1_a_sym = simplify(-(B_perp*Gfun(sigma_a_sym)) / (B_perp*Dfun(sigma_a_sym)*sigmaprime_a_sym));
    psi_2_a_sym = simplify(-(B_perp*(Dfun(sigma_a_sym)*sigmapprime_a_sym + Cfun([sigma_a_sym;sigmaprime_a_sym])*sigmaprime_a_sym)) / (B_perp*Dfun(sigma_a_sym)*sigmaprime_a_sym));
    psi_1_a_sym_fun = matlabFunction(psi_1_a_sym,'Vars',{ai,th,q1plus,q1tilde});
    psi_2_a_sym_fun = matlabFunction(psi_2_a_sym,'Vars',{ai,th,q1plus,q1tilde});

    data.psi_1_a = psi_1_a_sym_fun;
    data.psi_2_a = psi_2_a_sym_fun;

    %Define Theta function

    TH = -omega*t / norm(omega*t);
    TH_fun = matlabFunction(TH,'Vars',{q,t});
    data.TH = TH_fun;

    %Define safe set definition functions

    I1 = q2 + 2*q1 - 2*pi;
    I2 = -2*q1-q2;
    I3 = -q1;
    I4 = q1 - pi;

    I1_fun = matlabFunction(I1,'Vars',{q});
    I2_fun = matlabFunction(I2,'Vars',{q});
    I3_fun = matlabFunction(I3,'Vars',{q});
    I4_fun = matlabFunction(I4,'Vars',{q});

    data.I1 = I1_fun;
    data.I2 = I2_fun;
    data.I3 = I3_fun;
    data.I4 = I4_fun;

    %For function

    % Solve for Initial Condition
    % th_vals = [0,0.5,1];
    % phi_vals = [q_plus(2),0,q_minus(2)];
    % poly = polyfit(th_vals,phi_vals,5);

    % g_acro_ai = g_acro_fun(sigma_a_sym)
    % g_acro_ai_0 = eval(subs(g_acro_ai,th,0))
    % g_acro_ai_05 = eval(subs(g_acro_ai,th,05)) 
    % g_acro_ai_1 = eval(subs(g_acro_ai,th,1))
    % 
    % f = f_acro;
    % sp_0 = subs(sigmaprime_a_sym,th,0) - f
    % sp_05 = subs(sigmaprime_a_sym,th,0.5) - f
    % sp_1 = subs(sigmaprime_a_sym,th,1) - f
    % 
    % val_0 = sp_0 ./ g_acro_ai_0
    % val_05 = sp_05 ./ g_acro_ai_05
    % val_1 = sp_1 ./ g_acro_ai_1
    % 
    % 
    % t_b_val = sigmaprime(1)/norm(sigmaprime(1));
    % 
    % f = f_acro;
    % g_0 = g_acro_fun(sigma(0));
    % g_05 = g_acro_fun(sigma(0.5));
    % g_1 = g_acro_fun(sigma(1));
    % sp_0 = sigmaprime(0) - f;
    % sp_05 = sigmaprime(0.5) - f;
    % sp_1 = sigmaprime(1) - f;
    % val_0 = sp_0 ./ g_0;
    % val_05 = sp_05 ./ g_05;
    % val_1 = sp_1 ./ g_1;
    % 
    % th_vals = [0,0,0.5,0.5,1,1];
    % u_vals = [val_0.',val_05.',val_1.'];
    % poly_u = polyfit(th_vals,u_vals,2);
    % 
    % x0 = zeros(sigma_poly_k+u_poly_k+3,1);
    % 
    % x0(1:(length(x0)-6)) = poly;
    % x0((length(x0)-5):(length(x0)-3)) = poly_u;
    % x0((length(x0)-2)) = beta;
    % x0((length(x0)-1):(length(x0))) = t_b_val;

    % Nonlinear Constraints
    nonlcon = @(x) nlcons(x,data);

    % Cost function
    fun = @(x) J(x,sigmaprime_a_sym_fun,upprime,0.5);

    % Bounds
    lb = [];
    ub = [];

    % Estimated bounds for random number generation
    poly_mag = 10;
    beta_mag = pi;

    % Save best parameters

    best_x = NaN;
    best_x0 = NaN;
    best_constr_vio = inf;

    iters = 0;
    while 1

        % Generate random initial condition
        x0_poly = -poly_mag + 2*poly_mag*rand(9,1);
        x0_beta = -beta_mag + 2*beta_mag*rand(1,1);
        x0_t = [-1;-1] + 2*rand(2,1);
        x0 = [x0_poly;x0_beta;x0_t];

        % Optimization

        %options = optimoptions("fmincon",'Display','iter',"Algorithm","sqp",'ConstraintTolerance',1e-14,'StepTolerance',1e-10,'OptimalityTolerance',10e-10,"MaxFunctionEvaluations",10000,"MaxIterations",100);
        options = optimoptions("fmincon",'Display','iter',"Algorithm","interior-point","EnableFeasibilityMode",true, "SubproblemAlgorithm","cg",'ConstraintTolerance',1,'StepTolerance',1e-10,'OptimalityTolerance',1e-10,"MaxFunctionEvaluations",10000,"MaxIterations",1000);
        [x,fval,exitflag,output] = fmincon(fun,x0,Aineq,bineq,Aeq,beq,lb,ub,nonlcon,options);

        % Update Best
        if output.constrviolation < best_constr_vio
            best_x = x;
            best_constr_vio = output.constrviolation;
            best_x0 = x0;
        end

        if exitflag == -2
            if ~isempty(output.bestfeasible)
                x = output.bestfeasible.x;

                if output.bestfeasible.constrviolation < best_constr_vio
                    best_x = x;
                    best_constr_vio = output.constrviolation;
                    best_x0 = x0;
                end
                disp("Converged to Infeasible Point, Best Feasible Point Returned")

            else
                disp("Optimization Failed")
            end
        else
            % Exit if success
            if exitflag == 1
                disp("Optimization Succeeded")
                break
            end
        end
        iters = iters+1;
        disp(iters)
    end
    %% 

    % Get parameters
    a_vec = x(1:(length(x)-6));
    u_vec = x((length(x)-5):(length(x)-3));
    beta = x(length(x)-2);
    t1 = x(length(x)-1);
    t2 = x(length(x));
    q_minus = [(pi-beta)/2;beta-pi];
    q_plus = [(beta+pi)/2;pi-beta];
    q1_tilde = q_plus(1)-q_minus(1);

end

%% HERE WE DEFINE THE FUNCTION phi_a AND ITS DERIVATIVES

a=a_vec;
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
W1_q1 = linspace(0,pi,1001);
W1_u = -2*W1_q1 + 2*pi;
W1_l = -2*W1_q1;

figure
plot(W1_q1,W1_u)
hold on
plot(W1_q1,W1_l)
hold on

fill([W1_q1,fliplr(W1_q1)], [W1_u,fliplr(W1_l)], 'y');

%Plot Curve and Points
plot(qs(1,:),qs(2,:),'k');
hold on
plot(q_plus(1),q_plus(2),'r*')
hold on
plot(q_minus(1),q_minus(2),'b*')
hold on
plot(pi/2,0,'g*')
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

% Integrate

ops = odeset('RelTol',1e-4,'AbsTol',1e-4);
N_points = 101;
[Theta,X] = ode45(@potential,linspace(0,1,N_points),[1;0],ops,data2);

% Plot potential functions

figure
plot(Theta,X(:,1));
hold on
plot(Theta,X(:,2));

% Save potential functions by curve fitting to spline
M = spline(Theta,X(:,1));
V = spline(Theta,X(:,2));
Mminus = X(N_points,1);
Vminus = X(N_points,2);

% Calculate quantities
I = Ifun(q_minus);
delta = dot(sigmaprime(0),I*sigmaprime(1)) / (sigmaprime(0).' * sigmaprime(0));

%%
if testvhc

    % Test for limit cycle
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
% syms q_1(t) q_2(t)
% phi_sym = poly2sym(a, th);
% phi_sym = subs(phi_sym,th,(q_plus(1)-q_1(t))/q1_tilde);
% 
% h = q_2(t) - phi_sym;
% q_ = [q_1(t);q_2(t)];
% dhq = jacobian(h,q_);
% ddhq_dt = diff(dhq,t);
% 
% h = subs(h,[q_1(t),q_2(t)],[q1,q2]);
% dhq = subs(dhq,[q_1(t),q_2(t)],[q1,q2]);
% 
% ddhq_dt = subs(ddhq_dt,diff(q_1(t),t),q1dot);
% ddhq_dt = subs(ddhq_dt,diff(q_2(t),t),q2dot);
% ddhq_dt = subs(ddhq_dt,q_2(t),q2);
% ddhq_dt = subs(ddhq_dt,q_1(t),q1);
% 
% hfun=matlabFunction(h,'Vars',{q});
% dhqfun=matlabFunction(dhq,'Vars',{q});
% ddhq_dtfun=matlabFunction(ddhq_dt,'Vars',{[q;qdot]});
% data.h = hfun;
% data.dhq = dhqfun;
% data.ddhq_dt = ddhq_dtfun;
% 
% %% NOW YOU CAN SIMULATE THE ROBOT. PLACE YOUR CONTROLLER INSIDE THE FUNCTION acrobot AT THE END OF THIS SCRIPT
% ops= odeset('reltol',1e-7,'abstol',1e-7,'Events',@ground_impact);
% dt=1/60; % 60 fps; time increment in simulations and animations
% 
% fprintf('\n Simulating...\n')
% %% DEFINE THE INITIAL CONDITION [q0;qdot0];
% q0 = sigma(0);
% thetaadot = double(delta*sqrt((-2*Vminus) / (Mminus - delta^2)));
% qdot0 = sigmaprime(0)*thetaadot;
% T=[];
% X=[];
% Te=[];
% Ie=[];
% Xe=[];
% post_impact_state=[q0;qdot0];
% % Simulate number_steps steps
% for step=1:number_steps
%     fprintf('\n...step %d...\n',step);
%     [t,x,te,xe,ie]=ode45(@(t,x) acrobot(t,x,data),0:dt:10,post_impact_state,ops);
%     % Application of the impact map
%     impact_state=xe(end,:)';
%     post_impact_state=Deltafun(impact_state);
%     T{step}=t;
%     X{step}=x;
%     Ie{step}=ie;
%     Te{step}=te;
%     Xe{step}=xe;
% end
% 
% fprintf('\n Setting up animation...\n')
% figure;
% 
% %% Animation of the simulation results
% ref=0;time_passed=0;step=1;
% Axis=[-1 4 0 2];
% Time=text(-1+2,1.8,['time= ','0',' secs,',' step= ',num2str(step)]);
% axis(Axis);
% stance_leg=line([ref l*cos(q0(1))],[0 l*sin(q0(1))],'color','red','linewidth',2);
% swing_leg=line([ref+l*cos(q0(1)) ref+l*cos(q0(1))+l*cos(q0(1)+q0(2))],...
%     [l*sin(q0(1)) l*sin(q0(1))+l*sin(q0(1)+q0(2))],'linewidth',2);
% fprintf('\n Animation is ready...\n')
% 
% % Define video writer
% videoFile = 'simulation_animation_poly6.mp4'; % Specify the file name and format
% videoWriter = VideoWriter(videoFile, 'MPEG-4'); % Use the 'MPEG-4' format
% 
% % Open the video writer
% open(videoWriter);
% 
% animation_slowdown_factor=1; % >1 means slow down
% for step=1:length(Ie)
%     t=T{step};
%     x=X{step};
%     xe=Xe{step};
%     xe=xe(end,:);
%     for k=2:length(t)
%         t0=clock;
%         drawnow;
%         q=x(k,1:2)';
%         xdata1=[ref ref+l*cos(q(1))];
%         xdata2=[ref+l*cos(q(1)) ref+l*cos(q(1))+l*cos(q(1)+q(2))];
%         ydata1=[0 l*sin(q(1))];
%         ydata2=[l*sin(q(1)) l*sin(q(1))+l*sin(q(1)+q(2))];
%         set(stance_leg,'xdata',xdata1,'ydata',ydata1);
%         set(swing_leg,'xdata',xdata2,'ydata',ydata2);
%         set(Time,'String',['time= ',num2str(round(time_passed+t(k),1)),' secs,',' step= ',num2str(step)]);
%         current_axis=gca;
%         if ref>.95*current_axis.XLim(end)
%             current_axis.XLim=[.95*ref .95*ref+5];
%             Time.Position=[.95*ref+2 1.8 0];
%             Axis=axis;
%         else
%             axis(Axis)
%         end
% 
%         % Capture the current frame
%         frame = getframe(gcf);
%         writeVideo(videoWriter, frame);
% 
%         while etime(clock,t0)<animation_slowdown_factor*(t(k)-t(k-1))
%         end
%     end
%     time_passed=time_passed+t(end);
%     ref=ref+l*(cos(xe(1))+cos(xe(1)+xe(2)));
% end
% 
% % Close the video writer
% close(videoWriter);
% 
% fprintf('\n Animation saved as %s\n', videoFile);

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
% impact occurs when q2 = -2*q1
value=q2+2*q1;

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

% Potential function calculation in optimization problem
function xdot=potential_opt(theta,x,data)
    M = x(1);
    Mdot = -2*M*data.psi_2(data.a,theta,data.q1_plus,data.q1_tilde);
    Vdot = -data.psi_1(data.a,theta,data.q1_plus,data.q1_tilde)*M;
    xdot = [Mdot;Vdot];
end

%% Added functions for generating nonlinear constraints for optimization

% Cost function integral
function xdot=J_integral(theta,x,data)
    J1dot = norm(data.sigmaprime_a(data.a,theta,data.q1_plus,data.q1_tilde));
    Jregdot = norm(data.upprime(data.u,theta))^2;
    xdot = [J1dot;Jregdot];
end

% Cost function
function f = J(x,sigmaprime_a,upprime,gamma)
    beta = x(length(x)-2);
    as = x(1:(length(x)-6));
    us = x((length(x)-5):(length(x)-3));

    q1_plus = (beta+pi)/2;
    q1_tilde = beta;
    
    data4.sigmaprime_a = sigmaprime_a;
    data4.a = as;
    data4.u = us;
    data4.q1_plus = q1_plus;
    data4.q1_tilde = q1_tilde;
    data4.upprime = upprime;
    
    ops = odeset('RelTol',1e-4,'AbsTol',1e-4);
    [Theta,X] = ode45(@J_integral,linspace(0,1,1001),[0;0],ops,data4);
    f = X(1001,1) + X(1001,2)^(1/2);
end

% Nonlinear constraint generation
function [c, ceq]=nlcons(x,data)

% Get quantities
    I = data.I;
    D = data.D;
    B = data.B;
    f_acro = data.f_acro;
    g_acro = data.g_acro;
    TH = data.TH;
    u = data.u;
    I1 = data.I1;
    I2 = data.I2;
    I3 = data.I3;
    I4 = data.I4;
    
    samples = data.samples;
    
    % Calculate terms
    beta = x(length(x)-2);
    t1 = x(length(x)-1);
    t2 = x(length(x));
    t_b = [t1;t2];

    q_minus = [(pi-beta)/2;beta-pi];
    q_plus = [(beta+pi)/2;pi-beta];
    q1_tilde = q_plus(1)-q_minus(1);
    
    sigma_a = data.sigma_a;
    sigmaprime_a = data.sigmaprime_a;
    
    I = I(q_minus);

    %Error hyperparameters
    e_gen = 10e-6;
    e = 10e-6;
    e1 = 10e-6;
    e2 = 10e-6;
    
    as = x(1:(length(x)-6));
    us = x((length(x)-5):(length(x)-3));

    N = length(samples);
    c = zeros(4*N+7,1);
    ceq = zeros(2*N+10,1);
    
    data3.a = as;
    data3.q1_plus = q_plus(1);
    data3.q1_tilde = q1_tilde;
    data3.psi_1 = data.psi_1_a;
    data3.psi_2 = data.psi_2_a;

    % Get potential functions with current optimization parameters 
    
    ops = odeset('RelTol',1e-4,'AbsTol',1e-4);
    [Theta,X] = ode45(@potential_opt,linspace(0,1,1001),[1;0],ops,data3);
    
    % Save potential functions by curve fitting to spline
    V = spline(Theta,X(:,2));
    Mminus = X(1001,1);
    Vminus = X(1001,2);

    % Calculate quantities
    sigma0 = sigma_a(as,0,q_plus(1),q1_tilde);
    sigma1 = sigma_a(as,1,q_plus(1),q1_tilde);
    
    sigmaprime0 = sigmaprime_a(as,0,q_plus(1),q1_tilde);
    sigmaprime1 = sigmaprime_a(as,1,q_plus(1),q1_tilde);

    delta = dot(sigmaprime0,I*sigmaprime1) / (sigmaprime0.' * sigmaprime0);
    
    Vs = ppval(V,samples);

    for i = 1:N
        % Sample theta
        theta_val = samples(i,1);

        % Calculate q qdot from sample
        q_th = sigma_a(as,theta_val,q_plus(1),q1_tilde);
        qdot_th = sigmaprime_a(as,theta_val,q_plus(1),q1_tilde);

        % Equality constraints
        ceq_i = f_acro + g_acro(q_th)*u(us,theta_val) - qdot_th;
        ceq((2*i-1):(2*i),1) = ceq_i;

        % Inequality constraints
        c(4*i-3,1) = I1(q_th);
        c(4*i-2,1) = I2(q_th);
        c(4*i-1,1) = I3(q_th);
        c(4*i,1) = I4(q_th);
    end

    Vmax = max(Vs);
    
    % Limit cycles constraints
    c(4*N+1,1) = -delta^2/Mminus + e1;
    c(4*N+2,1) = delta^2/Mminus - 1 + e2;
    c(4*N+3,1) = (Vminus*delta^2) / (Mminus - delta^2) + Vmax + e;

    % Transversality constraints for tangent vector
    mu = sign(det([f_acro, g_acro(q_plus)]));
    c(4*N+4,1) = -mu*det([t_b,inv(D(q_minus))*B]) + e_gen;
    c(4*N+5,1) = -mu*det([TH(q_minus,t_b),inv(D(q_plus))*B]) + e_gen;

    % Tangent vector constraints
    n = -[2,1].';
    c(4*N+6,1) = -dot(t_b,n);
    c(4*N+7,1) = -dot(TH(q_plus,t_b),n);

    ceq(((2*N+1):(2*N+2)),1) = norm(sigmaprime1)*t_b - sigmaprime1;
    ceq(((2*N+3):(2*N+4)),1) = norm(sigmaprime0)*TH(q_minus,t_b) - sigmaprime0;

    % Optimization problem constraints
    ceq(((2*N+5):(2*N+6)),1) = q_plus - sigma0;
    ceq(((2*N+7):(2*N+8)),1) = q_minus - sigma1;

    ub = norm(sigmaprime1)*inv([f_acro,g_acro(q_minus)])*t_b;
    ub = ub(2,1);

    ua = norm(sigmaprime0)*inv([f_acro,g_acro(q_plus)])*TH(q_minus,t_b);
    ua = ua(2,1);

    ceq(2*N+9,1) = ua - u(us,0);
    ceq(2*N+10,1) = ub - u(us,1);

end
