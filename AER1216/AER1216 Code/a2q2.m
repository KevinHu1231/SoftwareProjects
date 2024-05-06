clear
clear clc
inch_to_meter = 0.0254;

%Propeller data

%R/r
r_R = [0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50,0.55,0.60,0.65,0.70,0.75,0.80,0.85,0.90,0.95,1.00];
%c/r
c_R = [0.109,0.132,0.156,0.176,0.193,0.206,0.216,0.223,0.226,0.225,0.219,0.210,0.197,0.179,0.157,0.130,0.087,0.042];
%Beta from chord line
beta_c = [21.11,23.90,24.65,24.11,22.78,21.01,19.00,17.06,15.33,13.82,12.51,11.36,10.27,9.32,8.36,7.27,6.15,5.04];
N = length(r_R);

%Sigma
sigma = c_R/pi;
%RPM, Omega
rpm = 6000;
rps = rpm/60;
omega = rps*2*pi;
%cd
cd = 0.04;
%a (lift slope)
a = 5.7;

%Diameter, Radius
D = 10*inch_to_meter;
R = D/2;

%Convert to radians
deg_to_rad = 2*pi/360;
beta_c_rad = beta_c*deg_to_rad;

%Convert beta relative to zero lift line
c_to_zll = 6;
beta_rad = beta_c_rad + c_to_zll*deg_to_rad;

%Vector of Velocities, Js and Lambda Cs
Vs = linspace(0,17,18);
Js = Vs/(rps*D);
lambda_cs = Vs/(omega*R);

%Vector of CT and CQ for propeller
CT_props = zeros(1,18);
CQ_props = zeros(1,18);
j = 1;
for lambda_c = lambda_cs
    CT = 0;
    CQ = 0;
    %For each section of propeller
    for i = 1:N
        %Delta
        if i == 1
            delta_x_i = r_R(1);
        else
            delta_x_i = r_R(i) - r_R(i-1);
        end
        %Four simultaneous equations to satisfy with a single value of lambda i 
        syms lambda_i
        dCT1(lambda_i) = (sigma(i)*a/2)*(beta_rad(i)*r_R(i)^2-lambda_i*r_R(i))*delta_x_i;
        dCT2(lambda_i) = 4*lambda_i*(lambda_i-lambda_c)*r_R(i)*delta_x_i;
        dCQ1(lambda_i) = ((sigma(i)*a/2)*(beta_rad(i)*r_R(i)*lambda_i-lambda_i^2) + sigma(i)*cd*r_R(i)^2/2)*r_R(i)*delta_x_i;
        dCQ2(lambda_i) = 4*lambda_i^2*(lambda_i-lambda_c)*r_R(i)*delta_x_i;
        dCT1_ = matlabFunction(dCT1, 'Vars', lambda_i);
        dCT2_ = matlabFunction(dCT2, 'Vars', lambda_i);
        dCQ1_ = matlabFunction(dCQ1, 'Vars', lambda_i);
        dCQ2_ = matlabFunction(dCQ2, 'Vars', lambda_i);

        %Four equations impossible to satisfy exactly at the same time for
        %a single value of lambda i so objective is to minimize the error
        %between the two calculated values of dCT and dCQ

        %Objective function for minimization optimization problem
        objective = @(lambda_i) (dCT1_(lambda_i) - dCT2_(lambda_i))^2 + (dCQ1_(lambda_i) - dCQ2_(lambda_i))^2;
        
        %Minimizer parameters
        lb = -10e9;
        ub = 10e9;
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        x0 = 0;
        
        options = optimset('Display', 'iter', 'TolFun', 1e-8);  % Optional: Display the optimization progress
        %Solve for optimal lambda_i and minimize error fval
        [lambda_opt, fval] = fmincon(objective, x0, A, b, Aeq, beq, lb, ub, [], options);

        %Recalculate dCT and dCQ with optimal lambda i
        dCT_1 = (sigma(i)*a/2)*(beta_rad(i)*r_R(i)^2-lambda_opt*r_R(i))*delta_x_i;
        dCT_2 = 4*lambda_opt*(lambda_opt-lambda_c)*r_R(i)*delta_x_i;
        dCQ_1 = ((sigma(i)*a/2)*(beta_rad(i)*r_R(i)*lambda_opt-lambda_opt^2) + sigma(i)*cd*r_R(i)^2/2)*r_R(i)*delta_x_i;
        dCQ_2 = 4*lambda_opt^2*(lambda_opt-lambda_c)*r_R(i)*delta_x_i;

        %Accumulate CT and CQ with average of both dCT and dCQ values
        CT = CT + dCT_2;
        CQ = CQ + dCQ_2;
    end
    %CT and CQ helicopter to propeller conversion
    CT_heli_prop = (pi*R^4)/(D^4/(4*pi^2));
    CQ_heli_prop = (pi*R^5)/(D^5/(4*pi^2));
    %CT and CQ for propeller
    CT_prop = CT*CT_heli_prop;
    CQ_prop = CQ*CQ_heli_prop;
    %Add to vector
    CT_props(j) = CT_prop;
    CQ_props(j) = CQ_prop;
    j = j + 1;
end

%Plot V vs CT 
figure
plot(Vs,CT_props);
title("CT as a Function of Velocity");
xlabel("Velocity (m/s)");
ylabel("CT");
saveas(gcf,'V_CT.png');
%Plot J vs CT 
figure
plot(Js,CT_props);
title("CT as a Function of J");
xlabel("J");
ylabel("CT");
saveas(gcf,'J_CT.png');
%Plot V vs CQ 
figure
plot(Vs,CQ_props);
title("CQ as a Function of Velocity");
xlabel("Velocity (m/s)");
ylabel("CQ");
saveas(gcf,'V_CQ.png');
%Plot J vs CQ 
figure
plot(Js,CQ_props);
title("CQ as a Function of J");
xlabel("J");
ylabel("CQ");
saveas(gcf,'J_CQ.png');

%When compared to UIUC J vs CT plot the near linear portion of the curve
%starts later at around J = 0.2 compared to just after J = 0.1. The highest 
%value of CT the plot reaches is CT = 0.17 compared to CT = 0.12 for UIUC.
%The UIUC J vs CT plot has CT = 0 when J = 0.62 but this plot only reaches
%CT = 0.145 showing that the UIUC plot has a much steeper near linear
%portion.

%A CQ plot was not available in the UIUC database so not comment was made
%for this plot.