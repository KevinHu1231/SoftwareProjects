clear
clear all

density = 1.225;
Sref = 0.01;
CDf = 0.7;
W = 8;
A = pi*(8*0.0254)^2;
Vs = linspace(0,20,(20/0.5)+1);
gamma_m = 0.8;
gamma_esc = 0.9;
Eb = 51948;

Ts = zeros(1,length(Vs));
Pinds = zeros(1,length(Vs));
Ptots = zeros(1,length(Vs));
Ptot_Vs = zeros(1,length(Vs));
i = 1;

for V = Vs
    Df = 1/2*density*Sref*CDf*V^2;
    T = sqrt(W^2+Df^2);
    alpha_D = atan(Df/W);
    eqn = [1,2*V*sin(alpha_D),V^2,0,-(W^2+Df^2)/(2*density*A)^2];
    root = roots(eqn);
    vind = root(end);
    Pind = T*vind;
    Ptot = T*(vind+V*sin(alpha_D));
    
    Ts(1,i) = T;
    Pinds(1,i) = Pind;
    Ptots(1,i) = Ptot;
    Ptot_Vs(1,i) = Ptot/V;
    i = i+1;
end

figure
plot(Vs,Ts)
title("Thrust as a Function of Velocity Curve")
xlabel("Velocity (m/s)")
ylabel("Thrust (N)")

figure
plot(Vs,Pinds)
title("Induced Power as a Function of Velocity Curve")
xlabel("Velocity (m/s)")
ylabel("Induced Power (W)")

figure
plot(Vs,Ptots)
title("Total Power as a Function of Velocity Curve")
xlabel("Velocity (m/s)")
ylabel("Total Power (W)")

figure
plot(Vs,Ptot_Vs)
title("Total Power Over Velocity as a Function of Velocity Curve")
xlabel("Velocity (m/s)")
ylabel("Total Power Over Velocity (W/(m/s))")


endurance = (Eb*gamma_esc*gamma_m)./Ptots;
range = Vs.*endurance;

[max_endurance, max_endurance_id] = max(endurance);
[max_range, max_range_id] = max(range);

max_endurance
max_range

max_endurance_V = Vs(max_endurance_id)
max_range_V = Vs(max_range_id)



