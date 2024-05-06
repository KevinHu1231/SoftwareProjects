clear
clear all
clear clc

alt = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 15000];
density = [1.789, 1.758, 1.726, 1.694, 1.661, 1.628, 1.595, 1.561, 1.527, 1.493, 1.458, 1.422];
% Source: https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
CL_max = 1.25;
W = 11000;
S = 24;
CD0 = 0.03;
epsilon = 0.7;
AR = 10;
K = 1/(pi*epsilon*AR);
CL_TRmin = sqrt(CD0/K);
V_TRmin = sqrt((2*W/S)./(CL_TRmin.*density));
V_min = sqrt((2*W/S)./(CL_max.*density));

Ts = 5000;
T = (Ts/density(1)).*density;
T_W = T./W;
V_RCmax = sqrt((T_W.*(W/S)./(3*CD0.*density)).*(1+sqrt(1+(12*CD0*K)./T_W.^2)));

figure
plot(alt,V_min);
hold on
plot(alt,V_TRmin);
title("Velocity Altitude Envelope")
xlabel("Altitude (m)")
ylabel("Velocity (m/s)")
legend("Vmin CLmax","VTRmin")

figure
plot(alt,V_RCmax)
title("Maximum Rate of Climbing With Respect To Altitude")
xlabel("Altitude")
ylabel("Rate of Climbing (m/s)")
