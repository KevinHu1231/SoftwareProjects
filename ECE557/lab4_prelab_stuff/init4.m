% clear all



% initializations

setup_lab_ip01_2_sip



% specs of pendulum and cart system

M = Mc   % cart mass

m = Mp   % pendulum mass

l = lp   % pend. length

g = 9.8





% linearized model of pendulum-cart system

alpha1 = -Eff_g*Kg^2*Eff_m*Kt*Km/Rm/r_mp^2

alpha2 = Eff_g*Kg*Eff_m*Kt/Rm/r_mp





% you may enter script here

A = [0 0 1 0; 0 0 0 1; 0 m*g/M alpha1/M 0; 0 ((m+M)*g)/(l*M) alpha1/(l*M) 0];
B = [0; 0; alpha2/M; alpha2/(l*M)];
C = [1 0 0 0; 0 1 0 0];
D = [0 0]';
K = place(A, -B, [-2 -4 -10 -20]);
L = place(A', C', [-40 -40 -50 -50])';

