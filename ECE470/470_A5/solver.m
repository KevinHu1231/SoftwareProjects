syms q1(t) q2(t) m1 m2 I2 g tau1 tau2
ode1 = (m1+m2)*diff(q1,2)+m2*cos(q2)*diff(q2,2)-m2*sin(q2)*(diff(q2))^2+g*(0.5*m1+m2) == tau1;
ode2 = m2*cos(q2)*diff(q1,2)+(m2+I2)*diff(q2,2)+g*m2*cos(q2) == tau2;
cond1 = q1(0) == 0;
cond2 = q2(0) == 0;
conds = [cond1; cond2];
odes = [ode1, ode2]
S = dsolve(odes)
%% 
syms q1 q2 m1 m2 I2 g tau1 tau2
eqn = tau1 - (m1+m2)*(tau2-g*m2*cos(q2))/(m2*cos(q2))-g*(0.5*m1+m2) == 0
Seq = solve(eqn,q2);
Seq = Seq(2,1)
%% 
syms x1 x2 x3 x4
f2 = (tau1-m2*cos(x3)*(-g*m2*cos(x3)+tau2)/(m2+I2)+m2*sin(x3)*(x4)^2-g*(0.5*m1+m2))/(m1+m2-(m2^2*cos(x3)^2)/(m2+I2))
f4 = (tau1-(m1+m2)*(tau2-g*m2*cos(x3))/(m2*cos(x3))+m2*sin(x3)*(x4)^2)/(m2+I2-((m1+m2)*(m2+I2)/(m2*cos(x3))))
%% 
clear all
%% 
syms q1dd q2dd x1 x2 x3 x4 m1 m2 I2 g tau1 tau2
eqn1 = (m1+m2)*q1dd + m2*cos(x3)*q2dd-m2*sin(x3)*x4^2+g*(0.5*m1+m2) == tau1;
eqn2 = m2*cos(x3)*q1dd + (m2+I2)*q2dd+g*m2*cos(x3) == tau2;
eqns = [eqn1;eqn2];
S = solve(eqns,[q1dd;q2dd]);
f2 = S.q1dd
f4 = S.q2dd
%% 
f2new = subs(f2,x1,0);
f2new = subs(f2new,x2,0);
f2new = subs(f2new,x4,0)

f4new = subs(f4,x1,0);
f4new = subs(f4new,x2,0);
f4new = subs(f4new,x4,0)
%% 
Seq1 = solve(f2new,x3);
Seq2 = solve(f4new,x3);
x3_eq = Seq2(2,1)
%% 
df2_dx3 = diff(f2,x3)
df2_dx4 = diff(f2,x4)
df4_dx3 = diff(f4,x3)
df4_dx4 = diff(f4,x4)
%% 
df2_dx3_eq = subs(df2_dx3,x1,0);
df2_dx3_eq = subs(df2_dx3_eq,x2,0);
df2_dx3_eq = subs(df2_dx3_eq,x3,x3_eq);
df2_dx3_eq = subs(df2_dx3_eq,x4,0)

df2_dx4_eq = subs(df2_dx4,x1,0);
df2_dx4_eq = subs(df2_dx4_eq,x2,0);
df2_dx4_eq = subs(df2_dx4_eq,x3,x3_eq);
df2_dx4_eq = subs(df2_dx4_eq,x4,0)

df4_dx3_eq = subs(df4_dx3,x1,0);
df4_dx3_eq = subs(df4_dx3_eq,x2,0);
df4_dx3_eq = subs(df4_dx3_eq,x3,x3_eq);
df4_dx3_eq = subs(df4_dx3_eq,x4,0)

df4_dx4_eq = subs(df4_dx4,x1,0);
df4_dx4_eq = subs(df4_dx4_eq,x2,0);
df4_dx4_eq = subs(df4_dx4_eq,x3,x3_eq);
df4_dx4_eq = subs(df4_dx4_eq,x4,0)
%% 
df2_du1 = diff(f2,tau1)
df2_du2 = diff(f2,tau2)
df4_du1 = diff(f4,tau1)
df4_du2 = diff(f4,tau2)
%% 
A = [0 1 0 0; 0 0 df2_dx3_eq df2_dx4_eq; 0 0 0 1; 0 0 df4_dx3_eq df4_dx4_eq]
B = [0 0; df2_du1 df2_du2; 0 0; df4_du1 df4_du2]
%% 
syms t x1_0 x2_0 x3_0 x4_0
S = expm(A*t)*[x1_0;x2_0;x3_0;x4_0] + [0; 0; x3_eq; 0]
%% 
%Snew = subs(S,m1,1);
%Snew = subs(Snew,m2,1);
%Snew = subs(Snew,g,10);
%Snew = subs(Snew,I2,1);
Snew = subs(S,x1_0,0);
Snew = subs(Snew,x2_0,0);
Snew = subs(Snew,x3_0,0);
Snew = subs(Snew,x4_0,0)