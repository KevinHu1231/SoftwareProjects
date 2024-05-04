syms fx s cx fy cy si theta phi tx ty tz w1 w2 w3
K = [fx s cx; 0 fy cy; 0 0 1]
C = [cos(si)*cos(theta) cos(si)*sin(theta)*sin(phi)-sin(si)*cos(phi) cos(si)*sin(theta)*cos(phi)+sin(si)*cos(phi);
    sin(si)*cos(theta) sin(si)*sin(theta)*sin(phi)+cos(si)*cos(phi) sin(si)*sin(theta)*cos(phi)-cos(si)*sin(phi);
    -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)]
t = [tx; ty; tz]
C_t = [C,t]

T = [C_t; 0 0 0 1]
T_inv = inv(T)
C_t_inv = T_inv(1:3,1:4)

pw = [w1; w2; w3; 1];
X = -K*C_t_inv*pw;
X1 = X(1)/X(3);
X2 = X(2)/X(3);
dX1_dtx = diff(X1,tx);
dX2_dtx = diff(X2,tx);
dX1_dty = diff(X1,ty);
dX2_dty = diff(X2,ty);
dX1_dtz = diff(X1,tz);
dX2_dtz = diff(X2,tz);
dX1_dphi = diff(X1,phi);
dX2_dphi = diff(X2,phi);
dX1_dtheta = diff(X1,theta);
dX2_dtheta = diff(X2,theta);
dX1_dsi = diff(X1,si);
dX2_dsi = diff(X2,si);

J11 = double(subs(dX1_dtx,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J12 = double(subs(dX1_dty,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J13 = double(subs(dX1_dtz,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J14 = double(subs(dX1_dphi,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J15 = double(subs(dX1_dtheta,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J16 = double(subs(dX1_dsi,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J21 = double(subs(dX2_dtx,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J22 = double(subs(dX2_dty,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J23 = double(subs(dX2_dtz,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J24 = double(subs(dX2_dphi,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J25 = double(subs(dX2_dtheta,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));
J26 = double(subs(dX2_dsi,[fx s cx fy cy si theta phi tx ty tz w1 w2 w3],[564.9 0 337.3 564.3 226.5 -0.25651262 -0.11710196 -3.10564615 0.201090356081375 0.114474051344464 1.193821106321156 0.0635, 0, 0]));

J = [J11 J12 J13 J14 J15 J16; J21 J22 J23 J24 J25 J26];

%J = [dX1_dtx dX1_dty dX1_dtz dX1_dphi dX1_dtheta dX1_dsi; dX2_dtx dX2_dty dX2_dtz dX2_dphi dX2_dtheta dX2_dsi]
