clear
clc

syms x y th T v om w1 w2 d real
syms xl yl nl1 nl2 real

h = [x + T*cos(th)*(v+w1); y + T*sin(th)*(v+w1) ; th + T*(om+w2)];
h_syms = [x,y,th,v,om,w1,w2,T];

x_ = [x,y,th];
F = jacobian(h,x_)
F_syms = [th, v, w1, T];

gl = [sqrt((xl - x - d*cos(th))^2 + (yl - y - d*sin(th))^2) + nl1 ; atan2((yl - y - d*sin(th)), (xl - x - d*cos(th))) - th + nl2];


gl_syms = [x,y,th,xl,yl,nl1,nl2];

G_l = simplify(jacobian(gl,x_))
G_l_syms = [x,y,th,xl,yl,nl1,nl2];

w_ = [w1,w2];
nl_ = [nl1,nl2];
w_p = jacobian(h,w_)
nl_p = jacobian(gl,nl_)
