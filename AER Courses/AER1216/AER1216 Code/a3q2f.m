clear
clear all
clear clc

D = 1;
t = 0;
delta_t = 0.1;
vet = 34.1320;
im = 0.8781;
bat = 1300;
while 1
    g = vbib(D);
    syms vb ib ke
    eq1 = vb - (vet/ke);
    eq2 = ib - im*ke;
    eq3 = vb*(ib^0.05) - g;
    [vb_val,ib_val,ke_val] = solve(eq1,eq2,eq3,vb,ib,ke);
    vb_val = double(vb_val);
    ib_val = double(ib_val);
    ke_val = double(ke_val);
    val = (ib_val)*(delta_t);
    D = D + (ib_val/1000)*(delta_t/3600);
    t = t + delta_t;
    if ((D>bat) || (ke_val>1))
        break
    end
end
te = t