function [vbibD] = vbib(D)
    a = -7.7835*10^(-7);
    b = -0.008112;
    c = 12.3063;
    d_coeff = -0.000328;
    e = -4.7809*10^(-7);
    f = 1.4086*10^(-10);
    vbibD = (a*D^2+b*D+c)/(1+d_coeff*D+e*D^2+f*D^3);
end