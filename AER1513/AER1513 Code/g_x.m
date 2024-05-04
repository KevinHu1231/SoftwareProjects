function y_k = g_x(x_hat_k,l,d)
    L = size(l,1);
    y_k = zeros(2*L,1);
    for i = 1:L
        x_part = l(i,1)-x_hat_k(1)-d*cos(x_hat_k(3));
        y_part = l(i,2)-x_hat_k(2)-d*sin(x_hat_k(3));
        y_lk = [sqrt(x_part^2+y_part^2);(atan2(y_part,x_part)-x_hat_k(3))];
        y_k((2*i-1):2*i,1) = y_lk;
    end
end