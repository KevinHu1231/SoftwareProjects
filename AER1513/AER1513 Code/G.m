function G = G(x_hat_k,l,d)
    L = size(l,1);
    G = zeros(2*L,3);
    for i = 1:L
        denom = (l(i,1)-x_hat_k(1)-d*cos(x_hat_k(3)))^2+(l(i,2)-x_hat_k(2)-d*sin(x_hat_k(3)))^2;
        x_num = (x_hat_k(1)-l(i,1)+d*cos(x_hat_k(3)));
        y_num = (x_hat_k(2)-l(i,2)+d*sin(x_hat_k(3)));
        G_l = [x_num/sqrt(denom), y_num/sqrt(denom), (d*(l(i,1)-x_hat_k(1)+d*cos(x_hat_k(3)))*sin(x_hat_k(3))-d*(l(i,2)-x_hat_k(2)+d*sin(x_hat_k(3)))*cos(x_hat_k(3)))/sqrt(denom);
            -y_num/denom, x_num/denom, ((d*sin(x_hat_k(3))*y_num+d*cos(x_hat_k(3))*x_num)/denom)+1];
        G((2*i-1):2*i,:) = G_l;
    end
end