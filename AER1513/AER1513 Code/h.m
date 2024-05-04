function x_k = h(x_hat_k,v_k,T)
    x_k = x_hat_k + T*[cos(x_hat_k(3)), 0 ; sin(x_hat_k(3)), 0 ; 0, 1]*v_k;
end