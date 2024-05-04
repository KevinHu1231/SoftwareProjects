clear
clc
%Load dataset
load('dataset2.mat')
r_maxs = [5,3,1];

T = t(2) - t(1);
K = length(t)-1;

x_hat_0 = [x_true(1);y_true(1);th_true(1)];
vars_0 = [1,1,0.1];
P_hat_0 = diag(vars_0);

vs = [v.';om.'];

for r_max = r_maxs

    x_hat = zeros(3,K+1);
    vars = zeros(3,K+1);
    x_hat(:,1) = x_hat_0;
    vars(:,1) = vars_0;

    x_hat_k = x_hat_0;
    P_hat_k = P_hat_0;
    for k = 1:K
        r_k = r(k,:);
        b_k = b(k,:);
        cond_1 = r_k ~= 0;
        cond_2 = r_k < r_max;
        r_k_valid = cond_1 & cond_2;
        r_k_new = r_k(r_k_valid);
        b_k_new = b_k(r_k_valid);
        r_k_valid2 = repmat(r_k_valid.',1,2);
        l_new = reshape(l(r_k_valid2),[],2);
        
        F_k_1 = F(x_hat_k,v(k),T);
        G_k = G(x_hat_k,l_new,d);

        P_check_k = F_k_1*P_hat_k*F_k_1.' + Qp(x_hat_k,v_var,om_var,T);
        x_check_k = h(x_hat_k,vs(:,k),T);
        x_check_k(3) = wrapToPi(x_check_k(3));

        K_k = (P_check_k*G_k.')/(G_k*P_check_k*G_k.' + Rp(l_new,r_var,b_var));
        P_hat_k = (eye(3)-K_k*G_k)*P_check_k;

        innovation = y(r_k_new,b_k_new)-g_x(x_hat_k,l_new,d);
        innovation = wrapinno(innovation);

        x_hat_k = x_check_k + K_k*innovation;
        x_hat_k(3) = wrapToPi(x_hat_k(3));

        x_hat(:,k) = x_hat_k;
        vars(:,k) = diag(P_hat_k);
    end
    stds = sqrt(vars);

    % Difference between predicted and true state
    x_diff = x_hat(1,:).' - x_true;
    y_diff = x_hat(2,:).' - y_true;
    th_diff = x_hat(3,:).' - th_true;

    % Upper and lower bounds of standard deviation (+/- 3 std deviations)
    x_upper = 3*stds(1,:).';
    x_lower = -3*stds(1,:).';

    y_upper = 3*stds(2,:).';
    y_lower = -3*stds(2,:).';

    th_upper = 3*stds(3,:).';
    th_lower = -3*stds(3,:).';
    
    % Plot difference between predicted and true state
    
    figure
    plot(t,x_upper,':',t,x_lower,':',t,x_diff,'-');
    title(["xhatk - xk as a Function of Time for rmax = ", num2str(r_max)]);
    xlabel("Time t (s)");
    ylabel("xhatk - xk (m)");
    legend(["+3 Standard Deviations", "-3 Standard Deviations","xhatk - xk"]);
    fname = sprintf('Plotx%d.png', r_max);
    saveas(gcf,fname)

    figure
    plot(t,y_upper,':',t,y_lower,':',t,y_diff,'-');
    title(["yhatk - yk as a Function of Time for rmax = ", num2str(r_max)]);
    xlabel("Time t (s)");
    ylabel("yhatk - yk (m)");
    legend(["+3 Standard Deviations", "-3 Standard Deviations","yhatk - yk"]);
    fname = sprintf('Ploty%d.png', r_max);
    saveas(gcf,fname)

    figure
    plot(t,th_upper,':',t,th_lower,':',t,th_diff,'-');
    title(["thetahatk - thetak as a Function of Time for rmax = ", num2str(r_max)]);
    xlabel("Time t (s)");
    ylabel("thetahatk - thetak (m)");
    legend(["+3 Standard Deviations", "-3 Standard Deviations","thetahatk - thetak"]);
    fname = sprintf('Plottheta%d.png', r_max);
    saveas(gcf,fname)
end
