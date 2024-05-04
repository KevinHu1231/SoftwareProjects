%% Optimal Q

K = place(A, -B, [-2 -4 -10 -20]);

y_plot = zeros(size(y(:,1),1),4);
y_hat = zeros(size(y(:,1),1),4);
y_bar = zeros(size(y(:,1),1),1);

y_plot(:,1) = y(:,1);
y_plot(:,2) = theta(:,1);
y_plot(:,3) = y_dot(:,1);
y_plot(:,4) = theta_dot(:,1);

y_hat(:,1) = y(:,2);
y_hat(:,2) = theta(:,2);
y_hat(:,3) = y_dot(:,2);
y_hat(:,4) = theta_dot(:,2);

y_bar(:,1) = y(:,3);

plotter(linspace(0,30,size(y(:,2),1)), y_plot, y_hat, y_bar, 'State Variables for Cart with Pole Placement');

%%

q1 = 1; q2 = 4; r = 0.0005;
Q1 = [q1 0 0 0; 0 q2 0 0; 0 0 0 0; 0 0 0 0];
K = -lqr(A, B, Q1, r);

y_plot = zeros(size(y(:,1),1),4);
y_hat = zeros(size(y(:,1),1),4);
y_bar = zeros(size(y(:,1),1),1);

y_plot(:,1) = y(:,1);
y_plot(:,2) = theta(:,1);
y_plot(:,3) = y_dot(:,1);
y_plot(:,4) = theta_dot(:,1);

y_hat(:,1) = y(:,2);
y_hat(:,2) = theta(:,2);
y_hat(:,3) = y_dot(:,2);
y_hat(:,4) = theta_dot(:,2);

y_bar(:,1) = y(:,3);

plotter(linspace(0,30,size(y(:,2),1)), y_plot, y_hat, y_bar, 'State Variables Optimal K');

%%

syms a11 a12 a13 a14 a21 a22 a23 a24 a31 a32 a33 a34 a41 a42 a43 a44
syms L_11 L_12 L_21 L_22 L_31 L_32 L_41 L_42

Ax = [a11 a12 a13, a14; a21 a22 a23 a24; a31 a32 a33 a34; a41 a42 a43 a44];
Lx = [L_11 L_12; L_21 L_22; L_31 L_32; L_41 L_42];
Ax - Lx*C;

syms b1 b2 b3 b4
Bx = [b1; b2; b3; b4];
[Bx Lx]


%% Plotting code

function ret = plotter(t, x, x1,x2, name)
    
    % Overall title name
    plotTitle = ['\textbf{', name, '}'];
    sgtitle(plotTitle,'Interpreter','latex');
    
    % y(t)
    subplot(2,2,1);
    plot(t,x(:,1));
    hold on;
    plot(t,x1(:,1));
    hold on;
    plot(t,x2(:,1));
    legend('y', '$\hat{y}$', '$\bar{y}$', 'Interpreter', 'latex');
    title('y(t)','Interpreter','latex')
    xlabel('t[s]','Interpreter','latex')
    ylabel('$y(t)[m]$','Interpreter','latex')
    
    % theta(t)
    subplot(2,2,2);
    plot(t,x(:,2));
    hold on;
    plot(t,x1(:,2));
    legend('y', '$\hat{y}$', '$\bar{y}$', 'Interpreter', 'latex');
    title('$\theta(t)$','Interpreter','latex')
    xlabel('t[s]','Interpreter','latex')
    ylabel('$\theta$[rad]','Interpreter','latex')
    
    % \dot{y}(t)
    subplot(2,2,3);
    plot(t,x(:,3));
    hold on;
    plot(t,x1(:,3));
    legend('y', '$\hat{y}$', '$\bar{y}$', 'Interpreter', 'latex');
    title('$\dot{y}$(t)','Interpreter','latex')
    xlabel('t[s]','Interpreter','latex')
    ylabel('$\dot{y}$(t)[m/s]','Interpreter','latex')
    
    % \dot{theta}(t)
    subplot(2,2,4);
    plot(t,x(:,4));
    hold on;
    plot(t,x1(:,4));
    legend('y', '$\hat{y}$', '$\bar{y}$', 'Interpreter', 'latex');
    title('$\dot{\theta}(t)$','Interpreter','latex')
    xlabel('t[s]','Interpreter','latex')
    ylabel('$\dot{\theta}$(t)[rad/s]','Interpreter','latex')
    
    % u(t)
    %subplot(3,2,5);
    %plot(t,u);
    %title('u(t)','Interpreter','latex')
    
    ret = 1;
    
end





