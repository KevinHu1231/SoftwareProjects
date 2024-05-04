function simulate(x_init, Controller, A, B)
    ts = linspace(0,10,1001);
    xs = zeros(4,size(ts,2));
    xs(:,1) = x_init;

    for i = 1:size(ts,2)-1
        x_dot = (A-B*Controller)*xs(:,i);
        x_new = xs(:,i)+x_dot*0.01;
        xs(:,i+1) = x_new;
    end
    subplot(2,2,1);
    plot(ts,xs(1,:))
    legend('y')
    subplot(2,2,2);
    plot(ts,xs(2,:))
    legend('ydot')
    subplot(2,2,3);
    plot(ts,xs(3,:))
    legend('theta')
    subplot(2,2,4);
    plot(ts,xs(4,:))
    legend('thetadot')
end