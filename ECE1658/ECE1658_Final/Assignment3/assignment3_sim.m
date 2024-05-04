% Set tolerance
options = odeset('RelTol',1e-50,'AbsTol',1e-50);

% Define the system of ODEs
x1_dot = @(t, x) cos(x(3));
x2_dot = @(t, x) sin(x(3));
x3_dot = @(t, x) 1/round(x(1)*sin(x(3))-x(2)*cos(x(3)),10) + (-10*(x(1)^2+x(2)^2-1) + -10*(2*x(1)*cos(x(3))+2*x(2)*sin(x(3))))/round(2*x(2)*cos(x(3))-2*x(1)*sin(x(3)),10);

% Define the time span for simulation
t_span = [0 500]; % From t=0 to t=10

%Plot single solution
%initial_condition = [10;10;pi/4-pi];
%[t, sol] = ode45(@(t, x) [x1_dot(t, x); x2_dot(t, x); x3_dot(t, x)], t_span, initial_condition);
%plot(sol(:, 1),sol(:, 2))

%Number of trials
N = 1000;
x1_min = -2; % Lower bound of the range
x1_max = 2; % Upper bound of the range
x1_init = x1_min + (x1_max - x1_min) * rand([1, N]); %Uniform Distribution

x2_min = -2; % Lower bound of the range
x2_max = 2; % Upper bound of the range
x2_init = x2_min + (x2_max - x2_min) * rand([1, N]); %Uniform Distribution

x3_min = 0; % Lower bound of the range
x3_max = 2*pi; % Upper bound of the range
x3_init = x3_min + (x3_max - x3_min) * rand([1, N]); %Uniform Distribution

initial_conditions = [x1_init;x2_init;x3_init];

% Solutions that converge to Z1 or Z2
Z_1_counter = 0;
Z_2_counter = 0;

for i = 1:N
    % Define the initial condition
    initial_condition = initial_conditions(:,i); % Initial values of x and y
    % Call ode45 to solve the system of ODEs
    [t, sol] = ode45(@(t, x) [x1_dot(t, x); x2_dot(t, x); x3_dot(t, x)], t_span, initial_condition);

    if isnan(sol(end,end))
        fprintf('Integration interrupted')
    else
        % Plot the solution
        x1 = sol(:, 1);
        x2 = sol(:, 2);
        x3 = sol(:, 3);

        plot(x1, x2)
        hold on
        
        size(sol);
        % Find solution type
        if x3(end)-x3(end-1)>0
            'clockwise - Z_2'
            Z_2_counter = Z_2_counter + 1;

        elseif x3(end)-x3(end-1)<0 
            'counterclockwise - Z_1'
            Z_1_counter = Z_1_counter + 1;
        else
            'error'
        end
    end

end
%Label figure
xlabel('x');
ylabel('y');
xlim([-2 2]);
ylim([-2 2]);
title('Simulation of the System of ODEs');
grid on;