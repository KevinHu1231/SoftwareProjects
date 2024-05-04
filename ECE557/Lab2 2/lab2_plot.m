% Data plotting for ECE557 lab 2.
% Last modified August 29, 2017.

clear all;
close all;
clc;
delete(instrfindall);

% Ensure the following serial port is set to the same serial port being
% used by the Arduino controller.
serialPort = 'COM3';
serialObject = serial(serialPort, 'BaudRate', 115200);
fopen(serialObject);

% Set the time span and interval for data collection
stop_time = 10;
T_plot = 0.03;
n_samples = fix(stop_time/T_plot);

%% Set up the figure window

figure_handle = figure('NumberTitle','off', 'Name','Tracking');

% Set axes
axes_handle = axes('Parent',figure_handle, 'YGrid','on', 'XGrid','on');
hold on

plot_handle = plot(axes_handle, 0,0, 'Marker','.', 'LineWidth',1);
xlim(axes_handle, [0 stop_time]);
ylim(axes_handle, [-0.05 0.05]);

% Create xlabel
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);

% Create ylabel
ylabel('Position [m]', 'FontWeight','bold', 'FontSize',14);

%Create title
title('Cart Position as a Function of Time', ...
    'FontWeight','bold', 'FontSize',15);



%% Collect data
time = T_plot*(0:n_samples-1);
zero_matrix=zeros(1, n_samples);
cart_position_encoder=zero_matrix;
cart_position_ref=zero_matrix;

for count=2:n_samples
    cart_position_encoder(count) = fscanf(serialObject,'%f') ; 
    cart_position_ref(count) = fscanf(serialObject,'%f');
    
    % update the plot
      set(plot_handle, 'YData', cart_position_encoder(1:count), 'XData',time(1:count));
      set(figure_handle, 'Visible','on');
end

%% Plot all outstanding data
hline2 = plot(time, cart_position_ref, 'r', 'Linewidth',2);
hline1 = plot(time, cart_position_encoder, 'b', 'Marker','.','Linewidth',1);

xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14)
ylabel('Position [m]', 'FontWeight','bold', 'FontSize',14);
legend([hline1,hline2], 'Position', 'Reference Position');

hold off;


%% Clean up the serial object
fclose(serialObject);
delete(serialObject);
clear serialObject;
