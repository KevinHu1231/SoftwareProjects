% =========
% ass3_q1.m
% =========
%
% This assignment will introduce you to the idea of first building an
% occupancy grid then using that grid to estimate a robot's motion using a
% particle filter.
% 
% There are two questions to complete (5 marks each):
%
%    Question 1: code occupancy mapping algorithm 
%    Question 2: see ass3_q2.m
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plot/movie, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file
% and the two resulting AVI files from Questions 1 and 2.
%
% requires: basic Matlab, 'gazebo.mat'
%
% T D Barfoot, January 2016
%
clear all;

% set random seed for repeatability
rng(1);

% ==========================
% load the dataset from file
% ==========================
%
%    ground truth poses: t_true x_true y_true theta_true
% odometry measurements: t_odom v_odom omega_odom
%           laser scans: t_laser y_laser
%    laser range limits: r_min_laser r_max_laser
%    laser angle limits: phi_min_laser phi_max_laser
%
load gazebo.mat;

% =======================================
% Question 1: build an occupancy grid map
% =======================================
%
% Write an occupancy grid mapping algorithm that builds the map from the
% perfect ground-truth localization.  Some of the setup is done for you
% below.  The resulting map should look like "ass2_q1_soln.png".  You can
% watch the movie "ass2_q1_soln.mp4" to see what the entire mapping process
% should look like.  At the end you will save your occupancy grid map to
% the file "occmap.mat" for use in Question 2 of this assignment.

% allocate a big 2D array for the occupancy grid
ogres = 0.05;                   % resolution of occ grid
ogxmin = -7;                    % minimum x value
ogxmax = 8;                     % maximum x value
ogymin = -3;                    % minimum y value
ogymax = 6;                     % maximum y value
ognx = (ogxmax-ogxmin)/ogres;   % number of cells in x direction
ogny = (ogymax-ogymin)/ogres;   % number of cells in y direction
oglo = zeros(ogny,ognx);        % occupancy grid in log-odds format
ogp = zeros(ogny,ognx);         % occupancy grid in probability format

% precalculate some quantities
numodom = size(t_odom,1);
npoints = size(y_laser,2);
angles = linspace(phi_min_laser, phi_max_laser,npoints);
dx = ogres*cos(angles);
dy = ogres*sin(angles);

% interpolate the noise-free ground-truth at the laser timestamps
t_interp = linspace(t_true(1),t_true(numodom),numodom);
x_interp = interp1(t_interp,x_true,t_laser);
y_interp = interp1(t_interp,y_true,t_laser);
theta_interp = interp1(t_interp,theta_true,t_laser);
omega_interp = interp1(t_interp,omega_odom,t_laser);
  
% set up the plotting/movie recording
vid = VideoWriter('ass2_q1.avi');
open(vid);
figure(1);
clf;
pcolor(ogp);
colormap(1-gray);
shading('flat');
axis equal;
axis off;
M = getframe;
writeVideo(vid,M);

% loop over laser scans (every fifth)
for i=1:5:size(t_laser,1)
    
    % ------insert your occupancy grid mapping algorithm here------

    grid_x = floor((x_interp(i)-ogxmin)/ogres);
    grid_y = floor((y_interp(i)-ogymin)/ogres);

    for j=1:npoints

        if isnan(y_laser(i,j))
            continue
        end

        grid_dist = y_laser(i,j)/0.05;
        phi = theta_interp(i) + angle(j);
    
        if phi > pi
            phi = phi - 2*pi;
        elseif phi < -pi
            phi = phi + 2*pi;
        end 
        
        if (phi < 3*pi/4 && phi > pi/4) || (phi > -3*pi/4 && phi < -pi/4)
            
            grid_dist_y = ceil(grid_dist*sin(phi));
        
            if grid_dist_y + grid_y > ogny
                grid_dist_y = ogny-grid_y;
            elseif grid_dist_y + grid_y < 0
                grid_dist_y = 0 - grid_y;
            end

            if grid_dist_y < 0
                y_idxs = -(1:abs(grid_dist_y));
            else
                y_idxs = 1:grid_dist_y;
            end

            x_idxs = y_idxs/tan(phi);
        
        else

            grid_dist_x = ceil(grid_dist*cos(phi));

            if grid_dist_x + grid_x > 300
                grid_dist_x = ognx - grid_x;
            elseif grid_dist_x + grid_x < 0
                grid_dist_x = 0 - grid_x;
            end
        
            if grid_dist_x < 0
                x_idxs = -(1:abs(grid_dist_x));
            else
                x_idxs = 1:grid_dist_x;
            end

            y_idxs = x_idxs*tan(phi);

        end
        
        x_idxs = round(x_idxs + grid_x,0);
        y_idxs = round(y_idxs + grid_y,0);

        for idx=1:size(x_idxs, 2)

            if y_idxs(idx) > 180 || x_idxs(idx) > 300 || y_idxs(idx) == 0 || x_idxs(idx) == 0
                continue
            end 

            if idx < (size(x_idxs,2))
                oglo(y_idxs(idx),x_idxs(idx)) = oglo(y_idxs(idx),x_idxs(idx)) - 0.55;
            else
                oglo(y_idxs(idx),x_idxs(idx)) = oglo(y_idxs(idx),x_idxs(idx)) + 2;
            end

            ogp = exp(oglo)./(1.+exp(oglo));
        end

    end

    % ------end of your occupancy grid mapping algorithm-------

    % draw the map
    clf;
    pcolor(ogp);
    colormap(1-gray);
    shading('flat');
    axis equal;
    axis off;
    
    % draw the robot
    hold on;
    x = (x_interp(i)-ogxmin)/ogres;
    y = (y_interp(i)-ogymin)/ogres;
    th = theta_interp(i);
    r = 0.15/ogres;
    set(rectangle( 'Position', [x-r y-r 2*r 2*r], 'Curvature', [1 1]),'LineWidth',2,'FaceColor',[0.35 0.35 0.75]);
    set(plot([x x+r*cos(th)]', [y y+r*sin(th)]', 'k-'),'LineWidth',2);
    
    % save the video frame
    M = getframe;
    writeVideo(vid,M);
    
    pause(0.1);
    
end

close(vid);
print -dpng ass2_q1.png

save occmap.mat ogres ogxmin ogxmax ogymin ogymax ognx ogny oglo ogp;

