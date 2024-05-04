% ======
% ROB521_assignment1.m
% ======
% 
% This assignment will introduce you to the idea of motion planning for  
% holonomic robots that can move in any direction and change direction of 
% motion instantaneously.  Although unrealistic, it can work quite well for
% complex large scale planning.  You will generate mazes to plan through 
% and employ the PRM algorithm presented in lecture as well as any 
% variations you can invent in the later sections.
% 
% There are three questions to complete (5 marks each):
%
%    Question 1: implement the PRM algorithm to construct a graph
%    connecting start to finish nodes.
%    Question 2: find the shortest path over the graph by implementing the
%    Dijkstra's or A* algorithm.
%    Question 3: identify sampling, connection or collision checking 
%    strategies that can reduce runtime for mazes.
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plots, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file.
%
% requires: basic Matlab, 
%
% S L Waslander, January 2022
%
clear; close all; clc;

% set random seed for repeatability if desired
% rng(1);

% ==========================
% Maze Generation
% ==========================
%
% The maze function returns a map object with all of the edges in the maze.
% Each row of the map structure draws a single line of the maze.  The
% function returns the lines with coordinates [x1 y1 x2 y2].
% Bottom left corner of maze is [0.5 0.5], 
% Top right corner is [col+0.5 row+0.5]
%

row = 5; % Maze rows
col = 7; % Maze columns
map = maze(row,col); % Creates the maze
start = [0.5, 1.0]; % Start at the bottom left
finish = [col+0.5, row]; % Finish at the top right

h = figure(1);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

% ======================================================
% Question 1: construct a PRM connecting start and finish
% ======================================================
%
% Using 500 samples, construct a PRM graph whose milestones stay at least 
% 0.1 units away from all walls, using the MinDist2Edges function provided for 
% collision detection.  Use a nearest neighbour connection strategy and the 
% CheckCollision function provided for collision checking, and find an 
% appropriate number of connections to ensure a connection from  start to 
% finish with high probability.

% variables to store PRM components
nS = 500;  % number of samples to try for milestone creation
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]
disp("Time to create PRM graph")
tic;
% ------insert your PRM generation code here-------

% Note that code requires Statistical and Machine Learning Toolbox to run

% Generate random points
x_s = (rand(nS,1)*col)+0.5;
y_s = (rand(nS,1)*row)+0.5;
pts = [x_s,y_s];
% Find distance to maze walls
min_dists = MinDist2Edges(pts,map);
% Find all milestones that satisfy distance from wall requirement
good_pts = pts((min_dists>0.1),:);
milestones = [start;good_pts;finish];
num_milestones = size(milestones,1);
% Specify number of nearest neighbors guaranteeing high probability of path
N = 10;
% Generate edges and distances
[nn_inds,D] = knnsearch(milestones,milestones,'K',N+1,'Distance','euclidean');
nn_inds = nn_inds(:,2:size(nn_inds,2));
D = D(:,2:size(D,2));

%Check edges for collisions, only add those without collisions
edges = [0 0 0 0];
for i = 1:num_milestones
    for j = 1:N
        p1 = milestones(i,:);
        p2 = milestones(nn_inds(i,j),:);
        collision = CheckCollision(p1,p2,map);
        if collision == 0
            edges = [edges;[p1 p2]];
        else
            % Set edge and distance matrix to NaN if there is collision
            nn_inds(i,j) = NaN;
            D(i,j) = NaN;
        end
    end
end
edges = edges(2:size(edges,1),:);
% ------end of your PRM generation code -------
toc;

figure(1);
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta') % line uses [x1 x2 y1 y2]
end
str = sprintf('Q1 - %d X %d Maze PRM', row, col);
title(str);
drawnow;

print -dpng assignment1_q1.png


% =================================================================
% Question 2: Find the shortest path over the PRM graph
% =================================================================
%
% Using an optimal graph search method (Dijkstra's or A*) , find the 
% shortest path across the graph generated.  Please code your own 
% implementation instead of using any built in functions.

disp('Time to find shortest path');
tic;

% Variable to store shortest path
spath = []; % shortest path, stored as a milestone row index sequence
% ------insert your shortest path finding algorithm here-------

%Dijkstra's Algorithm Implementation
milestone_d = Inf(num_milestones,1);
milestone_p = NaN(num_milestones,1);
milestone_d(1) = 0;
Q = (1:1:num_milestones).';
Q_d = milestone_d;

while isempty(Q) == 0
    %Find entry with min distance in queue
    a = find(Q_d==min(Q_d));
    argmin = a(1);
    u = Q(argmin);
    %Delete from queue and distances
    Q = [Q(1:(argmin-1));Q((argmin+1):length(Q))];
    Q_d = [Q_d(1:(argmin-1));Q_d((argmin+1):length(Q_d))];
    
    for i = 1:N
        v = nn_inds(u,i); %Neighboring node
        if ~isnan(v) %For all neighbors without collision
            if (sum(Q==v)>0)
                %Alternate path length
                alt = milestone_d(u) + D(u,i);
                if alt < milestone_d(v)
                    %Improve and save path length and update parent
                    milestone_d(v) = alt;
                    argv = find(Q == v);
                    Q_d(argv) = alt;
                    milestone_p(v) = u;
                end
            end
        end
    end
end

% Backtrack from end to start using each nodes parent
node_idx = num_milestones;
spath = [node_idx];
while ~(node_idx == 1)
    node_idx = milestone_p(node_idx);
    spath = [node_idx;spath];
end

% ------end of shortest path finding algorithm------- 
toc;

% plot the shortest path
figure(1);
for i = 1:length(spath)-1
    plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
end
str = sprintf('Q2 - %d X %d Maze Shortest Path', row, col);
title(str);
drawnow;

print -dpng assingment1_q2.png


% ================================================================
% Question 3: find a faster way
% ================================================================
%
% Modify your milestone generation, edge connection, collision detection 
% and/or shortest path methods to reduce runtime.  What is the largest maze 
% for which you can find a shortest path from start to goal in under 20 
% seconds on your computer? (Anything larger than 40x40 will suffice for 
% full marks)

row = 60;
col = 60;
map = maze(row,col);
start = [0.5, 1.0];
finish = [col+0.5, row];
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

h = figure(2);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

fprintf("Attempting large %d X %d maze... \n", row, col);
tic;        
% ------insert your optimized algorithm here------

% Do smarter milestone generation in a grid style to assist with maze
% structure
[X,Y] = meshgrid(linspace(1,col,col),linspace(1,row,row));
pts = [X(:), Y(:)];

milestones = [start;pts;finish];
num_milestones = size(milestones,1);

% Reduce number of neighbors to 4 for each cardinal direction
N = 4;
[nn_inds,D] = knnsearch(milestones,milestones,'K',N+1,'Distance','euclidean');
nn_inds = nn_inds(:,2:size(nn_inds,2));
D = D(:,2:size(D,2));

% Rest of algorithm stays the same. Collision checking takes the most time
% so reducing number of nearest neighbors (collision checks) reduces
% the runtime the most.

edges = [0 0 0 0];
for i = 1:num_milestones
    for j = 1:N
        p1 = milestones(i,:);
        p2 = milestones(nn_inds(i,j),:);

        collision = CheckCollision(p1,p2,map);
        if collision == 0
            edges = [edges;[p1 p2]];
        else
            nn_inds(i,j) = NaN;
            D(i,j) = NaN;
        end

    end
end
edges = edges(2:size(edges,1),:);

figure(2); hold on;
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta')
end
drawnow;

spath = []; % shortest path, stored as a milestone row index sequence


milestone_d = Inf(num_milestones,1);
milestone_p = NaN(num_milestones,1);
milestone_d(1) = 0;
Q = (1:1:num_milestones).';
Q_d = milestone_d;

while isempty(Q) == 0
    a = find(Q_d==min(Q_d));
    argmin = a(1);
    u = Q(argmin);
    Q = [Q(1:(argmin-1));Q((argmin+1):length(Q))];
    Q_d = [Q_d(1:(argmin-1));Q_d((argmin+1):length(Q_d))];
    for i = 1:N
        v = nn_inds(u,i);
        if ~isnan(v)
            if any(Q==v)
                alt = milestone_d(u) + D(u,i);
                if alt < milestone_d(v)
                    milestone_d(v) = alt;
                    argv = find(Q == v);
                    Q_d(argv) = alt;
                    milestone_p(v) = u;
                end
            end
        end
    end
end

node_idx = num_milestones;
spath = [node_idx];
while ~(node_idx == 1)
    node_idx = milestone_p(node_idx);
    spath = [node_idx;spath];
end

% ------end of your optimized algorithm-------
dt = toc;

if (~isempty(spath))
    for i=1:length(spath)-1
        plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
    end
end

str = sprintf('Q3 - %d X %d Maze solved in %f seconds', row, col, dt);
title(str);

print -dpng assignment1_q3.png
