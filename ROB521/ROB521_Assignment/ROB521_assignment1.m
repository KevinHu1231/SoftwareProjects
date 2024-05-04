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

x_s = (rand(nS,1)*col)+0.5;
y_s = (rand(nS,1)*row)+0.5;
pts = [x_s,y_s];
min_dists = MinDist2Edges(pts,map);
good_pts = pts((min_dists>0.1),:);
milestones = [start;good_pts;finish];
num_milestones = size(milestones,1);
N = 10;
[nn_inds,D] = knnsearch(milestones,milestones,'K',N+1,'Distance','euclidean');
nn_inds = nn_inds(:,2:size(nn_inds,2));
D = D(:,2:size(D,2));

edges = [0 0 0 0];
for i = 1:num_milestones
    for j = 1:N
        p1 = milestones(i,:);
        p2 = milestones(nn_inds(i,j),:);
        %if ismember([p2 p1],edges,'rows') == 0
        collision = CheckCollision(p1,p2,map);
        if collision == 0
            edges = [edges;[p1 p2]];
        else
            nn_inds(i,j) = NaN;
            D(i,j) = NaN;
        end
        %else
        %nn_inds(i,j) = NaN;
        %D(i,j) = NaN;
        %end
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

%find(ismember(A, [-2 1],'rows'))

milestone_ctg = pdist2(milestones,finish)
milestone_ctc = Inf(num_milestones,1)
milestone_f = Inf(num_milestones,1)
milestone_parent = NaN(num_milestones,1)
milestone_ctc(1) = 0
milestone_f(1) = milestone_ctg(1)

open_list = [start]
open_values = [milestone_f(1)]
closed_list = [NaN NaN]
failure = 1

while isempty(open_list) == 0
    x = open_list(1,:)
    open_list = open_list(2:size(open_list,1),:)
    open_values = open_values(2:size(open_values,1),:)
    if (x(1) == finish(1)) && (x(2) == finish(2))
        %milestone_parent(n_idx) = x_idx;
        failure = 0
        break
    end

    x_idx = find(ismember(milestones, x,'rows'))
    for i = 1:N
        n_idx = nn_inds(x_idx,i)
        if ~isnan(n_idx)
            n_pt = milestones(n_idx,:)
            %if (n_pt(1) == finish(1)) && (n_pt(2) == finish(2))
             %   milestone_parent(n_idx) = x_idx
              %  failure = 0
               % break
            %else
            g = milestone_ctc(x_idx) + D(x_idx,i)
            f = g + milestone_ctg(n_idx)
            %end
            if (ismember(n_pt,open_list,'rows') == 1) && (f >= milestone_f(n_idx))
                continue
            elseif (ismember(n_pt,closed_list,'rows') == 1) %&& (f >= milestone_f(n_idx))
                continue
            else
                if ismember(n_pt,closed_list,'rows') == 1
                     % Remove from closed list
                     closed_idx = find(ismember(closed_list,n_pt,'rows'))
                     closed_list = [closed_list(1:(closed_idx-1),:);closed_list((closed_idx+1):size(closed_list,1),:)]
                elseif ismember(n_pt,open_list,'rows') == 1
                    % Remove from open list and open list values
                    open_idx = find(ismember(open_list,n_pt,'rows'))
                    open_list = [open_list(1:(open_idx-1),:);open_list((open_idx+1):size(open_list,1),:)]
                    open_values = [open_values(1:(open_idx-1));open_values((open_idx+1):size(open_values))]
                end

                % Add to open list
                f_idx = find(open_values>f,1)
                if isempty(f_idx)
                    open_list = [open_list;n_pt]
                    open_values = [open_values;f]
                else
                    open_list = [open_list(1:(f_idx-1),:);n_pt;open_list(f_idx:size(open_list,1),:)]
                    open_values = [open_values(1:(f_idx-1));f;open_values(f_idx:size(open_values))]
                end

                % Update ctc and f lists
                milestone_ctc(n_idx) = g
                milestone_f(n_idx) = f

                % Update parent
                milestone_parent(n_idx) = x_idx
            end
        end
    end
    closed_list = [closed_list;x]
end

if failure == 0
    disp('SUCCESS');
else
    disp('FAILURE');
end

node_idx = num_milestones
spath = [num_milestones]
while ~(node_idx == 1)
    node_idx = milestone_parent(node_idx)
    spath = [node_idx;spath]
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

row = 25;
col = 25;
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

nS = round(500*(row*col)/35);  % number of samples to try for milestone creation

x_s = (rand(nS,1)*col)+0.5;
y_s = (rand(nS,1)*row)+0.5;
pts = [x_s,y_s];
min_dists = MinDist2Edges(pts,map);
good_pts = pts((min_dists>0.1),:);
milestones = [start;good_pts;finish];
num_milestones = size(milestones,1);
N = 10;
[nn_inds,D] = knnsearch(milestones,milestones,'K',N+1,'Distance','euclidean');
nn_inds = nn_inds(:,2:size(nn_inds,2));
D = D(:,2:size(D,2));

edges = [0 0 0 0];
for i = 1:num_milestones
    for j = 1:N
        p1 = milestones(i,:);
        p2 = milestones(nn_inds(i,j),:);
        %if ismember([p2 p1],edges,'rows') == 0
        collision = CheckCollision(p1,p2,map);
        if collision == 0
            edges = [edges;[p1 p2]];
        else
            nn_inds(i,j) = NaN;
            D(i,j) = NaN;
        end
        %else
        %nn_inds(i,j) = NaN;
        %D(i,j) = NaN;
        %end
    end
end
edges = edges(2:size(edges,1),:);

spath = []; % shortest path, stored as a milestone row index sequence

%find(ismember(A, [-2 1],'rows'))

milestone_ctg = pdist2(milestones,finish);
milestone_ctc = Inf(num_milestones,1);
milestone_f = Inf(num_milestones,1);
milestone_parent = NaN(num_milestones,1);
milestone_ctc(1) = 0;
milestone_f(1) = milestone_ctg(1);

open_list = [start];
open_values = [milestone_f(1)];
closed_list = [NaN NaN];
failure = 1;

while isempty(open_list) == 0
    x = open_list(1,:);
    open_list = open_list(2:size(open_list,1),:);
    open_values = open_values(2:size(open_values,1),:);
    if (x(1) == finish(1)) && (x(2) == finish(2))
        %milestone_parent(n_idx) = x_idx;
        failure = 0;
        break
    end

    x_idx = find(ismember(milestones, x,'rows'));
    for i = 1:N
        n_idx = nn_inds(x_idx,i);
        if ~isnan(n_idx)
            n_pt = milestones(n_idx,:);
            %if (n_pt(1) == finish(1)) && (n_pt(2) == finish(2))
                %milestone_parent(n_idx) = x_idx;
                %failure = 0;
                %break
            %else
            g = milestone_ctc(x_idx) + D(x_idx,i);
            f = g + milestone_ctg(n_idx);
            %end
            if (ismember(n_pt,open_list,'rows') == 1) && (f >= milestone_f(n_idx))
                continue
            elseif (ismember(n_pt,closed_list,'rows') == 1) %&& (f >= milestone_f(n_idx))
                continue
            else
                if ismember(n_pt,closed_list,'rows') == 1
                     % Remove from closed list
                     closed_idx = find(ismember(closed_list,n_pt,'rows'));
                     closed_list = [closed_list(1:(closed_idx-1),:);closed_list((closed_idx+1):size(closed_list,1),:)];
                elseif ismember(n_pt,open_list,'rows') == 1
                    % Remove from open list and open list values
                    open_idx = find(ismember(open_list,n_pt,'rows'));
                    open_list = [open_list(1:(open_idx-1),:);open_list((open_idx+1):size(open_list,1),:)];
                    open_values = [open_values(1:(open_idx-1));open_values((open_idx+1):size(open_values))];
                end

                % Add to open list
                f_idx = find(open_values>f,1);
                if isempty(f_idx)
                    open_list = [open_list;n_pt];
                    open_values = [open_values;f];
                else
                    open_list = [open_list(1:(f_idx-1),:);n_pt;open_list(f_idx:size(open_list,1),:)];
                    open_values = [open_values(1:(f_idx-1));f;open_values(f_idx:size(open_values))];
                end

                % Update ctc and f lists
                milestone_ctc(n_idx) = g;
                milestone_f(n_idx) = f;

                % Update parent
                milestone_parent(n_idx) = x_idx;
            end
        end
    end
    closed_list = [closed_list;x];
end

if failure == 0
    disp('SUCCESS');
else
    disp('FAILURE');
end

node_idx = num_milestones;
spath = [num_milestones];
while ~(node_idx == 1)
    node_idx = milestone_parent(node_idx);
    spath = [node_idx;spath];
end

if failure == 0
    disp('SUCCESS');
else
    disp('FAILURE');
end

node_idx = num_milestones;
spath = [num_milestones];
while ~(node_idx == 1)
    node_idx = milestone_parent(node_idx);
    spath = [node_idx;spath];
end

% ------end of your optimized algorithm-------
dt = toc;

figure(2); hold on;
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta')
end
if (~isempty(spath))
    for i=1:length(spath)-1
        plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
    end
end
str = sprintf('Q3 - %d X %d Maze solved in %f seconds', row, col, dt);
title(str);

print -dpng assignment1_q3.png

