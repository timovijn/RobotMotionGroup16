%% RRT Ambulance on Highway
% Robot Motion and Planning, Group 16

% Navigation Toolbox needed for running of code
% Automated Driving Toolbox needed for running of code

clear all
close all
clc

%% Custom User Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% LANES
lanes.count = 5 ;
lanes.width = 7 ;       % Pixel coordinates (x1/2)  % Must be uneven
lanes.length = 500 ;     % Pixel coordinates (x1/2) %thus length of langes = half of this number

% OBSTACLE VEHICLES
obs.width = 5 ;         % Pixel coordinates (x1/2) 
obs.length = 10 ;       % Pixel coordinates (x1/2) (aka is 5 in costmap)

% Obstacles space for 4 different scenarios 
% if time: make this easier implementable
% obs.lane = [1 1 2 3 3 4 5 5];        % Vehicle lane     % Pixel coordinates (x1/2)
% obs.x = [100 250 400 150 300 80 200 420]; % Vehicle rear     % Pixel coordinates (x1/2)

obs.lane = [4 4 2 1 1 1 1 4 3 4 1 1 2 1 1 2 4 4 4 5 5 5 5 5 5];        % Vehicle lane     % Pixel coordinates (x1/2)
obs.x = [40 60 80 100 120 140 160 200 220 240 300 320 340 360 400 360 400 440 470 100 150 200 300 400 450];   % Vehicle rear     % Pixel coordinates (x1/2)

% obs.lane = [1 1 2 3 3 4 5 5 6 6 7 7 8 8 ];        % Vehicle lane     % Pixel coordinates (x1/2)
% obs.x = [100 250 400 150 300 80 200 420 200 250 450 120 300 470]; % Vehicle rear     % Pixel coordinates (x1/2)

% obs.lane = [4 4 2 1 1 1 1 4 3 4 1 1 2 1 1 2 4 4 4 5 5 5 5 5 5 6 6 7 8 8 8 8 8 8];        % Vehicle lane     % Pixel coordinates (x1/2)
% obs.x = [40 60 80 100 120 140 160 200 220 240 300 320 340 360 400 360 400 440 470 100 150 200 300 400 450 200 250 120 300 470 100 160 350 40];   % Vehicle rear     % Pixel coordinates (x1/2)


obs.count = length(obs.lane) ;

% EGO VEHICLE
ego.width = 2.5 ;           % Real coordinates (x1)
ego.length = 5 ;            % Real coordinates (x1)

ego.x.start = 10 ;          % Real coordinates (x1)
ego.y.start = 10 ;          % Real coordinates (x1)

ego.x.goal = 248 ;          % Real coordinates (x1)
ego.y.goal = 10 ;           % Real coordinates (x1)

ego.speed.max = 5 ;         % m/s
ego.speed.start = 0 ;       % m/s
ego.speed.end = 0 ;         % m/s

%% World definition  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% LANES
lanes.center = zeros(1,lanes.count) ;
for i = 1:lanes.count
    lanes.center(i) = ceil(lanes.width/2)+(i-1)*lanes.width ;
end

%%  Construct world      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% CONSTRUCT LANES
world = zeros(lanes.count*lanes.width,lanes.length) ;

% CONSTRUCT OBSTACLE CARS
for i = 1:obs.count
    world(lanes.center(obs.lane(i))-floor(obs.width/2):lanes.center(obs.lane(i))+floor(obs.width/2) , obs.x(i):obs.x(i)+obs.length-1) = 1;
end

% CONSTRUCT LANE SIDES
world = [ones(1,lanes.length) ; world ; ones(1,lanes.length)] ;

res = 0.5;

costmap = vehicleCostmap(world,'CellSize',res);
costmap.CollisionChecker.InflationRadius = 2.5;

figure
plot(costmap, 'Inflation', 'off')
legend off

%% RRT*

figure(6)
plot(costmap)
hold on

n = 300; % number of iterations
initR = 12; %search radius

costRRT = zeros(10);
costRRTstar = zeros(10);
[nodes] = RRT_star(costmap, ego, initR, res, lanes, n) ;


%% Comparison

motionPlanner = pathPlannerRRT(costmap, 'MinIterations', n, ...
    'ConnectionDistance', initR, 'MinTurningRadius', 10.4);
    
[refPath,tree] = plan(motionPlanner, [10,10,0], [248,10,0]);

% CostRRT(i) = nodes(n+2).dubinscost;
% CostRRTstar(i) = refPath.Length;
% 
% av_costRRT = sum(CostRRT)/10;
% av_costRRTstar = sum(CostRRTstar)/10;

% figure(3)
% plot(costmap)
% hold on
% plot(refPath)
