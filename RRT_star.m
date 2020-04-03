function [nodes] = RRT_star(costmap, ego, initR, res, lanes,n)
    %% Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    nodes_count = n ;          % Amount of nodes

    q_start.coord = [ego.x.start ego.y.start] ;
    q_start.cost = 0 ;
    q_start.parent = 0 ;
    q_start.theta = 0 ;
    q_start.index = 1;
    q_start.dubinscost = 0;
    q_start.path = cell(1);

    q_goal.coord = [ego.x.goal ego.y.goal] ;
    q_goal.cost = 0 ;
    q_goal.theta = 0 ;

    nodes(1) = q_start ;
    gif_frame = 1;

    %% Step 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    while length(nodes) <= nodes_count

        W = waitbar(length(nodes)/nodes_count);
        q_rand.coord = [rand(1)*lanes.length*res rand(1)*lanes.width*lanes.count*res] ;

%       Break if goal node is already reached
        for j = 1:length(nodes)
            if nodes(j).coord == q_goal.coord          % Break if one of my nodes is the goal node
                break
            end
        end

        % Step 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        distance = [] ;
        for j = 1:length(nodes)
            distance = [distance vecnorm(nodes(j).coord - q_rand.coord)] ;
        end

        [mindist.val,mindist.idx] = min(distance) ;

        q_near = nodes(mindist.idx);

        if mindist.val > initR
            q_new.coord(1) = q_near.coord(1) + (q_rand.coord(1)-q_near.coord(1))*initR/vecnorm(q_rand.coord-q_near.coord) ;
            q_new.coord(2) = q_near.coord(2) + (q_rand.coord(2)-q_near.coord(2))*initR/vecnorm(q_rand.coord-q_near.coord) ;
        else
            q_new = q_rand ;
        end

        q_new.index = length(nodes)+1;

        [collision_true,pathSegments,~,q_new] = DubinsPathConnect(costmap, q_new, q_near, res);
        opt_pathsegment = pathSegments;

        % Step 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        q_new.cost = vecnorm(q_new.coord - q_near.coord) + q_near.cost ;

        % Step 5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        q_radius = [] ;
        r = initR ;                   % Search radius
        neighbor_idx = 1 ;

        for j = 1:length(nodes)
            if vecnorm(nodes(j).coord - q_new.coord) <= r
                q_radius(neighbor_idx).coord = nodes(j).coord;
                q_radius(neighbor_idx).theta = nodes(j).theta;
                q_radius(neighbor_idx).cost = nodes(j).cost;
                neighbor_idx = neighbor_idx + 1 ;
            end
        end

        % Step 6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        q_min = q_near ;
        C_min = q_new.cost ;

        for j = 1:length(q_radius)
            [collision,pathSegments,~,q_new_try] = DubinsPathConnect(costmap, q_new, q_radius(j), res);
            if collision == 0 && q_radius(j).cost + vecnorm(q_radius(j).coord - q_new.coord) < C_min
                q_min = q_radius(j) ;
                C_min = q_radius(j).cost + vecnorm(q_radius(j).coord - q_new.coord) ;
                opt_pathsegment = pathSegments;
                collision_true = 0;
                q_new = q_new_try;
            end
        q_new.cost = C_min;
        end
        if collision_true == 1
            continue
        end

        poses = interpolate(opt_pathsegment{1,1},0:res:opt_pathsegment{1,1}.Length);

        axis tight manual % this ensures that getframe() returns a consistent size
        figure(1)
        legend('off')
        hold on
        plot(poses(:,1),poses(:,2))
           create_gif('TreeMaking.gif', gif_frame, figure(1),0.005)
        gif_frame = gif_frame + 1;
        q_new.path = opt_pathsegment{1,1};

        for j = 1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;% For every node we now know its parent
            end
        end
        
        [~,~,pathCosts,~] = DubinsPathConnect(costmap, q_new, nodes(q_new.parent), res);
        q_new.dubinscost = pathCosts + nodes(q_new.parent).dubinscost; 
        
        % Step 7 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        nodes = [nodes q_new];                 % Append all information on new node to list
    end

    %% Step 8 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Backward iteration to find optimal path
    D = [];                     % Initialise D
    for j = 1:length(nodes)
        tmpdist = vecnorm(nodes(j).coord - q_goal.coord);
        D = [D tmpdist];
    end

    [val, idx] = min(D);
    q_final = nodes(idx);
    q_goal.parent = idx;
    q_end = q_goal ;
    q_goal.index = length(nodes) + 1;
    q_goal.path = cell(1);
    q_goal.dubinscost = val + nodes(idx).dubinscost;
    
    nodes = [nodes q_goal]; % Append goal node to list of nodes
    
    if val > initR
        display('Goal node is not reached');
    else
        display('Feasible path is found');
        axis tight manual % this ensures that getframe() returns a consistent size
        gif_frame = 1;

        figure(1)
        line([q_end.coord(1), nodes(idx).coord(1)], [q_end.coord(2), nodes(idx).coord(2)], 'Color', 'g', 'LineWidth', 2) ;
        start = q_goal.index;
        while q_end.parent ~= 1     % Inverse of “==”
            start = q_end.parent ;
            poses = interpolate(nodes(start).path,0:res:nodes(start).path.Length);

            figure(1)
            plot(poses(:,1),poses(:,2), 'Color', 'g',  'LineWidth', 2) ;
            drawnow
            hold on
            create_gif('OptimalPathFinder.gif', gif_frame, figure(1),0.2)
              gif_frame = gif_frame + 1;   
            q_end = nodes(start) ;
        end
    end
end
