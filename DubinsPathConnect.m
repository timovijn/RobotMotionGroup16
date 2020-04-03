function [collision,pathSegments,pathCost,q_new] = DubinsPathConnect(costmap, q_new, q_near, res)

if costmap.checkOccupied([q_new.coord(1),q_new.coord(2)]) == true
    collision = 1;
    pathSegments = NaN;
    pathCost = NaN;
    q_new.theta = NaN;
else    
    x = q_new.coord(1) - q_near.coord(1);
    y = q_new.coord(2) - q_near.coord(2);
    %theta_random = wrapToPi(atan2(x,y));
    theta_random = atan2(y,x);
    
    if abs(theta_random) > (1/6)*pi 
        theta_random = sign(theta_random)*(1/6)*pi;
    end
    %theta_random = normalizeAngle(atan2(x,y));
    
    q_new.theta = theta_random;
    
    minR = 10.4*res;    
    dubConnObj = dubinsConnection('MinTurningRadius', minR);
    
    [pathSegments,pathCost] = connect(dubConnObj,[q_near.coord q_near.theta],[q_new.coord q_new.theta]);
    poses = interpolate(pathSegments{1,1},0:res:pathSegments{1,1}.Length);

    if any(poses(:,1)>250) || any(poses(:,2)>18) || pathCost > 2*vecnorm(q_new.coord - q_near.coord)
        collision = 1;
        return
    end
    if any(costmap.checkOccupied([poses(:,1),poses(:,2)])) == true
        collision = 1;
    else
        collision = 0;
    end
end

end