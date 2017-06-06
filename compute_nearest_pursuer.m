function [dist_min, evader, pursuer] = compute_nearest_pursuer(robot, no_of_robots)
% [dist_min, k_min] = compute_nearest_pursuer(robot, no_of_robots)
%  This function determines the closest pursuer to the evader. It is
% called with the robot objects for thr pursuer and the evader and returns
% the minimum distance betwee and evader and a pursuer
%
    dist_min = 100;
    for i=1:no_of_robots
         for k=1:no_of_robots
             if (robot(i).type == 1 && robot(k).type == 2)
                if (robot(i).capture(k).dist < dist_min)
                    dist_min = robot(i).capture(k).dist;
                    evader = k;
                    pursuer = i;
                end
             end
         end
     end
end

