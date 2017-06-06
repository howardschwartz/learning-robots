function [capture_heading, closing_dist, line_of_sight ] = compute_reward(robot, delup)
% [capture_heading, closing_dist, line_of_sight ] = compute_reward(robot)
% Compute the rewards for each robot
%
% First compute the rewards for the pursuers
%
global dist_max;
global no_of_robots;
   for i=1,no_of_robots
       for j=1,no_of_robots
           if (robot(i).type == 1 && robot(j).type == 0)
               capture_heading(i,j) = 2*exp(-(delup(i,j)^2)) - 1;
               closing_dist(i,j) = (rel_dist_old(i,j) - rel_dist(i,j))/dist_max
               line_of_sight(i,j) = 2*exp(-(dellos(i,j)^2)/0.05) - 1;
           end
       end
   end
end

