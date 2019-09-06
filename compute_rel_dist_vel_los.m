function [robot, rel_dist, rel_speed, los, relative_vel] = compute_rel_dist_vel_los( robot, no_of_robots)
%   [rel_dist, rel_vel, los] = copute_rel_dist_vel_los( robot )
%   Detailed explanation goes here
%
% Initialize the data
%
    rel_dist = zeros(no_of_robots, no_of_robots);
    rel_speed = zeros(no_of_robots, no_of_robots);
    los = zeros(no_of_robots, no_of_robots);
    relative_vel = zeros(no_of_robots, no_of_robots);
    for i=1:no_of_robots
       for j=1:no_of_robots
           if (j ~= i)
              xi = robot(i).x;
              yi = robot(i).y;
              xj = robot(j).x;
              yj = robot(j).y;
              velxi = robot(i).velx;
              velyi = robot(i).vely;
              velxj = robot(j).velx;
              velyj = robot(j).vely;
              robot(i).rel_pos(j).x = robot(j).x - robot(i).x;
              robot(i).rel_pos(j).y = robot(j).y - robot(i).y;
              robot(i).rel_vel(j).x = robot(j).velx - robot(i).velx;
              robot(i).rel_vel(j).y = robot(j).vely - robot(i).vely;
              robot(i).los(j).actual = atan2(robot(i).rel_pos(j).y, robot(i).rel_pos(j).x);
              rel_dist(i, j) = sqrt(robot(i).rel_pos(j).x^2 + robot(i).rel_pos(j).y^2);
              rel_speed(i, j) = sqrt(robot(i).rel_vel(j).x^2 + robot(i).rel_vel(j).y^2);
              los(i, j) = robot(i).los(j).actual;
              los2 = atan2(-robot(i).rel_pos(j).y, -robot(i).rel_pos(j).x);
              velx = robot(i).rel_vel(j).x;
              vely = robot(i).rel_vel(j).y;
              unitx = cos(los2);
              unity = sin(los2);
              relative_vel(i, j) = velx*unitx + vely*unity;
           end
       end
    end
end

