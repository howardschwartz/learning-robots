function [robot, v_rel_dist, v_rel_speed, v_los, v_relative_vel] = virtual_compute_rel_dist_vel_los(robot, no_of_robots, no_of_v_robots)
%   [robot, v_rel_dist, v_rel_speed, v_los] = compute_rel_dist_vel_los(robot, no_of_robots)
% The relative distance, speed and Line of Sight between each virtual robot
% and the other robots.
%
% Initialize the data
%
    v_rel_dist = zeros(no_of_robots, no_of_robots, no_of_v_robots);
    v_rel_speed = zeros(no_of_robots, no_of_robots, no_of_v_robots);
    v_los = zeros(no_of_robots, no_of_robots, no_of_v_robots);
    v_relative_vel = zeros(no_of_robots, no_of_robots);
    for i=1:no_of_robots
       for j=1:no_of_robots
           if (j ~= i)
              for k=1:no_of_v_robots
                 xi = robot(i).x;
                 yi = robot(i).y;
                 xj = robot(j).x;
                 yj = robot(j).y;
                 velxi = robot(i).velx;
                 velyi = robot(i).vely;
                 velxj = robot(j).velx;
                 velyj = robot(j).vely;
                 robot(i).rel_pos(j).virtualx(k) = robot(j).virtualx(k) - robot(i).x;
                 robot(i).rel_pos(j).virtualy(k) = robot(j).virtualy(k) - robot(i).y;
                 robot(i).rel_vel(j).virtualx(k) = robot(j).virtual_velx(k) - robot(i).velx;
                 robot(i).rel_vel(j).virtualy(k) = robot(j).virtual_vely(k) - robot(i).vely;
                 robot(i).los(j).virtual(k) = atan2(robot(i).rel_pos(j).virtualy(k), robot(i).rel_pos(j).virtualx(k));
                 los2 = atan2(-robot(i).rel_pos(j).virtualy(k), -robot(i).rel_pos(j).virtualx(k));
                 v_rel_dist(i, j, k) = sqrt(robot(i).rel_pos(j).virtualx(k)^2 + robot(i).rel_pos(j).virtualy(k)^2);
                 v_rel_speed(i, j, k) = sqrt(robot(i).rel_vel(j).virtualx(k)^2 + robot(i).rel_vel(j).virtualy(k)^2);
                 v_los(i, j, k) = robot(i).los(j).virtual(k);
                 velx = robot(i).rel_vel(j).virtualx(k);
                 vely = robot(i).rel_vel(j).virtualy(k);
                 unitx = cos(los2);
                 unity = sin(los2);
                 v_relative_vel(i, j) = velx*unitx + vely*unity;
              end
           end
       end
    end
end


