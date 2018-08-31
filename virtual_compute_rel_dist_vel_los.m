function [robot, v_rel_dist, v_rel_speed, v_los] = virtual_compute_rel_dist_vel_los(robot, no_of_robots, no_of_v_robots)
%   [robot, v_rel_dist, v_rel_speed, v_los] = compute_rel_dist_vel_los(robot, no_of_robots)
% The relative distance, speed and Line of Sight between each virtual robot
% and the other robots.
%
% Initialize the data
%
    v_rel_dist = zeros(no_of_robots, no_of_robots, no_of_v_robots);
    v_rel_speed = zeros(no_of_robots, no_of_robots, no_of_v_robots);
    v_los = zeros(no_of_robots, no_of_robots, no_of_v_robots);
    for i=1:no_of_robots
       for j=1:no_of_robots
           if (j ~= i)
              for k=1:no_of_v_robots
                 robot(i).rel_pos(j).virtualx(k) = robot(j).virtualx(k) - robot(i).x;
                 robot(i).rel_pos(j).virtualy(k) = robot(j).virtualy(k) - robot(i).y;
                 robot(i).rel_vel(j).virtualx(k) = robot(j).virtual_velx(k) - robot(i).velx;
                 robot(i).rel_vel(j).virtualy(k) = robot(j).virtual_vely(k) - robot(i).vely;
                 robot(i).los(j).virtual(k) = atan2(robot(i).rel_pos(j).virtualy(k), robot(i).rel_pos(j).virtualx(k));
                 v_rel_dist(i, j, k) = sqrt(robot(i).rel_pos(j).virtualx(k)^2 + robot(i).rel_pos(j).virtualy(k)^2);
                 v_rel_speed(i, j, k) = sqrt(robot(i).rel_vel(j).virtualx(k)^2 + robot(i).rel_vel(j).virtualy(k)^2);
                 virtual_rel_speed = v_rel_speed(i, j, k);
                 speedx = robot(i).velx;
                 speedy = robot(i).vely;
                 virtualspeedx = robot(j).virtual_velx(k);
                 virtualspeedy = robot(j).virtual_vely(k);
                 v_los(i, j, k) = robot(i).los(j).virtual(k);
              end
           end
       end
    end
end


