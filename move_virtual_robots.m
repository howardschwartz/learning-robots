function [robot] = move_virtual_robots(robot, robot_x, robot_y, speed, no_of_robots, no_of_v_robots)
% This function moves the noise corrupted virtual robots
%   The input arguments are the robot data structure, robot and the number
%   of robots called as [robot] = move_virtual_robot(robot, no_of_robots)
%
   global u; % Heading angle
   global v; % speed
   for i = 1:no_of_robots
       v = robot(i).speed;
       for j = 1:no_of_v_robots
          y(1) = robot_x(i);
          y(2) = robot_y(i);
          u = robot(i).vheading(j);
          [T, yout] = ode45(robot(i).dynamics, [0 0.05 0.1], y);
          y = yout(3, :);
          robot(i).virtualx(j) = y(1);
          robot(i).virtualy(j) = y(2);
          robot(i).virtual_velx(j) = v*cos(u);
          robot(i).virtual_vely(j) = v*sin(u);
       end
   end       

end

