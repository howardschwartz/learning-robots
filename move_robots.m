function [robot] = move_robots(robot, no_of_robots)
%  move_robots(robot, no_of_robots)
%  uses ode45 to simulate the robtos based on their specific dynamics.
%
   global u; % Heading angle
   global v; % speed
   for i = 1, no_of_robots
       u = robot(i).heading;
       v = robot(i).speed;
       y(1) = robot(i).x;
       y(2) = robot(i).y;
       [T, yout] = ode45(robot(i).dynamics, [0 0.05 0.1], y);
       y = yout(3, :);
       robot(i).x = y(1);
       robot(i).y = y(2);
   end       
end

