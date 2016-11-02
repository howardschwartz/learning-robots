function [output_data] = robot_pursuer_evader(robot_init)
   % Simulation of the multi-Robot pursuer evader machine learning game
   % Started by Prof. Schwartz Oct. 30, 2016
   % Define globsl variables 
   global up1; % Heading angle pursuer 1
   global up2; % Heading angle pursuer 2
   global up3; % Heading angle of pursuer 3
   global ue;  % heading angle evader
   global vp1; % pursuer 1 speed
   global vp2; % pursuer 2 speed
   global vp3; % pursuer 3 speed
   global ve;  % evader speed
   %
   % Initialize the robot structure
   %
   [robot, no_of_robots] = init_robots(robot_init);
   %
   % Initialize the counters
   %
   count = 0;
   count2= 0;
   count3 = 0;
   countm = 0;
   count_success_times = 0;
%
% Compute the capture condition between each robot
%
   for i=1:no_of_robots
       for j = 1:no_of_robots
           [condition, alpha, up_des, delup] = capture_condition(robot(i), robot(j));
           robot(i).capture(j).condition = condition;
           robot(i).capture(j).alpha = alpha;
           robot(i).capture(j).des_heading = up_des;
           robot(i).capture(j).del_heading = delup;
       end
   end
   
end

