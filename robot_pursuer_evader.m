function [robot, no_of_robots] = robot_pursuer_evader()
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
   % Read in the robot data
   %
   fid = fopen('robot.txt');
   no_of_data = [19, inf];
   robot_data = fscanf(fid, '%f', no_of_data);
   robot_data = robot_data';
   [m, n] = size(robot_data);
   no_of_robots = m;
   for i = 1:no_of_robots
       robot_init(i).type = robot_data(i, 1);
       robot_init(i).x = robot_data(i, 2);
       robot_init(i).y = robot_data(i, 3);
       robot_init(i).speed = robot_data(i, 4);
       robot_init(i).heading = robot_data(i, 5);
       robot_init(i).critic.no_of_inputs = robot_data(i, 6);
       no_of_inputs = robot_data(i, 6);
       k = 6;
       for j = 1:no_of_inputs
          robot_init(i).critic.mf_per_input(j).no_of_mf = robot_data(i, k+1);
          robot_init(i).critic.mf_per_input(j).range(1,1) = robot_data(i, k+2);
          robot_init(i).critic.mf_per_input(j).range(1,2) = robot_data(i, k+3);
          k = k + 3;
       end
       robot_init(i).actor.no_of_inputs = robot_data(i, k+1);
       no_of_inputs = robot_data(i, k+1);
       k = k+1;
       for j = 1:no_of_inputs
          robot_init(i).actor.mf_per_input(j).no_of_mf = robot_data(i, k+1);
          robot_init(i).actor.mf_per_input(j).range(1,1) = robot_data(i, k+2);
          robot_init(i).actor.mf_per_input(j).range(1,2) = robot_data(i, k+3);
          k = k + 3;
       end
   end 
   % Initialize the robot structure
   %
   [robot] = init_robots(robot_init, no_of_robots)
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
   for i = 1:no_of_robots
       for j = 1:no_of_robots
           [condition, alpha, up_des, delup] = capture_condition(robot(i), robot(j));
           robot(i).capture(j).condition = condition;
           robot(i).capture(j).alpha = alpha;
           robot(i).capture(j).des_heading = up_des;
           robot(i).capture(j).del_heading = delup;
       end
   end
   
end

