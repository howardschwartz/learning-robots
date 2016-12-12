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
   %
   % Initialize parameters associated with success
   %
  % psil2_success = psil2;
   %wl2_success = wl2;
  % psil2_success_init = psil2;
  % wl2_success_init = wl2;
  % psil2_init = psil2;
  % wl2_init = wl2;
   %alpha_success = alpha;
  % beta_success = beta;
  % sigma_success = sigma;
  % capture_cond2_old = 0;
  %
  game_no = 1;
  count_success = 150;
  sigma_plot = zeros(1,game_no);
  sigma_success_plot = zeros(1, game_no);
  %
  % % start here %
  % ***************************************************************
  for j=1:game_no
    %
    % Initialize pursuer and evader positions velocity and heading
    %
    for i=1:no_of_robots
      robot(i).x =  robot_init(i).x;
      robot(i).y =  robot_init(i).y;
      robot(i).speed =  robot_init(i).speed; % Start from not moving
      robot(i).heading = robot_init(i).heading;   % start with zero heading
    end
    %
    % Initialize some conditions
    %
    %capture_fail = 0;
    %psil2_init = psil2;
    %wl2_init = wl2;
    %sigma_plot(j) = sigma;
    %sigma_success_plot(j) = sigma_success;
    game_on = 1; %start the game
    dt = 0.1; % sampling time in seconds
   % *******************************************************************
   % 3rd change
   %
   % Initialize the figure that we make use of to plot the trajectories
   % of the agents.
   % *******************************************************************
   close all % Close all open figures
   gamePlot = figure('visible','off'); % Create new figure but don't display it
   axis([-10 25 -10 25]) % set the axis of the figure
   hold on % ensure continuos plot on the same figure
   grid on % turn on the grid lines
   % *******************************************************************
      while(game_on == 1)
          [robot, rel_dist, rel_speed, los] = compute_rel_dist_vel_los(robot, no_of_robots, dt )
          for i=1:no_of_robots
             for k=1:no_of_robots
                if (robot(i).type == 1 && robot(k).type == 2)
                    inputs = [robot(i).rel_pos(k).x, robot(i).rel_pos(k).y];
                    [action] = compute_robot_action( robot(i), inputs );
                    [value, phi_norm] = compute_robot_state_value(robot(i), inputs);
                    robot(i).value = value
                    robot(i).heading = action;
                end
             end
          end
          %
          % Remeber the state before we take an action
          %
          robot_old = robot;
          %
          % Lets move the robots one step
          %
          [robot] = move_robots(robot, no_of_robots);
          %
          % Recompute the relative distances and the new value
          %
          [robot, rel_dist, rel_speed, los] = compute_rel_dist_vel_los(robot, no_of_robots, dt )
          for i=1:no_of_robots
             for k=1:no_of_robots
                if (robot(i).type == 1 && robot(k).type == 2)
                    inputs = [robot(i).rel_pos(k).x, robot(i).rel_pos(k).y];
                    [value, phi_norm] = compute_robot_state_value(robot(i), inputs);
                    robot(i).value = value
                end
             end
          end
          game_on = 0;
      end %% ****  END the While Loop of Epoch ****%
  end
   
end

