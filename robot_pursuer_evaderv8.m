function [robot, no_of_robots, movie_frame] = robot_pursuer_evaderv8()
   % Simulation of the multi-Robot pursuer evader machine learning game
   % Started by Prof. Schwartz Oct. 30, 2016. Adding in the virtual robots.
   %
   % Read in the robot data
   %
   %fid = fopen('robot.txt');
   fid = fopen('robotv2.txt');
   no_of_data = [25, inf];
%    fid = fopen('robot11v2.txt');
%    no_of_data = [67, inf];
   robot_data = fscanf(fid, '%f', no_of_data);
   robot_data = robot_data';
   [m, n] = size(robot_data);
   no_of_robots = m;
   no_of_v_robots = 1; % set the number of virtual robots
   no_update = zeros(1, no_of_v_robots);
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
   [robot] = init_robotsv6(robot_init, no_of_robots, no_of_v_robots);
   %[robot] = init_robots(robot_init, no_of_robots, no_of_v_robots);
   %
   % Reset the membership functions and inputs for the pursuers.
   %
   for i = 1:no_of_robots
       if(robot(i).type == 2)
           robot(i).heading = 0; % reset the initial evader heading to zero.
           robot_init(i).actor.no_of_inputs = 1;
           robot_init(i).actor.mf_per_input(1).no_of_mf = 10;
           robot_init(i).actor.mf_per_input(1).range(1,1) = -pi;
           robot_init(i).actor.mf_per_input(1).range(1,2) = pi;
           robot_init(i).critic.no_of_inputs = 1;
           robot_init(i).critic.mf_per_input(1).no_of_mf = 10;
           robot_init(i).critic.mf_per_input(1).range(1,1) = -pi;
           robot_init(i).critic.mf_per_input(1).range(1,2) = pi;
           for j=1:robot_init(i).critic.no_of_inputs
              robot(i).critic.input(j).range_min = robot_init(i).critic.mf_per_input(j).range(1,1);
              robot(i).critic.input(j).range_max = robot_init(i).critic.mf_per_input(j).range(1,2);
           end
      %
           for j=1:robot_init(i).actor.no_of_inputs
              robot(i).actor.input(j).range_min = robot_init(i).critic.mf_per_input(j).range(1,1);
              robot(i).actor.input(j).range_max = robot_init(i).critic.mf_per_input(j).range(1,2);
           end
           [rule_critic, no_of_rules_critic] = init_mf_rules_robot(robot_init(i).critic);
           [rule_actor, no_of_rules_actor] = init_mf_rules_robot(robot_init(i).actor);
           robot(i).target.no_of_rules_critic = no_of_rules_critic; % number of rules
           robot(i).target.rule_critic = rule_critic; 
           robot(i).target.psi = zeros(1, no_of_rules_critic); % initialze critic Parameters
           robot(i).target.phi_norm_critic = zeros(1, no_of_rules_critic);
           robot(i).target.no_of_rules_actor = no_of_rules_actor; % number of rules
           robot(i).target.rule_actor = rule_actor; 
           robot(i).target.w = zeros(1, no_of_rules_actor); % initialze actor parameters
           robot(i).target.phi_norm_actor = zeros(1, no_of_rules_actor);
           robot(i).target.heading = 0;
           robot(i).target.des_heading = robot_init(i).heading; % initialize the robot's desired heading
           robot(i).target.condition = 1; % Can always capture the target
           robot(i).target.no_capture_psi = zeros(1, no_of_rules_critic);
           robot(i).target.no_capture_w = zeros(1, no_of_rules_actor);
       end
   end
   %
   % Initialize the counters
   %
   count = 0;
   count2 = 0;
   x = zeros(1, no_of_robots);
   y = zeros(1, no_of_robots);
   speed = zeros(1, no_of_robots);
   count3 = 0;
   game_no = 1000;
   movie_stop = 0;
   count_max = 150;
   td_count = 1;
   v_reward_capture_max = -100;
   v_reward_no_capture_max = -100;
   max_rules_fired = 2^(no_of_inputs);
   %
   % Compute the capture condition between each robot
   %
   for i = 1:no_of_robots
       for k = 1:no_of_robots
           [condition, up_des, delup] = capture_condition(robot(i), robot(k));
           robot(i).capture(k).condition = condition; %can pursuer i capture evader k?
           robot(i).capture(k).des_heading = up_des;
           robot(i).capture(k).del_heading = delup;
       end
   end
  %
  % % start here %
  % ***************************************************************
  for j=1:game_no % This is the loop for each epoch
    sprintf(' The game number is %d ', j)
    if(movie_stop == 0)
        clearvars movie_frame;
    end
%     if(j == 500)
%         robot(1).speed = 1.6;
%     end
    %
    % Initialize pursuer and evader positions velocity and heading
    %
    for i=1:no_of_robots
      robot(i).x =  robot_init(i).x;
      robot(i).y =  robot_init(i).y;
      robot(i).speed =  robot_init(i).speed; % Start from not moving
      %robot(i).heading = robot_init(i).heading;   % start with zero heading
      %Start here to init capture conditions psi_init and w_init
      for k=1:no_of_robots
           if (robot(i).type == 1 && robot(k).type == 2)
               %robot(i).capture(k).psi_init = robot(i).psi;
               robot(i).capture(k).psi_init = robot(i).capture(k).psi;
               %robot(i).capture(k).w_init = robot(i).w;
               robot(i).capture(k).w_init = robot(i).capture(k).w;
               robot(i).capture(k).condition_change_to_fail = 0;
           end
      end   
    end
    %
    % Initialize the capture conditions
    %
    for i = 1:no_of_robots
       for k = 1:no_of_robots
          if (robot(i).type == 1 && robot(k).type == 2)
             [condition, up_des, delup] = capture_condition(robot(i), robot(k));
             robot(i).capture(k).condition = condition;
             robot(i).capture(k).des_heading = up_des;
             robot(i).capture(k).del_heading = delup;
          end
        end
    end
    %
    % Initialize the target
    %
    for i=1:no_of_robots
           if (robot(i).type == 2)
               %robot(i).capture(k).psi_init = robot(i).psi;
               robot(i).target.psi_init = robot(i).target.psi;
               %robot(i).capture(k).w_init = robot(i).w;
               robot(i).target.w_init = robot(i).target.w;
               robot(i).target.des_heading = robot_init(i).heading;
               target_des_heading = robot(i).target.des_heading;
           end
      end 
    %
    % Initialize some conditions
    %
    rule_capture_condition_change = 0;
    count = 0;
    count2 = 0;
    game_on = 1; %start the game
    dt = 0.1; % sampling time in seconds
    %
    % Initialize the figure that we make use of to plot the trajectories
    %close all % Close all open figures
    gamePlot = figure('visible','off'); % Create new figure but don't display it
    axis([-10 20 -10 20]) % set the axis of the figure
    hold on % ensure continuous plot on the same figure
    grid on % turn on the grid lines
    % *******************************************************************
    % *******************************************************************
    while(game_on == 1)
          count = count + 1;
          rule_set_number_new = 0;
          %sprintf(' The count is %d the game number is %d ', count, j)
          [robot, rel_dist, rel_speed, los] = compute_rel_dist_vel_los(robot, no_of_robots);
          %
          % First compute the actions of the pursuers
          %
          for i = 1:no_of_robots
              robot(i).no_update = 0;
          end
          no_update = 0;
          rule_capture_condition_change = 0;
          for i=1:no_of_robots
             for k=1:no_of_robots
                if (robot(i).type == 1 && robot(k).type == 2)
                    robot(i).capture(k).condition_change_to_fail = 0;
                    inputs = [robot(i).rel_pos(k).x, robot(i).rel_pos(k).y, robot(k).heading];
                    %inputs = [robot(1).rel_pos(k).x, robot(1).rel_pos(k).y, robot(1).heading, robot(2).rel_pos(k).x, robot(2).rel_pos(k).y, robot(2).heading, robot(3).rel_pos(k).x, robot(3).rel_pos(k).y, robot(3).heading, robot(k).heading];
                    if(inputs(1) > 10 || inputs(1) < -10 || inputs(2) > 10 || inputs(2) < -10 || inputs(3) > pi || inputs(3) < - pi)
                         %sprintf(' The input is out of range, the pursuer is %d ', i)
                         %inputs
                         robot(i).no_update = 1;
                    end
                    %[value, phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_robot_state_value(robot(i).capture(k), inputs, robot(i).critic.input);
                    %[phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_rules_fired_set(robot(i).capture(k), inputs, robot(i).critic.input);
                    [phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_rules_fired_setv81(robot(i).capture(k), inputs, robot(i).critic.input, max_rules_fired);
                    robot(i).capture(k).phi_norm_critic = phi_norm;
                    robot(i).capture(k).phi_norm_actor = phi_norm;
                    %
                    % Count how many times a rule has fired
                    %
                    [rule_set_number, number, condition_old, rule_found] = search_for_rule_set(robot(i).capture(k), rules_fired);
                    if (rule_found == 1)
                       robot(i).capture(k).rules_fired(rule_set_number).number = number;
                       robot(i).capture(k).rules_fired(rule_set_number).condition_old = condition_old;
                       robot(i).capture(k).rules_fired(rule_set_number).phi_norm = phi_norm;
                    end
                    %
                    % If this a new rule set then initialize it
                    %
                    count3 = robot(i).capture(k).number_of_rules_fired;
                    if (rule_found == 0)
                          count3 = count3 + 1;
                          robot(i).capture(k).rules_fired(count3).rules_fired = rules_fired;
                          robot(i).capture(k).rules_fired(count3).number = 1;
                          robot(i).capture(k).rules_fired(count3).phi_norm = phi_norm;
                          robot(i).capture(k).rules_fired(count3).reward_max_cap = -100;
                          robot(i).capture(k).rules_fired(count3).reward_max_cap_old = -100;
                          robot(i).capture(k).rules_fired(count3).reward_max_no_cap = -100;
                          robot(i).capture(k).rules_fired(count3).reward_max_no_cap_old = -100;
                          robot(i).capture(k).rules_fired(count3).condition = robot(i).capture(k).condition;
                          robot(i).capture(k).rules_fired(count3).condition_old = robot(i).capture(k).condition;
                          robot(i).capture(k).rules_fired(count3).td = 100;
                          robot(i).capture(k).rules_fired(count3).td_avg = 0;
                          robot(i).capture(k).rules_fired(count3).td_sigma = 0;
                          robot(i).capture(k).rules_fired(count3).td_actor = 100;
                          robot(i).capture(k).rules_fired(count3).td_avg_actor = 0;
                          robot(i).capture(k).rules_fired(count3).td_sigma_actor = 0;
                          robot(i).capture(k).rules_fired(count3).td_converged = 0;
                          robot(i).capture(k).rules_fired(count3).parameters_capture = zeros(1, 10);
                          robot(i).capture(k).rules_fired(count3).parameters_no_capture = zeros(1, 10);
                          robot(i).capture(k).rules_fired(count3).psi_parameters_capture = zeros(1, 10);
                          robot(i).capture(k).rules_fired(count3).psi_parameters_no_capture = zeros(1, 10);
                          robot(i).capture(k).rules_fired(count3).zed = 0;
                          rule_set_number = count3;
                          robot(i).capture(k).number_of_rules_fired = count3;
                    end
                    %[value] = compute_robot_state_valuev8(robot(i).capture(k), phi_norm, rule_fire_count, rules_fired);
                    [value] = compute_robot_state_valuev81(robot(i).capture(k), rule_fire_count, rule_set_number);
                    robot(i).capture(k).value_old = value;
                    robot(i).capture(k).rules_fired(rule_set_number).value = value;
                    robot(i).capture(k).current_rules_fired = rules_fired;
                    robot(i).capture(k).rule_fire_count = rule_fire_count;
                    robot(i).capture(k).dist = rel_dist(i, k);
                    robot(i).capture(k).condition_change_to_fail = 0;
                    [robot(i).capture(k), action] = compute_robot_actionv8(robot(i).capture(k), rule_fire_count, rule_set_number);
                    robot(i).capture(k).heading = action;
                    robot(i).capture(k).rule_set_number = rule_set_number;
                    robot(i).heading = action;
                     %
                    for m = 1:no_of_v_robots
                       robot(i).capture(k).virtual(m).phi_norm_critic = phi_norm;
                       robot(i).capture(k).virtual(m).phi_norm_actor = phi_norm;
                       robot(i).capture(k).v_value(m) = value; 
                    end
                    if (robot(i).capture(k).condition == 1)
                        robot(i).capture(k).capture_not_zero_phi = not_zero_phi;
                    end
                    if (robot(i).capture(k).condition == 0)
                       robot(i).capture(k).no_capture_not_zero_phi = not_zero_phi;
                    end
                    %
                end
             end
          end
          %
          % Compute the nearest pursuer to the evader that can capture
          %
          [dist_min, evader, nearest] = compute_nearest_pursuer(robot, no_of_robots);
          %
          % If the nearest pursuer robot is close then run away
          %
          % To Do run away from nearest and nearest that can capture.
          if (dist_min < 1.0)
             %run away from nearest robot
             % robot(evader).heading = los(nearest, evader);
              robot(evader).des_heading = los(nearest, evader);
              robot(evader).target.des_heading = robot(evader).des_heading;
              sprintf(' Run away from nearest %d robot at dist_min %f', nearest, dist_min)
              sprintf(' The target desired heading is, %f', robot(evader).target.des_heading)
              if (nearest == 4)
                  sprintf(' The target desired heading is, %f', robot(evader).target.des_heading)
              end
          end
          %
          % Compute heading to target
          %
          for i=1:no_of_robots
              if(robot(i).type == 2)
               % sprintf(' The target desired heading is, %f', robot(i).target.des_heading)
               % sprintf(' The robot heading is, %f', robot(i).heading)
                robot(i).heading_error = robot(i).target.des_heading - robot(i).heading;
                inputs = robot(i).heading_error;
                [action] = compute_evader_action(robot(i).target, inputs, robot(i).actor.input);
                [value, phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_robot_state_value(robot(i).target, inputs, robot(i).critic.input );
              %  sprintf(' The computed action is, %f', action)
                robot(i).target.value_old = value;
                robot(i).target.heading = robot(i).target.heading + action;
                heading = robot(i).heading + action;
                robot(i).heading = heading;
                robot(i).vheading = heading;
                if(robot(i).heading > pi)
                    robot(i).heading = pi;
                    %sprintf(' Robot %d has saturated at pi ', i)
                    robot(i).no_update = 1;
                end
                if(robot(i).heading < -pi)
                    robot(i).heading = -pi;
                    %sprintf(' Robot %d has saturated at -pi ', i)
                    robot(i).no_update = 1;
                end
                robot(i).target.phi_norm_critic = phi_norm;
                robot(i).target.phi_norm_actor = phi_norm;
              end
          end
          %
          % Compute the exploration noise for each robot
          %
          for i=1:no_of_robots
              if (robot(i).type == 1) % pursuers learning
                  %
                  % We want to try and implement particle
                  % excitation
                  %
                  %robot(i).noise = normrnd(0,robot(i).sigma);
                  vsigma = robot(i).vsigma;
                  robot(i).vnoise = normrnd(0, robot(i).vsigma, 1, no_of_v_robots);
                  %robot(i).vnoise = robot(i).noise*ones(1, no_of_v_robots);
                  virtual_noise = robot(i).vnoise;
                  robot(i).vheading = robot(i).heading*ones(1, no_of_v_robots) + robot(i).vnoise;
                  %robot(i).no_update = zeros(1, no_of_v_robots);
                  for m=1:no_of_v_robots
                      if (robot(i).vheading(m) > pi || robot(i).vheading(m) < -pi)
                          robot(i).no_update = 1;
%                           sprintf(' DEBUG hms02162018 no_update robot(%d) virtual robot %d ', i, m)
%                           sprintf(' DEBUG hms02162018 virtual heading %f', robot(i).vheading(m))
%                           sprintf(' DEBUG hms09252018 virtual noise %f', robot(i).vnoise(m))
%                           sprintf(' DEBUG hms09252018 actual heading %f', robot(i).heading)
                      end
                  end
                  
                  robot(i).noise = 0;
                  %robot(i).noise = normrnd(0,robot(i).sigma);
                  robot(i).heading = robot(i).heading + robot(i).noise;
                  if(robot(i).heading > pi)
                    robot(i).heading = pi;
                   % sprintf(' Robot %d has saturated at pi ', i)
                    robot(i).no_update = 1;
                  end
                  if(robot(i).heading < -pi)
                    robot(i).heading = -pi;
                    %sprintf(' Robot %d has saturated at -pi ', i)
                    robot(i).no_update = 1;
                  end
                  if(robot(i).type == 2)
                     %sprintf(' The robot heading is, %f', robot(i).heading)
                    % sprintf(' The noise is for robot(%d), %f', i, robot(i).noise)
                  end
              end
              if (robot(i).type == 2) % evader learning
                  if(robot(i).heading > pi)
                    robot(i).heading = pi;
                    %sprintf(' Robot %d has saturated at pi ', i)
                  end
                  if(robot(i).heading < -pi)
                    robot(i).heading = -pi;
                   %sprintf(' Robot %d has saturated at -pi ', i) 
                  end
                 % sprintf(' The robot heading is, %f', robot(i).heading)
              end
          end
          %
          % Lets move the robots one step
          %***********************************************************
          for i = 1:no_of_robots
              x(i) = robot(i).x;
              y(i) = robot(i).y;
          end
          %
          [robot] = move_robots(robot, no_of_robots);
          [robot] = move_virtual_robots(robot, x, y, speed, no_of_robots, no_of_v_robots);
          %***********************************************************
          %
          % Recompute the relative distances and the new value
          %
          [robot, rel_dist, rel_speed, los] = compute_rel_dist_vel_los(robot, no_of_robots);
          [robot, v_rel_dist, v_rel_speed, v_los] = virtual_compute_rel_dist_vel_los(robot, no_of_robots, no_of_v_robots);
          %
          % Recompute the capture conditions
          %
          for i = 1:no_of_robots
             for k = 1:no_of_robots
                if (robot(i).type == 1 && robot(k).type == 2)
                    v_condition_final = -1;
                   [condition, up_des, delup] = capture_condition(robot(i), robot(k));
                   [v_condition, v_up_des, v_delup] = capture_condition_virtual(robot(i), robot(k), no_of_v_robots);
                   %
                   % If the condition has changed for the virtual robot do
                   % not update.
                   %
                   for m1 = 1:no_of_v_robots
                       if (v_condition(m1) ~= robot(i).capture(k).condition || v_condition(m1) ~= condition)
                           %sprintf(' The pursuer %d and evader %d, the count is %d and the epoch is %d, the v_condition is not the same ', i, k, count, j)
                           robot(i).no_update = 1;
                           robot(i).capture(k).no_update = 1;
                           no_update = 1;
                       end
                   end
                   if (max(v_condition) == min(v_condition))
                       v_condition_final = max(v_condition);
                   end
                   if(v_condition_final == -1)
                      % sprintf(' The pursuer %d,  evader %d, the count is %d and the epoch is %d v_condition_final = -1 ', i, k, count, j)
                       no_update = 1;
                       robot(i).no_update = 1;
                   end
                   %
                   % Check if the capture condition changed
                   if (robot(i).capture(k).condition == 0 && condition == 1)
                       robot(i).capture(k).condition_change_to_fail = 1;
                       sprintf(' The pursuer %d can now capture evader %d, the count is %d and the epoch is %d ', i, k, count, j)
                       robot(i).no_update = 1;
                       no_update = 1;
                   end
                   %
                   if (robot(i).capture(k).condition == 1 && condition == 0)
                       sprintf(' The pursuer %d can no longer capture evader %d, the count is %d and the epoch is %d ', i, k, count, j)
                       robot(i).capture(k).condition_change_to_fail = 1;
                       robot(i).no_update = 1;
                       no_update = 1;
                       robot(i).capture(k).condition = 0;
                       [psi, w, sigma] = change_capture_condition(robot(i).capture(k));
                       robot(i).capture(k).psi = psi;
                       robot(i).capture(k).w = w;
                       robot(i).capture(k).sigma = sigma;
                       robot(i).sigma = sigma;
                      % game_on = 0; % End the game
                   end
                   robot(i).capture(k).condition = condition;
                   robot(i).capture(k).des_heading = up_des;
                   robot(i).capture(k).del_heading = delup;
                   reward_capture_heading = 2*exp(-(delup^2)) - 1;
                   robot(i).capture(k).reward_capture_heading = 2*exp(-(delup^2)) - 1;
                   robot(i).capture(k).reward_rel_vel = -rel_speed(i,k);
                   robot(i).capture(k).reward_heading_difference = 2*exp(-(delup^2)) - 1;
                   v_reward_capture_heading = zeros(1, no_of_v_robots);
                   for m = 1:no_of_v_robots
                      v_reward_capture_heading(m) = 2*exp(-(v_delup(m))^2) - 1;
                      robot(i).capture(k).v_reward_capture_heading(m) = 2*exp(-(v_delup(m))^2) - 1;
                      reward_relative_speed(m) = -v_rel_speed(i,k,m);
                      robot(i).capture(k).v_reward_rel_vel(m) = -v_rel_speed(k,i,m);
                      %
                      % Compute reward as difference in heading between
                      % pursuer and evader
                      %
                      heading_difference = robot(k).heading - robot(i).vheading(m);
                      robot(i).capture(k).v_reward_heading_difference(m) = 2*exp(-(heading_difference)^2) - 1;
                      v_reward_heading_difference(m) = 2*exp(-(heading_difference)^2) - 1;
                      %
                   end
                   %
                   % I need to find the maximum reward for the virtual
                   % robots
                   %
                   if (robot(i).capture(k).condition == 1 && v_condition_final == 1)
                       v_reward_capture_max = max(v_reward_capture_heading);
                   end
                   if (robot(i).capture(k).condition == 0 && v_condition_final == 0)
                       v_reward_no_capture_max = max(reward_relative_speed);
                   end
                   robot(i).capture(k).v_reward_capture_max = v_reward_capture_max;
                   robot(i).capture(k).v_reward_no_capture_max = v_reward_no_capture_max;
                   %
                   %  How do I use this??
                   %
                   if (robot(i).capture(k).condition == 1 && v_condition_final == 1)
                       robot(i).capture(k).current_reward = robot(i).capture(k).reward_capture_heading;
                       robot(i).capture(k).current_reward_virtual = v_reward_capture_max;
                       robot(i).capture(k).current_reward_max_cap = max([v_reward_capture_max, robot(i).capture(k).reward_capture_heading]);
                   end
                   if (robot(i).capture(k).condition == 0 && v_condition_final == 0)
                       robot(i).capture(k).current_reward = robot(i).capture(k).reward_rel_vel;
                       robot(i).capture(k).current_reward_virtual = v_reward_no_capture_max;
                       robot(i).capture(k).current_reward_max_no_cap = max([v_reward_no_capture_max, robot(i).capture(k).reward_rel_vel]);
%                        if(robot(i).capture(k).current_reward_max_no_cap > -0.4)
%                           sprintf(' The no_cap reward is bigger than -0.4, %d ', robot(i).capture(k).current_reward_max_no_cap)
%                        end
                   end
                   %
                   %  Compute reward for capture
                   %
                   %sprintf(' The relative position is x(%d, %d) = %f and y = %f', i, k, robot(i).rel_pos(k).x, robot(i).rel_pos(k).y)
                  % dist = sqrt((robot(i).rel_pos(k).x)^2 + (robot(i).rel_pos(k).y)^2);
%                    if(dist < 0.5) % We have successfully captured.
%                       sprintf(' Pursuer %d has captured the evader. The distance is %f and count is %d and the epoch is %d', i, dist, count, j)
%                       robot(i).capture(k).current_reward = 100.0;
%                    end
                   %
                   % End compute reward for capture
                   % 
                   %sprintf(' The reward is %f ', robot(i).reward_capture_heading)
                   %
                   % What is the maximum reward for this set of rules?
                   % Has this rule set fired previously and what is the
                   % maximum reward?
                   %
                   count3 = robot(i).capture(k).number_of_rules_fired;
                   for j1 = 1:count3
                       if (robot(i).capture(k).rules_fired(j1).rules_fired == robot(i).capture(k).current_rules_fired)
                           if(robot(i).capture(k).condition == 1 && robot(i).no_update == 0 && robot(i).capture(k).rules_fired(j1).condition == 1)
                              if(robot(i).capture(k).current_reward_max_cap > robot(i).capture(k).rules_fired(j1).reward_max_cap)
                                reward_max_capture = robot(i).capture(k).rules_fired(j1).reward_max_cap;
                                current_reward_max_cap = robot(i).capture(k).current_reward_max_cap;
                                robot(i).capture(k).rules_fired(j1).reward_max_cap_old = robot(i).capture(k).rules_fired(j1).reward_max_cap;
                                robot(i).capture(k).rules_fired(j1).reward_max_cap = robot(i).capture(k).current_reward_max_cap;
                                robot(i).capture(k).rules_fired(j1).condition = robot(i).capture(k).condition;
                                if(robot(i).capture(k).rules_fired(j1).condition ~= robot(i).capture(k).rules_fired(j1).condition_old)
                                    sprintf(' The rule condition changed for pursuer %d and evader %d, the count is %d and the epoch is %d ', i, k, count, j)
                                    rule_capture_condition_change = 1;
                                end
                              end
                           end
                           if(robot(i).capture(k).condition == 0 && robot(i).no_update == 0 && robot(i).capture(k).rules_fired(j1).condition == 0)
                              if(robot(i).capture(k).current_reward_max_no_cap > robot(i).capture(k).rules_fired(j1).reward_max_no_cap)
                                robot(i).capture(k).rules_fired(j1).reward_max_no_cap_old = robot(i).capture(k).rules_fired(j1).reward_max_no_cap;
                                robot(i).capture(k).rules_fired(j1).reward_max_no_cap = robot(i).capture(k).current_reward_max_no_cap;
%                                 if(robot(i).capture(k).current_reward_max_no_cap > -0.39)
%                                     sprintf(' The no_cap reward is bigger than -0.39, %d ', robot(i).capture(k).current_reward_max_no_cap)
%                                 end
                                robot(i).capture(k).rules_fired(j1).condition = robot(i).capture(k).condition;
                                if(robot(i).capture(k).rules_fired(j1).condition ~= robot(i).capture(k).rules_fired(j1).condition_old)
                                    sprintf(' The rule condition changed for pursuer %d and evader %d, the count is %d and the epoch is %d ', i, k, count, j)
                                    rule_capture_condition_change = 1;
                                    robot(i).no_update = 1;
                                end
                              end
                           end
                       end
                   end
                end
             end
          end
          % Compute the update for the pursuers
          %
          for i=1:no_of_robots
%              if(robot(i).no_update == 1)
%                 sprintf(' No update for pursuer %d, the count is %d and the epoch is %d ', i, count, j) 
%              end
             if (robot(i).no_update == 0)
             for k=1:no_of_robots
                if (robot(i).type == 1 && robot(k).type == 2)
                    inputs = [robot(i).rel_pos(k).x, robot(i).rel_pos(k).y, robot(k).heading];
                    %inputs = [robot(1).rel_pos(k).x, robot(1).rel_pos(k).y, robot(1).heading, robot(2).rel_pos(k).x, robot(2).rel_pos(k).y, robot(2).heading, robot(3).rel_pos(k).x, robot(3).rel_pos(k).y, robot(3).heading, robot(k).heading];
                    v_inputs = [robot(i).rel_pos(k).virtualx(1), robot(i).rel_pos(k).virtualy(1), robot(k).vheading];
                    %[value, phi_norm, not_zero_phi, not_zero_count, rules_fired] = compute_robot_state_value(robot(i).capture(k), inputs, robot(i).critic.input );
                    %[phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_rules_fired_set(robot(i).capture(k), inputs, robot(i).critic.input);
                    [phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_rules_fired_setv81(robot(i).capture(k), inputs, robot(i).critic.input, max_rules_fired);
                    [v_phi_norm, v_not_zero_phi, v_rule_fire_count, v_rules_fired] = compute_rules_fired_setv81(robot(i).capture(k), v_inputs, robot(i).critic.input, max_rules_fired);
                    %[robot(i).capture(k), rule_set_number, rule_found] = search_for_rule_set(robot(i).capture(k), rules_fired);
                    [rule_set_number_new, number, condition_old, rule_found] = search_for_rule_set(robot(i).capture(k), rules_fired);
                    [v_rule_set_number_new, v_number, v_condition_old, v_rule_found] = search_for_rule_set(robot(i).capture(k), v_rules_fired);
                    %[value] = compute_robot_state_valuev8(robot(i).capture(k), phi_norm, rule_fire_count, rules_fired);
                    if (rule_found == 1 && v_rule_found == 1)
                    [value] = compute_robot_state_valuev81(robot(i).capture(k), rule_fire_count, rule_set_number_new);
                    [v_value] = compute_robot_state_valuev81(robot(i).capture(k), v_rule_fire_count, v_rule_set_number_new);
                    % robot(i).capture(k).phi_norm_critic = phi_norm;
                    % robot(i).capture(k).phi_norm_actor = phi_norm;
                   % robot(i).capture(k).value_old = robot(i).capture(k).value;
                    robot(i).capture(k).value = value;
                    robot(i).capture(k).v_value(1) = v_value;
                    %rule_set_number_old = robot(i).capture(k).rule_set_number;
                    %robot(i).capture(k).rule_set_number = rule_set_number_new;
                    %robot(i).capture(k).current_rules_fired = rules_fired;
                    %robot(i).capture(k).rule_fire_count = rule_fire_count;
                    %
                    % Do not update if the capture condition changed to
                    % fail
                    %
%                     if (j == 500)
%                         sprintf(' This is the 500th epoch')
%                     end
                    if (robot(i).capture(k).condition_change_to_fail == 0 && rule_capture_condition_change == 0)
                      % [robot(i).capture(k)] = compute_critic_updatev6(robot(i).capture(k));
                       [robot(i).capture(k), set_rule_flag, rule_count] = compute_critic_updatev8(robot(i).capture(k), i, k);
%                        if (set_rule_flag == 1 && i == 4 && k == 1)
%                            if(td_count == 12300)
%                                sprintf(' This is the td_count 12300')
%                            end
%                            td_plot(td_count) = robot(i).capture(k).rules_fired(rule_count).td;
%                            no_capture_psi1(td_count) =  robot(i).capture(k).rules_fired(rule_count).psi_parameters_no_capture(1);
%                            phi1_plot(td_count) = robot(i).capture(k).phi_norm_critic(32);
%                            td_count = td_count + 1;
%                        end
                       %[robot(i).capture(k)] = compute_actor_updatev6(robot(i).capture(k),robot(i).noise);
					   %[robot(i).capture(k)] = v_compute_critic_updatev7(robot(i).capture(k), robot(i).no_update, no_of_v_robots);
					   [robot(i).capture(k)] = v_compute_actor_updatev8(robot(i).capture(k),robot(i).vnoise, robot(i).no_update, no_of_v_robots);
                    end
                    end
                end
             end
             end
          end
          %************************************************************
          % Compute the update for the evaders Change to Target *******
          %
          for i=1:no_of_robots
                if (robot(i).type == 2)
                   % sprintf(' ** Now do the update section ** ')
                   % sprintf(' The target desired heading is, %f', robot(i).target.des_heading)
                   % sprintf(' The robot heading is, %f', robot(i).heading)
                    robot(i).heading_error = robot(i).target.des_heading - robot(i).heading;
                    inputs = robot(i).heading_error;
                   % sprintf(' The robot heading_error is, %f', robot(i).heading_error)
                    %sprintf(' The robot noise is, %f', noise)
                    [action] = compute_evader_action(robot(i).target, inputs, robot(i).actor.input);
                    error_in_action = robot(i).heading_error - action;
                    [value, phi_norm, not_zero_phi, rules_fired] = compute_robot_state_value(robot(i).target, inputs, robot(i).critic.input);
                    robot(i).target.phi_norm_critic = phi_norm;
                    robot(i).target.phi_norm_actor = phi_norm;
                   % sprintf(' The computed action is, %f', action)
                    %sprintf(' The error in action is, %f', error_in_action)
                   % sprintf(' The computed old value is, %f', value)
                    robot(i).target.value_old = value;
                    %
                    % I now have to take action
                    % add in some noise
                    %
                    noise = normrnd(0,robot(i).target.sigma);
                  %  sprintf(' The robot noise is, %f', noise)
                    action = action + noise;
                    %robot(i).heading = robot(i).heading + action;
                    heading = robot(i).heading + action;
                    robot(i).heading_error = robot(i).target.des_heading - heading;
                    inputs = robot(i).heading_error;
                  %  sprintf(' The robot heading after action and noise added is, %f', robot(i).heading)
                   % sprintf(' The robot heading_error after action and noise added is, %f', robot(i).heading_error)
                    [value, phi_norm, not_zero_phi, rules_fired] = compute_robot_state_value(robot(i).target, robot(i).heading_error, robot(i).critic.input);
                    %robot(i).target.phi_norm_critic = phi_norm;
                    %robot(i).target.phi_norm_actor = phi_norm;
                    robot(i).target.value = value;
                    %sprintf(' The new computed value is, %f', value)
                    delup = robot(i).heading_error;
                    reward_capture_heading = 2*exp(-(delup^2/0.1)) - 1;
                   % sprintf(' The reward is, %f', reward_capture_heading)
                    robot(i).target.reward_capture_heading = 2*exp(-(delup^2/0.1)) - 1;
                    %
                    % Do not update if the capture condition changed to
                    % fail
                    %
                    [robot(i).target] = compute_critic_update(robot(i).target);
                    [robot(i).target] = compute_actor_update(robot(i).target, noise);
                   % sprintf(' The new actor weights are')
                    %robot(i).target.w
                   % sprintf(' The new critic weights are')
                   % robot(i).target.psi
                end
          end
          %************************************************************
          for i=1:no_of_robots
             for k=1:no_of_robots
                 if (robot(i).type == 1 && robot(k).type == 2)
                    %sprintf(' The relative position is x(%d, %d) = %f and y = %f', i, k, robot(i).rel_pos(k).x, robot(i).rel_pos(k).y)
                    dist = sqrt((robot(i).rel_pos(k).x)^2 + (robot(i).rel_pos(k).y)^2);
                    if(dist < 0.5) % We have successfully captured.
                       sprintf(' Pursuer %d has captured the evader. The distance is %f and count is %d and the epoch is %d', i, dist, count, j)
                       if (j > 400)
                          game_on = 0; %Captured
                          if( j > 950)
                             if (i == 4)
                                fileName = sprintf('Epoch_%d.jpg', j); % define the file name
                                saveas( gamePlot, [ pwd strcat('/', folderName, '/', fileName, '.png') ]  );  % save the file
                                movie_stop = 1;
                             end
                          end
                       end
                       [capture, psi, w, alpha, beta, sigma] = robot_captured(robot(i).capture(k), count);
                       capture
                       robot(i).capture(k) = capture;
                       robot(i).capture(k).psi = psi;
                       robot(i).capture(k).w = w;
                       robot(i).capture(k).alpha = alpha;
                       robot(i).capture(k).beta = beta;
                       robot(i).capture(k).sigma = sigma;
                       robot(i).psi = psi;
                       robot(i).w = w;
                       robot(i).alpha = alpha;
                       robot(i).beta = beta;
                       robot(i).sigma = sigma;
                      % fileName = sprintf('Epoch_%d.jpg', j); % define the file name
                      % saveas( gamePlot, [ pwd strcat('/', folderName, '/', fileName, '.png') ]  );  % save the file
                    end
                 end
             end
          end
          if( count > count_max) % stop the game
             game_on = 0;
          end
          % Update the current figure with the new location of the players
          % The if statement "if mod(iteration_count,10) == 0" will plot the
          % trajectory of the players after every 10 iterations. This is done to
          % improve the visualization of the plot.
          % ****************************************************************
          if  mod(count,5) == 1
             plot(robot(1).x, robot(1).y, '*m', robot(2).x, robot(2).y, '*r', robot(3).x, robot(3).y, '*b', robot(4).x, robot(4).y, 'kd', 'MarkerFaceColor', 'k' )
             %plot(ya(1), ya(2), '*m', ya(3), ya(4), '*r', ya(5), ya(6), '*m', ya(7), ya(8), 'dk', 'MarkerFaceColor', 'k' )
             % uncomment this line to get real time visualization of the
             % players trajectory. (Warning: May slow down your system.)
             % pause(0.0000001);
             if (j > 950 && movie_stop == 0)
                count2 = count2 + 1;
                movie_frame(count2) = getframe;
             end
          end
          % ****************************************************************
    end %% ****  END the While Loop of Epoch ****%
    %
     for i=1:no_of_robots
             robot(i).alpha = 0.9999*robot(i).alpha;
             robot(i).beta = 0.9999*robot(i).beta;
             robot(i).sigma = 0.9999*robot(i).sigma;
			 robot(i).vsigma = 0.9999*robot(i).vsigma;
             for k=1:no_of_robots
                 if (robot(i).type == 1 && robot(k).type == 2)
                       robot(i).capture(k).alpha = robot(i).alpha;
                       robot(i).capture(k).beta = robot(i).beta;
                       robot(i).capture(k).sigma = robot(i).sigma;
                 end
             end
             
     end               
    % Create a new folder to save all the game plots
    % *******************************************************************
    if j == 1 % Check if this a new simulation
       date_and_time = datestr(clock,0); % obtain the current system time
       folderName = strcat('Simulation_results_', date_and_time); % define the name of the folder
       folderName = strrep(folderName, ' ', '_');  % replace all ' ' with '_'
       folderName = strrep(folderName, ':', '_');  % replace all ':' with '_'
       folderName = strrep(folderName, '-', '_');  % replace all '-' with '_'
       mkdir(folderName) % create new folder
    end
    %
    % Save the game plots in the new folder
    % *******************************************************************
    if  mod(j,100) == 0
      fileName = sprintf('Epoch_%d.jpg', j); % define the file name
      saveas( gamePlot, [ pwd strcat('/', folderName, '/', fileName, '.png') ]  );  % save the file
    end 
  end
  robot(1).target.w
  for i=1:no_of_robots
     for k=1:no_of_robots
         if (robot(i).type == 1 && robot(k).type == 2)
             sprintf(' The Game Is Over Print out the final Capture information')
             robot(i).capture(k)
         end
     end
  end
end

