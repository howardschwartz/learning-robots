function [robot, no_of_robots] = robot_pursuer_evader()
   % Simulation of the multi-Robot pursuer evader machine learning game
   % Started by Prof. Schwartz Oct. 30, 2016
   %
   % Read in the robot data
   %
   fid = fopen('robot.txt');
   no_of_data = [25, inf];
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
   [robot] = init_robots(robot_init, no_of_robots);
   %
   % Reset the membership functions and inputs for the pursuers.
   % Added hms 06302017
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
           for k = 1:no_of_robots
               if(robot(k).type == 1)
                   robot(i).pursuer(k).psi_init = zeros(1, no_of_rules_critic); % initialze critic Parameters
                   robot(i).pursuer(k).w_init = zeros(1, no_of_rules_actor); % initialze actor parameters
                   robot(i).pursuer(k).psi_success_init = zeros(1, no_of_rules_critic); % initialze critic Parameters
                   robot(i).pursuer(k).w_success_init = zeros(1, no_of_rules_actor); % initialze actor parameters
                   robot(i).pursuer(k).psi = zeros(1, no_of_rules_critic);
                   robot(i).pursuer(k).w = zeros(1, no_of_rules_actor);
                   robot(i).pursuer(k).no_capture_psi = zeros(1, no_of_rules_critic);
                   robot(i).pursuer(k).no_capture_w = zeros(1, no_of_rules_actor);
                   robot(i).pursuer(k).psi_success = zeros(1, no_of_rules_critic);
                   robot(i).pursuer(k).w_success = zeros(1, no_of_rules_actor);
                   robot(i).pursuer(k).no_of_rules_critic = no_of_rules_critic; % number of rules
                   robot(i).pursuer(k).rule_critic = rule_critic; 
                   robot(i).pursuer(k).no_of_rules_actor = no_of_rules_actor; % number of rules
                   robot(i).pursuer(k).rule_actor = rule_actor;
                   robot(i).pursuer(k).phi_norm_critic = zeros(1, no_of_rules_critic);
                   robot(i).pursuer(k).phi_norm_actor = zeros(1, no_of_rules_actor);
               end 
           end
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
   % end addition 06302017
   %
   % Initialize the counters
   %
   count = 0;
   %
   % Compute the capture condition between each robot
   %
   for i = 1:no_of_robots
       for k = 1:no_of_robots
           [condition, up_des, delup] = capture_condition(robot(i), robot(k));
           robot(i).capture(k).condition = condition; %can pursuer i capture evader k?
           %hms 06272017
           robot(k).pursuer(i).condition = condition; %can evader k be captured by pursuer i?
           %hms 06272017
           robot(i).capture(k).des_heading = up_des;
           robot(i).capture(k).del_heading = delup;
       end
   end
  game_no = 500;
  %
  % % start here %
  % ***************************************************************
  for j=1:game_no % This is the loop for each epoch
    sprintf(' The game number is %d ', j)
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
             %hms 06272017
             robot(k).pursuer(i).condition = condition; %can evader k be captured by pursuer i?
             %hms 06272017
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
    count = 0;
    game_on = 1; %start the game
    dt = 0.1; % sampling time in seconds
    %
    % Initialize the figure that we make use of to plot the trajectories
    %close all % Close all open figures
    gamePlot = figure('visible','off'); % Create new figure but don't display it
    axis([-10 25 -10 25]) % set the axis of the figure
    hold on % ensure continuous plot on the same figure
    grid on % turn on the grid lines
    % *******************************************************************
    % *******************************************************************
    while(game_on == 1)
          count = count + 1;
          %sprintf(' The count is %d the game number is %d ', count, j)
          [robot, rel_dist, rel_speed, los] = compute_rel_dist_vel_los(robot, no_of_robots);
          %
          % First compute the actions of the pursuers
          %
          for i=1:no_of_robots
             for k=1:no_of_robots
                if (robot(i).type == 1 && robot(k).type == 2)
                    inputs = [robot(i).rel_pos(k).x, robot(i).rel_pos(k).y, robot(k).heading];
                    [action] = compute_robot_action(robot(i).capture(k), inputs, robot(i).actor.input);
                    [value, phi_norm] = compute_robot_state_value(robot(i).capture(k), inputs, robot(i).critic.input);
                    robot(i).capture(k).value_old = value;
                    robot(i).capture(k).heading = action;
                    robot(i).capture(k).phi_norm_critic = phi_norm;
                    robot(i).capture(k).phi_norm_actor = phi_norm;
                    robot(i).capture(k).dist = rel_dist(i, k);
                    robot(i).capture(k).condition_change_to_fail = 0;
                    robot(i).heading = action;
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
          % Compute the evader action separately from pursuer hms 06272017
          % Compute the action with respect to each pursuer 
          for i=1:no_of_robots
             for k=1:no_of_robots
                if (robot(i).type == 2 && robot(k).type == 1)
                    robot(i).pursuer(k).heading_error = robot(i).pursuer(k).des_heading - robot(i).pursuer(k).heading;
                    inputs = robot(i).pursuer(k).heading_error;
                    robot(i).pursuer(k).dist = rel_dist(i, k);
                    [action] = compute_evader_action(robot(i).pursuer(k), inputs, robot(i).actor.input);
                    [value, phi_norm] = compute_robot_state_value(robot(i).pursuer(k), inputs, robot(i).critic.input);
                    robot(i).pursuer(k).value_old = value;
                    robot(i).pursuer(k).heading_error = action;
                    robot(i).pursuer(k).heading = robot(i).pursuer(k).heading + action;
                    robot(i).pursuer(k).phi_norm_critic = phi_norm;
                    robot(i).pursuer(k).phi_norm_actor = phi_norm;
                    robot(i).pursuer(k).condition_change_to_fail = 0;
                    % robot(i).heading = action; Need to determine the best
                    % action
                end
             end
          end 
          %
          % Determine which heading to follow
          %
          for i=1:no_of_robots
             for k=1:no_of_robots
                if (robot(i).type == 2 && robot(k).type == 1)
                    if (robot(i).pursuer(k).condition == 1 && robot(i).pursuer(k).dist < 1)
                        sprintf(' The condition for robot(%d).pursuer(%d) = %d ', i, k, robot(i).pursuer(k).condition)
                        sprintf(' The distance is for robot(%d).pursuer(%d) = %d ', i, k, robot(i).pursuer(k).dist)
                       % hms08272017 robot(i).heading = robot(i).pursuer(k).heading;
                        %robot(i).target.des_heading = los(nearest, evader);
                        % hms07272017 have to indicate that evader is now
                        % running away
                        robot(i).runaway = k;
                        sprintf(' The target desired heading has changed to, %f', robot(i).target.des_heading)
                    end
                end
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
                [value, phi_norm] = compute_robot_state_value(robot(i).target, inputs, robot(i).critic.input );
              %  sprintf(' The computed action is, %f', action)
                robot(i).target.value_old = value;
                robot(i).target.heading = robot(i).target.heading + action;
                heading = robot(i).heading + action;
                robot(i).heading = heading;
                if(robot(i).heading > pi)
                    robot(i).heading = pi;
                end
                 if(robot(i).heading < -pi)
                    robot(i).heading = -pi;
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
                  robot(i).noise = normrnd(0,robot(i).sigma);
                  robot(i).heading = robot(i).heading + robot(i).noise;
                  if(robot(i).heading > pi)
                    robot(i).heading = pi;
                  end
                  if(robot(i).heading < -pi)
                    robot(i).heading = -pi;
                  end
                  if(robot(i).type == 2)
                     sprintf(' The robot heading is, %f', robot(i).heading)
                     sprintf(' The noise is for robot(%d), %f', i, robot(i).noise)
                  end
              end
              if (robot(i).type == 2) % evader learning
                  if(robot(i).heading > pi)
                    robot(i).heading = pi;
                  end
                  if(robot(i).heading < -pi)
                    robot(i).heading = -pi;
                  end
                 % sprintf(' The robot heading is, %f', robot(i).heading)
              end
          end
          %
          % Lets move the robots one step
          %***********************************************************
          %
          [robot] = move_robots(robot, no_of_robots);
          %***********************************************************
          %
          % Recompute the relative distances and the new value
          %
          [robot, rel_dist, rel_speed, los] = compute_rel_dist_vel_los(robot, no_of_robots);
          %
          % Recompute the capture conditions
          %
          for i = 1:no_of_robots
             for k = 1:no_of_robots
                if (robot(i).type == 1 && robot(k).type == 2)
                   [condition, up_des, delup] = capture_condition(robot(i), robot(k));
                   %
                   % Check if the capture condition changed
                   if (robot(i).capture(k).condition == 0 && condition == 1)
                       sprintf(' The pursuer %d can now capture evader %d, the count is %d and the epoch is %d ', i, k, count, j)
                   end
                   %
                   if (robot(i).capture(k).condition == 1 && condition == 0)
                       sprintf(' The pursuer %d can no longer capture evader %d, the count is %d and the epoch is %d ', i, k, count, j)
                       robot(i).capture(k).condition_change_to_fail = 1;
                       robot(i).capture(k).condition = 0;
                       robot(k).pursuer(i).condition = 0;
                       [psi, w, sigma] = change_capture_condition(robot(i).capture(k));
                       robot(i).capture(k).psi = psi;
                       robot(i).capture(k).w = w;
                       robot(i).capture(k).sigma = sigma;
                      % game_on = 0; % End the game
                   end
                   robot(i).capture(k).condition = condition;
                   robot(i).capture(k).des_heading = up_des;
                   robot(i).capture(k).del_heading = delup;
                   reward_capture_heading = 2*exp(-(delup^2)) - 1;
                   robot(i).capture(k).reward_capture_heading = 2*exp(-(delup^2)) - 1;
                   robot(i).capture(k).reward_rel_vel = -rel_speed(i,k);
                   %sprintf(' The reward is %f ', robot(i).reward_capture_heading)
                end
             end
          end
          %
          % Check if anybody can capture.
          %
          can_capture = zeros(no_of_robots, no_of_robots);
          someone_can_capture = 0;
          for i = 1:no_of_robots
             for k = 1:no_of_robots
                if (robot(i).type == 1 && robot(k).type == 2)
                    if (robot(i).capture(k).condition == 1)
                        can_capture(i, k) = 1;
                        someone_can_capture = 1;
                    end
                end
             end
          end
          if (someone_can_capture == 0)
              %game_on = 0; % No one can capture so end the game
              %sprintf(' No one can capture the count is %d and the epoch is %d ', count, j)
          end
          % 
          % Compute the update for the pursuers
          %
          for i=1:no_of_robots
             for k=1:no_of_robots
                if (robot(i).type == 1 && robot(k).type == 2)
                    inputs = [robot(i).rel_pos(k).x, robot(i).rel_pos(k).y, robot(k).heading];
                    [value, phi_norm] = compute_robot_state_value(robot(i).capture(k), inputs, robot(i).critic.input );
                    robot(i).capture(k).phi_norm_critic = phi_norm;
                    robot(i).capture(k).phi_norm_actor = phi_norm;
                    robot(i).capture(k).value_old = robot(i).capture(k).value;
                    robot(i).capture(k).value = value;
                    %
                    % Do not update if the capture condition changed to
                    % fail
                    %
                    if (robot(i).capture(k).condition_change_to_fail == 0)
                       [robot(i).capture(k)] = compute_critic_update(robot(i).capture(k));
                       [robot(i).capture(k)] = compute_actor_update(robot(i).capture(k),robot(i).noise);
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
                    %[value, phi_norm] = compute_robot_state_value(robot(i).target, inputs);
                    [value, phi_norm] = compute_robot_state_value(robot(i).target, inputs, robot(i).critic.input);
                    robot(i).target.phi_norm_critic = phi_norm;
                    robot(i).target.phi_norm_actor = phi_norm;
                   % sprintf(' The computed action is, %f', action)
                    %sprintf(' The error in action is, %f', error_in_action)
                   % sprintf(' The computed old value is, %f', value)
                    robot(i).target.value_old = value;
                    %
                    % I now have to take action
                    %
                    %robot(i).heading = robot(i).heading + action;
                    %sprintf(' The corrected robot heading after adding action is, %f', robot(i).heading)
                    %robot(i).heading_error = robot(i).target.des_heading - robot(i).heading;
                    %sprintf(' The corrected robot heading error after adding action is, %f', robot(i).heading_error)
                    %
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
                    %[action] = compute_evader_action(robot(i).target, inputs);
                    %error_in_action = robot(i).heading_error + noise - action;
                    [value, phi_norm] = compute_robot_state_value(robot(i).target, robot(i).heading_error, robot(i).critic.input);
                    %robot(i).target.phi_norm_critic = phi_norm;
                    %robot(i).target.phi_norm_actor = phi_norm;
                    robot(i).target.value = value;
                    %sprintf(' The new computed value is, %f', value)
                    %delup = robot(i).target.des_heading - robot(i).heading; %This should not be target hms 8/30/2017
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
                % **** This section is a problem *****
                if (robot(i).type == 2 && robot(i).runaway > 0)
                    inputs = robot(i).heading_error;
                    runaway = robot(i).runaway;
                    [value, phi_norm] = compute_robot_state_value(robot(i).pursuer(runaway), inputs, robot(i).critic.input);
                    robot(i).pursuer(runaway).phi_norm_critic = phi_norm;
                    robot(i).pursuer(runaway).phi_norm_actor = phi_norm;
                    robot(i).pursuer(runaway).value_old = robot(i).target.value;
                    robot(i).pursuer(runaway).value = value;
                    delup = robot(i).target.des_heading - robot(i).heading;
                    reward_capture_heading = 2*exp(-(delup^2)) - 1;
                    robot(i).pursuer(runaway).reward_capture_heading = 2*exp(-(delup^2)) - 1;
                    %
                    %
                    [robot(i).pursuer(runaway)] = compute_critic_update(robot(i).pursuer(runaway));
                    [robot(i).pursuer(runaway)] = compute_actor_update(robot(i).pursuer(runaway),robot(i).noise);
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
                       if (j > 100)
                          game_on = 0; %Captured
                          if (i == 4)
                             fileName = sprintf('Epoch_%d.jpg', j); % define the file name
                             saveas( gamePlot, [ pwd strcat('/', folderName, '/', fileName, '.png') ]  );  % save the file
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
          if( count > 200) % stop the game
             game_on = 0;
          end
          % Update the current figure with the new location of the players
          % The if statement "if mod(iteration_count,10) == 0" will plot the
          % trajectory of the players after every 10 iterations. This is done to
          % improve the visualization of the plot.
          % ****************************************************************
          if  mod(count,10) == 1
             plot(robot(1).x, robot(1).y, '*m', robot(2).x, robot(2).y, '*r', robot(3).x, robot(3).y, '*b', robot(4).x, robot(4).y, 'kd', 'MarkerFaceColor', 'k' )
             %plot(ya(1), ya(2), '*m', ya(3), ya(4), '*r', ya(5), ya(6), '*m', ya(7), ya(8), 'dk', 'MarkerFaceColor', 'k' )
             % uncomment this line to get real time visualization of the
             % players trajectory. (Warning: May slow down your system.)
             % pause(0.0000001);
          end
          % ****************************************************************
    end %% ****  END the While Loop of Epoch ****%
    %
     for i=1:no_of_robots
             for k=1:no_of_robots
                 if (robot(i).type == 1 && robot(k).type == 2)
                     robot(i).alpha = 0.9999*robot(i).alpha;
                     robot(i).beta = 0.9999*robot(i).beta;
                     robot(i).sigma = 0.999*robot(i).sigma;
                     robot(i).capture(k).alpha = 0.9999*robot(i).capture(k).alpha;
                     robot(i).capture(k).beta = 0.9999*robot(i).capture(k).beta;
                     robot(i).capture(k).sigma = 0.99*robot(i).capture(k).sigma;
                 end
                 if (robot(i).type == 2 && robot(k).type == 1)
                     robot(i).alpha = 0.999*robot(i).alpha;
                     robot(i).beta = 0.999*robot(i).beta;
                     robot(i).sigma = 0.99*robot(i).sigma;
                     robot(i).pursuer(k).alpha = 0.999*robot(i).pursuer(k).alpha;
                     robot(i).pursuer(k).beta = 0.999*robot(i).pursuer(k).beta;
                     robot(i).pursuer(k).sigma = 0.99*robot(i).pursuer(k).sigma;
                 end
                 if (robot(i).type == 2)
                     robot(i).target.alpha = 1.0*robot(i).target.alpha;
                     robot(i).target.beta = 1.0*robot(i).target.beta;
                     robot(i).target.sigma = 1.0*robot(i).target.sigma;
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

