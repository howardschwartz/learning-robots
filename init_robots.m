function [robot] = init_robots(robot_init, no_of_robots)
% Initializes the robot structures
% This function is called with arguments and returns the robots'
% Initial state
%
% Start by setting number of robots to 4
%
%   no_of_robots = 4;
   for i=1:no_of_robots
      robot(i).x =  robot_init(i).x;
      robot(i).y =  robot_init(i).y;
      robot(i).type = robot_init(i).type; % Is it a pursuer 1, or evader 2
      robot(i).speed =  robot_init(i).speed; % Start from not moving
      robot(i).velx = 0; % velocity x component
      robot(i).vely = 0; % velocity y component
      robot(i).heading = robot_init(i).heading;   % start with zero heading
      [rule_critic, no_of_rules_critic] = init_mf_rules_robot(robot_init(i).critic);
      [rule_actor, no_of_rules_actor] = init_mf_rules_robot(robot_init(i).actor);
      for j=1:no_of_robots
         robot(i).capture(j).condition = 0; % cannot capture
         robot(i).capture(j).cond_change = 0; %Capture condition has not changed
         robot(i).capture(j).count_success = 150; %How many steps to capture
         robot(i).capture(j).alpha_success = 0.1;
         robot(i).capture(j).beta_success = 0.05;
         robot(i).capture(j).psi_init = zeros(1, no_of_rules_critic); % initialze critic Parameters
         robot(i).capture(j).w_init = zeros(1, no_of_rules_actor); % initialze actor parameters
         robot(i).capture(j).psi_success_init = zeros(1, no_of_rules_critic); % initialze critic Parameters
         robot(i).capture(j).w_success_init = zeros(1, no_of_rules_actor); % initialze actor parameters
         robot(i).capture(j).sigma_success = 1.0;
         robot(i).capture(j).alpha = 0.1;
         robot(i).capture(j).beta = 0.05;
         robot(i).capture(j).sigma = 1.0;
         robot(i).capture(j).des_heading = 0;
         robot(i).capture(j).del_heading = 0;
         robot(i).rel_pos(j).x = 0;
         robot(i).rel_pos(j).y = 0;
         robot(i).rel_vel(j).x = 0;
         robot(i).rel_pos(j).y = 0;
         robot(i).los(j) = 0;
      end
      robot(i).sigma = 1.0; % initial exploration variance
      robot(i).alpha = 0.1; % initial critic learning rate
      robot(i).beta = 0.05; % Initial actor learning rate
      robot(i).no_of_rules_critic = no_of_rules_critic; % number of rules
      robot(i).rule_critic = rule_critic; 
      robot(i).psi = zeros(1, no_of_rules_critic); % initialze critic Parameters
      robot(i).phi_norm_critic = zeros(1, no_of_rules_critic);
      robot(i).no_of_rules_actor = no_of_rules_actor; % number of rules
      robot(i).rule_actor = rule_actor; 
      robot(i).w = zeros(1, no_of_rules_actor); % initialze actor parameters
      robot(i).phi_norm_actor = zeros(1, no_of_rules_actor);
      robot(i).reward_capture_heading = 0;
      robot(i).gamma_capture_heading = 0;
      robot(i).reward_closing_distance = 0;
      robot(i).gamma_closing_distance = 0.9;
      robot(i).reward_line_of_sight = 0;
      robot(i).gamma_line_of_sight = 0;
      robot(i).value = 0;
      robot(i).value_old = 0;
      robot(i).noise = 0;
      robot(i).dynamics = @pedestrian;
   end
end

