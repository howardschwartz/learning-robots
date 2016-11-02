function [robot, no_of_robots] = init_robots(robot_init)
% Initializes the robot structures
% This function is called with arguments and returns the robots'
% Initial state
%
% Start by setting number of robots to 4
%
   no_of_robots = 4;
   for i=1, no_of_robots
      robot(i).x =  robot_init(i).x;
      robot(i).y =  robot_init(i).y;
      robot(i).type = robot_init(i).type; % Is it a pursuer 1, or evader 2
      robot(i).speed =  0; % Start from not moving
      robot(i).heading = 0;   % start with zero heading
      for j=1,no_of_robots
         robot(i).capture(j).condition = 0; % cannot capture
         robot(i).capture(j).alpha = 0;
         robot(i).capture(j).des_heading = 0;
         robot(i).capture(j).del_heading = 0;
      end
      robot(i).sigma = 1.0; % initial exploration variance
      robot(i).alpha = 0.1; % initial critic learning rate
      robot(i).beta = 0.05; % Initial actor learning rate
      [rule_critic, no_of_rules_critic] = init_mf_rules(robot_init(i).critic);
      robot(i).no_of_rules_critic = no_of_rules_critic; % number of rules
      robot(i).rule_critic = rule_critic; 
      robot(i).psil = zeros(1, no_of_rules_critic); % initialze critic Parameters
      [rule_actor, no_of_rules_actor] = init_mf_rules(robot_init(i).actor);
      robot(i).no_of_rules_actor = no_of_rules_actor; % number of rules
      robot(i).rule_actor = rule_actor; 
      robot(i).psil = zeros(1, no_of_rules_actor); % initialze critic Parameters
      robot(i).wl = zeros(1, no_of_rules_actor): % initialze actor parameters
      robot(i).reward_capture_heading = 0;
      robot(i).gamma_capture_heading = 0;
      robot(i).reward_closing_distance = 0;
      robot(i).gamma_closing_distance = 0.9;
      robot(i).reward_line_of_sight = 0;
      robot(i).gamma_line_of_sight = 0;
   end
end

