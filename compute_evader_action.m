function [ action ] = compute_evader_action(robot, input, input_range)
% compute_evader_action (robot, inputs) Computes the heading of the 
% evader with respect to the given pursuer.
%
% This function computes the robot action
% The inputs are the robot structure and the input vector
% The output is the heading, but can be anything
%
    no_of_rules = robot.no_of_rules_actor;
    rule = robot.rule_actor;
    phi = zeros(1, no_of_rules);
    for i=1:no_of_rules
       phi(i) = fire_strength_for_rule(input, rule(i).mf, input_range); % compute the firing strength
    end
    %
    % Compute the normalized firing strength for each rule.
    %
    phi_sum = sum(phi);    
    phi_norm = zeros(1, no_of_rules); 
    if (phi_sum ~= 0)
      for i=1:no_of_rules
        phi_norm(i) = phi(i)/phi_sum;
      end
    end
    %
    % Compute the action for evader
    %
    u = 0;
    w = robot.w;
    if (robot.condition == 1) 
       for i=1:no_of_rules
          u = u + phi_norm(i)*w(i);
        %sprintf(' phi_norm(%d) is %f ', i, phi_norm(i))
       end
    end
    %
    % Compute the action for pursuer CANNOT capture
    %
    if (robot.condition == 0) 
       for i=1:no_of_rules
          u = u + phi_norm(i)*robot.no_capture_w(i);
        %sprintf(' phi_norm(%d) is %f ', i, phi_norm(i))
       end
    end
    action = u;
end

