function [heading] = compute_robot_action( robot, inputs )
% This function computes the robot action
% The inputs are the robot structure and the input vector
% The output is the heading, but can be anything
%
    no_of_rules = robot.no_of_rules_actor;
    rule.mf = robot.rule_actor
    phi = zeros(1, no_of_rules); 
    for i=1:no_of_rules
       phi(i) = fire_strength_for_rule(inputs, rule.mf); % compute the firing strenght
    end
    %
    % Compute the normalized firing strength for each rule.
    %
    phi_sum = sum(phi);    
    phi1_norm = zeros(1, no_of_rules); 
    if (phi_sum ~= 0)
      for i=1:no_of_rules
        phi1_norm(i) = phi(i)/phi_sum;
      end
    end
    %
    % Compute the action for pursuer 1
    %
    u = 0;
    for i=1:no_of_rules
        u = u + phi1_norm(i)*wl1(i);
        %sprintf(' phi_norm(%d) is %f ', i, phi_norm(i))
    end
    heading = u;
end

