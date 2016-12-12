function [value, phi_norm] = compute_robot_state_value( robot, input )
% [value] = compute_robot_state_value(robot, input)
% Computes the value of the state
%
    no_of_rules = robot.no_of_rules_critic;
    rule = robot.rule_critic
    phi = zeros(1, no_of_rules); 
    for i=1:no_of_rules
        phi(i) = fire_strength_for_rule(input, rule(i).mf);
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
    % Compute the new value of the state
    %
    value = 0;
    for i=1:no_of_rules
        value = value + phi_norm(i)*robot.psi(i);
    end


end

