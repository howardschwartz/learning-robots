function [value, phi_norm] = compute_robot_state_value( robot, input, input_range)
% [value] = compute_robot_state_value(robot, input)
% Computes the value of the state
%
    % Input out of range
    %
    %if (input < robot_init(i).actor.mf_per_input(1).range(1,1)
    %
    no_of_rules = robot.no_of_rules_critic;
    rule = robot.rule_critic;
    phi = zeros(1, no_of_rules); 
    for i=1:no_of_rules
        phi(i) = fire_strength_for_rule(input, rule(i).mf, input_range);
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
    % Compute the value for pursuer CAN capture
    %
    value = 0;
    psi = robot.psi;
    if (robot.condition == 1)
       for i=1:no_of_rules
           value = value + phi_norm(i)*psi(i);
       end
    end
    %
    % Compute the value for pursuer CANNOT capture
    %
    no_capture_psi = robot.no_capture_psi;
    if (robot.condition == 0)
        for i=1:no_of_rules
           value = value + phi_norm(i)*no_capture_psi(i);
        end
    end
end

