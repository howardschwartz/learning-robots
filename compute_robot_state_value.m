function [value, phi_norm] = compute_robot_state_value( robot_capture, input )
% [value] = compute_robot_state_value(robot, input)
% Computes the value of the state
%
    no_of_rules = robot_capture.no_of_rules_critic;
    rule = robot_capture.rule_critic;
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
    % Compute the value for pursuer CAN capture
    %
    value = 0;
    if (robot_capture.condition == 1)
       for i=1:no_of_rules
           value = value + phi_norm(i)*robot_capture.psi(i);
       end
    end
    %
    % Compute the value for pursuer CANNOT capture
    %
    if (robot_capture.condition == 0)
        for i=1:no_of_rules
           value = value + phi_norm(i)*robot_capture.no_capture_psi(i);
        end
    end
end

