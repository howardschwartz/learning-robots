function [value, phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_robot_state_value( robot, input, input_range)
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
    not_zero_count = 0;
    rules_fired = zeros(1,10);
    rule_fire_count = 0;
    if (robot.condition == 1)
      not_zero_phi = robot.capture_not_zero_phi;
    end
    if (robot.condition == 0)
      not_zero_phi = robot.no_capture_not_zero_phi;
    end
    for i=1:no_of_rules
        phi(i) = fire_strength_for_rule(input, rule(i).mf, input_range);
         if(phi(i) < -0.001 || phi(i) > 0.001)
            not_zero_count = not_zero_count + 1;
            not_zero_phi(i) = not_zero_phi(i) + 1;
            rule_fire_count = rule_fire_count + 1;
            rules_fired(rule_fire_count) = i;
        end
    end
    %
    % Compute the normalized firing strength for each rule.
    %
%     phi_sum = sum(phi);
%     phi_norm = zeros(1, no_of_rules);
%     if (phi_sum ~= 0)
%        for i=1:no_of_rules
%            phi_norm(i) = phi(i)/phi_sum;
%        end
%     end
    phi_norm = phi;
    %
    % Compute the new value of the state
    % Compute the value for pursuer CAN capture
    %
    value = 0;
    if (robot.condition == 1)
        psi = robot.psi;
       %for i=1:no_of_rules
%           value = value + phi_norm(i)*psi(i);
       for i=1:not_zero_count
           i1 = rules_fired(i);
           value = value + phi_norm(i1)*psi(i1);
       end
    end
%        if (value > 10.0 || value < -10.0)
%           sprintf(' The can capture value %f ', value)
%        end
%    end
    %
    % Compute the value for pursuer CANNOT capture
    % 
    if (robot.condition == 0)
        no_capture_psi = robot.no_capture_psi;
        for i=1:not_zero_count
            i1 = rules_fired(i);
            value = value + phi_norm(i1)*no_capture_psi(i1);
        end
        %for i=1:no_of_rules
           %value = value + phi_norm(i)*no_capture_psi(i);
           %value = value + phi_norm(i)*psi(i);
       % end
         %if (value > 10.0 || value < -10.0)
 %          sprintf(' The cannot capture value %f ', value)
      % end
    end
end

