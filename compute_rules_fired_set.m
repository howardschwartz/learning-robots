function [phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_rules_fired_set(robot, input, input_range)
% Determine the rules that fired [phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_rules_fired_set(robot, input, input_range)
%   Detailed explanation goes here
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
    phi_norm = phi;
end

