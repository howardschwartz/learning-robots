function [value] = compute_robot_state_valuev8( robot, phi_norm, rule_fire_count, rules_fired)
%  [value] = compute_robot_state_valuev8( robot, phi_norm, rule_fire_count, rules_fired)
%   Compute the value of the state
%
    value = 0;
    if (robot.condition == 1)
        psi = robot.psi;
       for i=1:rule_fire_count
           i1 = rules_fired(i);
           value = value + phi_norm(i1)*psi(i1);
       end
    end
    %
    % Compute the value for pursuer CANNOT capture
    % 
    if (robot.condition == 0)
        no_capture_psi = robot.no_capture_psi;
        for i=1:rule_fire_count
            i1 = rules_fired(i);
            value = value + phi_norm(i1)*no_capture_psi(i1);
        end
    end
end

