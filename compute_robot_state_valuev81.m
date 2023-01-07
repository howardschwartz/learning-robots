function [value] = compute_robot_state_valuev81(robot, rule_fire_count, rule_set_number)
% [value1] = compute_robot_state_valuev81(robot, rule_fire_count, rule_set_number)
%   This function computes the state value for the rule set that fired.
%
    set_rule_flag = 0;
    psi_parameters_capture = robot.rules_fired(rule_set_number).psi_parameters_capture;
    psi_parameters_no_capture = robot.rules_fired(rule_set_number).psi_parameters_no_capture;
    rules_fired = robot.rules_fired(rule_set_number).rules_fired;
    number = robot.rules_fired(rule_set_number).number;
    phi_norm = robot.phi_norm_critic;
    phi_norm1 = robot.rules_fired(rule_set_number).phi_norm;
%     for i=1:rule_fire_count
%        i1 = rules_fired(i);
%        if (phi_norm1(i1) ~= phi_norm(i1))
%           sprintf(' The phi_norm is wrong ')
%        end
%     end
    sum_phi_norm = sum(phi_norm);
    if(sum_phi_norm > 1.001 || sum_phi_norm < 0.99)
        sprintf(' The phi_norm is wrong not equal to 1 ')
    end
%     if(rules_fired == [32, 33, 39, 40, 53, 54, 60, 61, 0, 0])
%        % sprintf(' The rule has fired in compute_critic_update');
%         set_rule_flag = 1;
%     end
%     if(number == 10639)
%         sprintf('This is number 10639')
%     end
%     if(set_rule_flag == 1 && number > 10000 && psi_parameters_no_capture(1) < -0.5)
%         sprintf(' Something in compute value is wrong %f ', psi_parameters_no_capture(1)) 
%     end
    value = 0;
    if (robot.condition == 1)
       for i=1:rule_fire_count
           i1 = rules_fired(i);
           value = value + phi_norm(i1)*psi_parameters_capture(i);
       end
    end
    %
    % Compute the value for pursuer CANNOT capture
    % 
    if (robot.condition == 0)
        for i=1:rule_fire_count
            i1 = rules_fired(i);
            value = value + phi_norm(i1)*psi_parameters_no_capture(i);
        end
%         if(set_rule_flag == 1 && value < -0.5 && number > 10000)
%             sprintf(' The value is wrong ')
%         end
    end
end

