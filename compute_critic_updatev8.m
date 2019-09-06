function [robot, set_rule_flag, count31] = compute_critic_updatev8(robot, i, k)
% [robot] = compute_critic_update( robot )
%   Compute the update to the critic parameters
    set_rule_flag = 0;
    count31 = 0;
    rules_fired = robot.current_rules_fired;
    rule_set_number = robot.rule_set_number;
    no_of_rules = robot.no_of_rules_critic;
    rule_fire_count = robot.rule_fire_count;
    if(rules_fired == [32, 33, 39, 40, 53, 54, 60, 61, 0, 0])
        if(i == 4 && k == 1)
           sprintf(' The rule has fired in compute_critic_update');
           set_rule_flag = 1;
        end
    end
%
    if (robot.condition == 1)% If it can capture
        reward = robot.reward_capture_heading;
        gamma = robot.gamma_capture_heading;
    end
     if (robot.condition == 0)% If it cannot capture
        %reward = robot.reward_heading_difference;
        reward = robot.reward_rel_vel;
        gamma = robot.gamma_rel_vel;
     end
    go_to_update = 0;
    alpha = robot.alpha;
    value = robot.value;
    value_old = robot.value_old;
    value_old1 = robot.rules_fired(rule_set_number).value;
    if(value_old ~= value_old1)
       sprintf(' value_old is not the same - what changed') 
    end
    phi_norm =robot.phi_norm_critic;
    phi_norm1 = robot.rules_fired(rule_set_number).phi_norm;
    for i=1:rule_fire_count
       i1 = rules_fired(i);
       if (phi_norm1(i1) ~= phi_norm(i1))
          sprintf(' The phi_norm is wrong ')
       end
    end
    sum_phi_norm = sum(phi_norm);
    if(sum_phi_norm > 1.001 || sum_phi_norm < 0.99)
        sprintf(' The phi_norm is wrong not equal to 1 ')
    end
    %
    % Find maximum reward for rule that fired
    %
    rule_found = 0;
    count3 = robot.number_of_rules_fired;
    if (count3 > 0)
        for count31 = 1:count3
        % Has the current rule set been fired yet?
        %
            if(robot.current_rules_fired == robot.rules_fired(count31).rules_fired)
                if(count31 ~= rule_set_number)
                    sprintf(' The rule set fired is different from rule discovered')
                end 
                if (robot.condition == 1 && robot.rules_fired(count31).condition == 1)
                   reward_max = robot.rules_fired(count31).reward_max_cap;
                   reward_max_old = robot.rules_fired(count31).reward_max_cap_old;
                   go_to_update = 1;
                end
                if (robot.condition == 0 && robot.rules_fired(count31).condition == 0)
                   reward_max = robot.rules_fired(count31).reward_max_no_cap;
                   reward_max_old = robot.rules_fired(count31).reward_max_no_cap_old;
                   go_to_update = 1;
                end
                 number_of_times_fired = robot.rules_fired(count31).number;
                 if (number_of_times_fired == 10639 && set_rule_flag == 1 && i == 4 && k == 1)
                      sprintf(' The rule has fired in compute_critic_update 10639') 
                  end
%                  if(rules_fired == [4, 5, 11, 12, 25, 26, 32, 33, 0, 0])
%                      if (number_of_times_fired > 5000)
%                      sprintf(' The rule has fired in compute_critic_update rule 12') 
%                      end
%                  end
                 td_old = robot.rules_fired(count31).td;
                 td_avg = robot.rules_fired(count31).td_avg;
                 td_sigma = robot.rules_fired(count31).td_sigma;
                 if( go_to_update == 1)
                 if(reward_max > -100)
                    % td = (reward_max + gamma*value) - value_old;
                     td = (reward + gamma*value) - value_old;
                     td_change = abs(td_old) - abs(td);
                     if(td_change < -0.2 && number_of_times_fired > 200 && set_rule_flag == 1)
                         sprintf(' The value of td_change is %f ', td_change) 
                     end
                     robot.rules_fired(count31).td = td;
                     % Compute td average.
                     td_avg_new = td_avg + (td - td_avg)/number_of_times_fired;
                     robot.rules_fired(count31).td_avg = td_avg_new;
                     % Compute td standard deviation.
                     td_sigma = td_sigma + (td - td_avg_new)*(td - td_avg);
                     robot.rules_fired(count31).td_sigma = td_sigma;
                     %
                     % Compute the Z - statistic for hyothesis testing
                     %
                     if (td_sigma > 0)
                       zed = sqrt(number_of_times_fired)*td_avg_new/td_sigma;
                       robot.rules_fired(count31).zed = zed;
                     end
                     if(number_of_times_fired > 5000)
                         robot.rules_fired(count31).td_converged = 1;
                     end
                 end
                 end
                 rule_found = 1;
                 break
             end
         end
    end
    %td = (reward + gamma*value) - value;
    %
    % If the robot can capture
    %
    if (robot.condition == 1 && go_to_update == 1)% If it can capture
       psi = robot.rules_fired(count31).psi_parameters_capture;
       for j=1:rule_fire_count
           j1 = rules_fired(j);
           psi(j) = psi(j) + alpha*td*phi_norm(j1);
       end
       robot.rules_fired(count31).psi_parameters_capture = psi;
    end
    %
    % If the robot cannot capture
    %
    if (robot.condition == 0 && go_to_update == 1)% If it cannot capture
       no_capture_psi = robot.rules_fired(count31).psi_parameters_no_capture;
       for j=1:rule_fire_count
           j1 = rules_fired(j);
           delta = alpha*td*phi_norm(j1);
%            if(abs(delta) > 0.1 && set_rule_flag == 1 && number_of_times_fired > 10000)
%                sprintf(' Something in update is wrong %f ', delta) 
%            end
           no_capture_psi(j) = no_capture_psi(j)+ alpha*td*phi_norm(j1);
           %psi(j) = psi(j)+ alpha*td*phi_norm(j);
           %sprintf(' Value psi(%d) is %f', j, psi(j))
       end
%        if(no_capture_psi(1) < -0.5 && set_rule_flag == 1 && number_of_times_fired > 10000)
%            sprintf(' Something in update is wrong %f ', no_capture_psi(1)) 
%        end
       robot.rules_fired(count31).psi_parameters_no_capture = no_capture_psi;
       %robot.psi = psi;
    end
end

