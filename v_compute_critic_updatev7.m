function [robot] = v_compute_critic_updatev7(robot, no_update, no_of_v_robots)
% [robot] = compute_critic_update( robot )
%   Compute the update to the critic parameters
%
    if (robot.condition == 1)% If it can capture
        reward = robot.v_reward_capture_heading;
        gamma = robot.gamma_capture_heading;
    end
     if (robot.condition == 0)% If it cannot capture
        reward = robot.v_reward_rel_vel;
        %reward = robot.v_reward_heading_difference; 
        gamma = robot.gamma_rel_vel;
    end
    alpha = robot.alpha;
    value = robot.value;
    value_old = robot.value_old;
    phi_norm =robot.phi_norm_critic;
    rules_fired = robot.current_rules_fired;
    no_of_rules = robot.no_of_rules_critic;
    rule_fire_count = robot.rule_fire_count;
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
                if (robot.condition == 1)
                   reward_max = robot.rules_fired(count31).reward_max_cap;
                   reward_max_old = robot.rules_fired(count31).reward_max_cap_old;
                end
                if (robot.condition == 0)
                   reward_max = robot.rules_fired(count31).reward_max_no_cap;
                   reward_max_old = robot.rules_fired(count31).reward_max_no_cap_old;
                end
                 number_of_times_fired = robot.rules_fired(count31).number;
                 td_old = robot.rules_fired(count31).td;
                 if(reward_max > -100)
                     td = (reward_max + gamma*value) - value_old;
                     td_change = abs(td_old) - abs(td);
                     if(td_change < -0.5 && number_of_times_fired > 20)
                         sprintf(' The value of td_change is %f ', td_change) 
                     end
                     if( td == 100)
                        sprintf(' The value of td is %f ', td) 
                     end
                     robot.rules_fired(count31).td = td;
                 end
                 rule_found = 1;
             end
         end
    end
    for k=1:no_of_v_robots
        if (no_update(k) == 0)
          value = robot.v_value(k);
          %
          % If the robot can capture
          %
          if (robot.condition == 1)% If it can capture
             psi = robot.psi;
             for j=1:no_of_rules
                 psi(j) = psi(j)+ alpha*td*phi_norm(j);
                 if (psi(j) > 1)
                   sprintf(' Value psi(%d) is %f', j, psi(j))
                 end
             end
             robot.psi = psi;
          end
          %
          % If the robot cannot capture
          %
          if (robot.condition == 0)% If it cannot capture
             no_capture_psi = robot.no_capture_psi;
             for j=1:no_of_rules
                no_capture_psi(j) = no_capture_psi(j)+ alpha*td*phi_norm(j);
                 if (no_capture_psi(j) > 1)
%                    sprintf(' Value no_capture_psi(%d) is %f', j, no_capture_psi(j))
                 end
             end
             robot.no_capture_psi = no_capture_psi;
          end
        end
    end % end for each virtual robot
end


