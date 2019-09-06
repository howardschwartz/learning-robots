function [robot] = v_compute_actor_updatev8(robot, noise, no_update, no_of_v_robots)
% [w] = compute_actor_update(robot, no_of_robots)
%  compute the update to the actor parameters
%
    beta = robot.beta;
    value_old = robot.value_old;
    value = robot.value;
    phi_norm =robot.phi_norm_actor;
    rules_fired = robot.current_rules_fired;
    rule_fire_count = robot.rule_fire_count;
    no_of_rules = robot.no_of_rules_actor;
    go_to_update = 0;
    converged  = 0;
   % **********************************************************************
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
                rule_set_number = count31;
                if (robot.condition == 1 && robot.rules_fired(count31).condition == 1)
                   reward_max = robot.rules_fired(count31).reward_max_cap;
                   reward_max_old = robot.rules_fired(count31).reward_max_cap_old;
                   converged  = robot.rules_fired(count31).td_converged;
                   parameters_capture = robot.rules_fired(count31).parameters_capture;
                   go_to_update = 1;
                end
                if (robot.condition == 0 && robot.rules_fired(count31).condition == 0)
                   reward_max = robot.rules_fired(count31).reward_max_no_cap;
                   reward_max_old = robot.rules_fired(count31).reward_max_no_cap_old;
                   converged  = robot.rules_fired(count31).td_converged;
                   parameters_no_capture = robot.rules_fired(count31).parameters_no_capture;
                   go_to_update = 1;
                end
                 number_of_times_fired = robot.rules_fired(count31).number;
                 td_old = robot.rules_fired(count31).td;
                 td_avg_actor = robot.rules_fired(count31).td_avg_actor;
                 td_sigma_actor = robot.rules_fired(count31).td_sigma_actor;
                 rule_found = 1;
                 break
             end
         end
    end
   % *********************************************************************
   
    if (robot.condition == 1 && go_to_update == 1)% If it can capture
%         reward = robot.v_reward_capture_heading;
%         reward_no_noise = robot.reward_capture_heading;
        reward = robot.v_reward_rel_vel;
        reward_no_noise = robot.reward_rel_vel;
        gamma = robot.gamma_capture_heading;
        td_no_noise = (reward_no_noise  + gamma*value) - value_old;
        w  = robot.w;
        % find the smallest td
        %
        for k=1:no_of_v_robots
            value = robot.v_value(k);
            td(k) = (reward(k) + gamma*value) - value_old;
            %td(k) = (reward(k) + gamma*value) - reward_max;
        end
        [td_min, k1] = min(abs(td));
        [v_reward_max, k2] = max(reward);
        td_update = td(k2);
        noise_update = noise(k2);       
        for j=1:rule_fire_count
             j1 = rules_fired(j);
             deltaw = beta*(td_update*noise_update)*phi_norm(j1);
             w(j1) = w(j1)+ deltaw;
             parameters_capture(j) = parameters_capture(j) + deltaw;
        end
        robot.w = w;
        robot.rules_fired(rule_set_number).parameters_capture = parameters_capture;
    end
    if (robot.condition == 0 && go_to_update == 1)% If it cannot capture
        %reward = robot.v_reward_heading_difference;
        reward = robot.v_reward_rel_vel;
        %reward_no_noise = robot.reward_heading_difference;
        reward_no_noise = robot.reward_rel_vel;
        gamma = robot.gamma_rel_vel;
        w = robot.no_capture_w;
        td_no_noise = (reward_no_noise  + gamma*value) - value_old;
        for k=1:no_of_v_robots
              value = robot.v_value(k);
              td(k) = (reward(k) + gamma*value) - value_old;
              %td(k) = (reward(k) + gamma*value) - reward_max;
        end
        [td_min, k1] = min(abs(td));
        [v_reward_max, k2] = max(reward);
        td_update = td(k2);
        noise_update = noise(k2);
        for j=1:rule_fire_count
                 j1 = rules_fired(j);
                 %deltaw = beta*sign(td*noise)*phi_norm(j);
                deltaw = beta*(td_update*noise_update)*phi_norm(j1);
                w(j1) = w(j1)+ deltaw;
                parameters_no_capture(j) = parameters_no_capture(j) + deltaw;
        end
        robot.no_capture_w = w;
        robot.rules_fired(rule_set_number).parameters_no_capture = parameters_no_capture;
    end
    %******
    if( go_to_update == 1)
          td_change = abs(td_old) - abs(td_update);
          robot.rules_fired(count31).td_actor = td_update;
          % Compute td average.
          td_avg_new = td_avg_actor + (td_update - td_avg_actor)/number_of_times_fired;
          robot.rules_fired(count31).td_avg_actor = td_avg_new;
          robot.rules_fired(count31).td_actor = td_update;
          % Compute td standard deviation.
           td_sigma_actor = td_sigma_actor + (td_update - td_avg_new)*(td_update - td_avg_actor);
           robot.rules_fired(count31).td_sigma_actor = td_sigma_actor;
           %
           % Compute the Z - statistic for hyothesis testing
           %
           if (td_sigma_actor > 0)
                zed = sqrt(number_of_times_fired)*td_avg_new/td_sigma_actor;
                robot.rules_fired(count31).zed_actor = zed;
           end
     end
    %*****
end


