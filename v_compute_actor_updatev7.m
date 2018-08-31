function [robot] = v_compute_actor_updatev7(robot, noise, no_update, no_of_v_robots)
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
    if (robot.condition == 1)% If it can capture
        reward = robot.v_reward_capture_heading;
        reward_no_noise =robot.reward_capture_heading;
        gamma = robot.gamma_capture_heading;
        td_no_noise = (reward_no_noise  + gamma*value) - value_old;
        w  = robot.w;
        % find the smallest td
        %
        for k=1:no_of_v_robots
            value = robot.v_value(k);
            td(k) = (reward(k) + gamma*value) - value_old;
        end
        [td_min, k1] = min(abs(td));
        if (abs(td_no_noise) <= abs(td_min))
            td_update = td_no_noise;
            noise_update = 0;
        else
            td_update = td_min;
            noise_update = noise(k1);
        end
%         for k=1:no_of_v_robots
%             if (no_update == 0)
%               value = robot.v_value(k);
%               td = (reward(k) + gamma*value) - value_old;
%               td_change = abs(td_old) - abs(td);
%               if(td_change < -0.5 && number_of_times_fired > 20)
%                   sprintf(' The value of td_change is %f ', td_change) 
%               end
              for j=1:rule_fire_count
                 j1 = rules_fired(j);
                 deltaw = beta*(td_update*noise_update)*phi_norm(j1);
                 w(j1) = w(j1)+ deltaw;
              end
%             end
%         end
        robot.w = w;
    end
    if (robot.condition == 0)% If it cannot capture
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
        end
        [td_min, k1] = min(abs(td));
        if (abs(td_no_noise) <= abs(td_min))
            td_update = td_no_noise;
            noise_update = 0;
        else
            td_update = td_min;
            noise_update = noise(k1);
        end
        for j=1:rule_fire_count
                 j1 = rules_fired(j);
                 %deltaw = beta*sign(td*noise)*phi_norm(j);
                deltaw = beta*(td_update*noise_update)*phi_norm(j1);
                w(j1) = w(j1)+ deltaw;
        end
        robot.no_capture_w = w;
    end
end


