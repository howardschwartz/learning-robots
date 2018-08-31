function [robot] = compute_actor_updatev6(robot, noise, not_zero_count)
% [w] = compute_actor_update(robot, no_of_robots)
%  compute the update to the actor parameters
%
    beta = robot.beta;
    value = robot.value;
    value_old = robot.value_old;
    phi_norm =robot.phi_norm_actor;
    rules_fired = robot.current_rules_fired;
    rule_fire_count = robot.rule_fire_count;
    no_of_rules = robot.no_of_rules_actor;
    if (robot.condition == 1)% If it can capture
        reward = robot.reward_capture_heading;
        gamma = robot.gamma_capture_heading;
        w  = robot.w;
        td = (reward + gamma*value) - value_old;
        for j=1:rule_fire_count
           %deltaw = beta*sign(td*noise)*phi_norm(j);
           j1 = rules_fired(j);
           deltaw = beta*(td*noise)*phi_norm(j1);
           w(j1) = w(j1)+ deltaw;
        end
        robot.w = w;
    end
    if (robot.condition == 0)% If it cannot capture
        reward = robot.reward_rel_vel;
       % reward = robot.reward_heading_difference;
        gamma = robot.gamma_rel_vel;
        w = robot.no_capture_w;
        %w = robot.w;
        td = (reward + gamma*value) - value_old;
        for j=1:rule_fire_count
            j1 = rules_fired(j);
           %deltaw = beta*sign(td*noise)*phi_norm(j);
           deltaw = beta*(td*noise)*phi_norm(j1);
           w(j1) = w(j1)+ deltaw;
        end
        %robot.w = w;
        robot.no_capture_w = w;
    end
end

