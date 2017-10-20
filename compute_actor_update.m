function [robot] = compute_actor_update(robot, noise)
% [w] = compute_actor_update(robot, no_of_robots)
%  compute the update to the actor parameters
%
    beta = robot.beta;
    value = robot.value;
    value_old = robot.value_old;
    phi_norm =robot.phi_norm_actor;
    no_of_rules = robot.no_of_rules_actor;
    if (robot.condition == 1)% If it can capture
        reward = robot.reward_capture_heading;
        gamma = robot.gamma_capture_heading;
        w  = robot.w;
        td = (reward + gamma*value) - value_old;
        for j=1:no_of_rules
           deltaw = beta*sign(td*noise)*phi_norm(j);
           w(j) = w(j)+ deltaw;
        end
        robot.w = w;
    end
    if (robot.condition == 0)% If it cannot capture
        reward = robot.reward_rel_vel;
        gamma = robot.gamma_rel_vel;
        w = robot.no_capture_w;
        td = (reward + gamma*value) - value_old;
        for j=1:no_of_rules
           deltaw = beta*sign(td*noise)*phi_norm(j);
           w(j) = w(j)+ deltaw;
        end
        robot.no_capture_w = w;
    end
end

