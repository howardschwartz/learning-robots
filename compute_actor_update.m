function [robot_capture] = compute_actor_update(robot_capture, noise)
% [w] = compute_actor_update(robot, no_of_robots)
%  compute the update to the actor parameters
%
    beta = robot_capture.beta;
    value = robot_capture.value;
    value_old = robot_capture.value_old;
    phi_norm =robot_capture.phi_norm_actor;
    no_of_rules = robot_capture.no_of_rules_actor;
    if (robot_capture.condition == 1)% If it can capture
        reward = robot_capture.reward_capture_heading;
        gamma = robot_capture.gamma_capture_heading;
        w  = robot_capture.w;
        td = (reward + gamma*value) - value_old;
        for j=1:no_of_rules
           deltaw = beta*sign(td*noise)*phi_norm(j);
           w(j) = w(j)+ deltaw;
        end
        robot_capture.w = w;
    end
    if (robot_capture.condition == 0)% If it cannot capture
        reward = robot_capture.reward_rel_vel;
        gamma = robot_capture.gamma_rel_vel;
        w = robot_capture.no_capture_w;
        td = (reward + gamma*value) - value_old;
        for j=1:no_of_rules
           deltaw = beta*sign(td*noise)*phi_norm(j);
           w(j) = w(j)+ deltaw;
        end
        robot_capture.no_capture_w = w;
    end
end

