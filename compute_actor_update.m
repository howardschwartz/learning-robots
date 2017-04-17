function [w] = compute_actor_update(robot_capture, noise)
% [w] = compute_actor_update(robot, no_of_robots)
%  compute the update to the actor parameters
%
    reward = robot_capture.reward_capture_heading;
    gamma = robot_capture.gamma_capture_heading;
    beta = robot_capture.beta;
    value = robot_capture.value;
    value_old = robot_capture.value_old;
    w = robot_capture.w;
    noise = robot_capture.noise;
    phi_norm =robot_capture.phi_norm_actor;
    no_of_rules = robot_capture.no_of_rules_actor;
    td = (reward + gamma*value) - value_old;
    for j=1:no_of_rules
       deltaw = beta*sign(td*noise)*phi_norm(j);
       w(j) = w(j)+ deltaw;
    end
end

