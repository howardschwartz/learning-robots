function [w] = compute_actor_update(robot, no_of_robots)
% [w] = compute_actor_update(robot, no_of_robots)
%  compute the update to the actor parameters
%
    reward = robot.reward_capture_heading;
    gamma = robot.gamma_capture_heading;
    beta = robot.beta;
    value = robot.value;
    value_old = robot.value_old;
    w = robot.w;
    noise = robot.noise;
    phi_norm =robot.phi_norm_actor;
    no_of_rules = robot.no_of_rules_actor;
    td = (reward + gamma*value) - value_old;
    for j=1:no_of_rules
       deltaw = beta*sign(td*noise)*phi_norm(j);
       w(j) = w(j)+ deltaw;
    end
end

