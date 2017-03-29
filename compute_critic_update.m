function [psi] = compute_critic_update(robot)
% [psi] = compute_critic_update( robot )
%   Compute the update to the critic parameters
%
    reward = robot.reward_capture_heading;
    gamma = robot.gamma_capture_heading;
    alpha = robot.alpha;
    value = robot.value;
    value_old = robot.value_old;
    psi = robot.psi;
    phi_norm =robot.phi_norm_critic;
    no_of_rules = robot.no_of_rules_critic;
    td = (reward + gamma*value) - value_old;
    for j=1:no_of_rules
        psi(j) = psi(j)+ alpha*td*phi_norm(j);
        %sprintf(' Value psi(%d) is %f', j, psi(j))
    end
end

