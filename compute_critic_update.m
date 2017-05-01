function [psi] = compute_critic_update(robot)
% [psi] = compute_critic_update( robot )
%   Compute the update to the critic parameters
%
    if (robot.condition == 1)% If it can capture
        reward = robot.reward_capture_heading;
        gamma = robot.gamma_capture_heading;
    end
     if (robot.condition == 0)% If it cannot capture
        reward = robot.reward_rel_vel;
        gamma = robot.gamma_rel_vel;
    end
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

