function [psi] = compute_critic_update( robot, no_of_robots)
% [psi] = compute_critic_update( robot )
%   Compute the update to the critic parameters
%
   for i = 1:no_of_robots
       reward = robot(i).reward_capture_heading;
       gamma = robot(i).gamma_capture_heading;
       alpha = robot(i).alpha;
       value = robot(i).value;
       value_old = robot(i).value_old;
       psi = robot(i).psi;
       phi_norm =robot(i).phi_norm_critic;
       no_of_rules = robot(i).no_of_rules_critic;
       td = (reward + gamma*value) - value_old;
       for j=1:no_of_rules
           psi(j) = psi(j)+ alpha*td*phi_norm(j);
           %sprintf(' Value psi(%d) is %f', j, psi(j))
       end
   end
end

