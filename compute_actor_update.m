function [w] = compute_actor_update(robot, no_of_robots)
% [w] = compute_actor_update(robot, no_of_robots)
%  compute the update to the actor parameters
%
   for i = 1:no_of_robots
       reward = robot(i).reward_capture_heading;
       gamma = robot(i).gamma_capture_heading;
       beta = robot(i).beta;
       value = robot(i).value;
       value_old = robot(i).value_old;
       w = robot(i).w;
       noise = robot(i).noise;
       phi_norm =robot(i).phi_norm_actor;
       no_of_rules = robot(i).no_of_rules_actor;
       td = (reward + gamma*value) - value_old;
       for j=1:no_of_rules
         deltaw = beta*sign(td*noise)*phi_norm(j);
         w(j) = w(j)+ deltaw;
       end
   end
end

