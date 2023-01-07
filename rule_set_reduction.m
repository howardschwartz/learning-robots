function [robot] = rule_set_reduction(robot)
%
% Call this function with robot(i).capture(k) 
% This function will reduce the number of rule sets that will be used with this 
% pursuer evader relationship.
%
     count = robot.number_of_rules_fired;
     count2  = 0;
     for i = 1:count
         if (robot.rules_fired(i).number > 100)
             count2 = count2 + 1;
             saved_rule_set(count2) = robot.rules_fired(i);
%              saved_rule_set(count2).rules_fired = robot.rules_fired(i).rules_fired;
%              saved_rule_set(count2).number = robot.rules_fired(i).number;
%              saved_rule_set(count2).phi_norm = robot.rules_fired(i).phi_norm;
%              saved_rule_set(count2).reward_max_cap = robot.rules_fired(i).reward_max_cap;
%              saved_rule_set(count2).reward_max_cap_old = robot.rules_fired(i).reward_max_cap_old;
%              saved_rule_set(count2).reward_max_no_cap = robot.rules_fired(i).reward_max_no_cap;
%              saved_rule_set(count2).reward_max_no_cap_old = robot.rules_fired(i).reward_max_no_cap_old;
%              saved_rule_set(count2).condition = robot.rules_fired(i).condition;
%              saved_rule_set(count2).td = robot.rules_fired(i).td;
%              saved_rule_set(count2).td_avg = robot.rules_fired(i).td_avg;
%              saved_rule_set(count2).td_sigma = robot.rules_fired(i).td_sigma;
%              saved_rule_set(count2).td_actor = robot.rules_fired(i).td_actor;
%              saved_rule_set(count2).td_sigma_actor = robot.rules_fired(i).td_sigma_actor;
%              saved_rule_set(count2).td_converged = robot.rules_fired(i).td_converged;
%              saved_rule_set(count2).parameters_capture = robot.rules_fired(i).parameters_capture;
%              saved_rule_set(count2).parameters_no_capture = robot.rules_fired(i).parameters_no_capture;
%              saved_rule_set(count2).parameters_capture = robot.rules_fired(i).psi_parameters_capture;
%              saved_rule_set(count2).parameters_no_capture = robot.rules_fired(i).psi_parameters_no_capture;
%              saved_rule_set(count2).parameters_capture = robot.rules_fired(i).parameters_capture;
%              saved_rule_set(count2).zed = robot.rules_fired(i).zed;
         end
     end
     robot.number_of_rules_fired = count2;
     robot.rules_fired = saved_rule_set;
end

