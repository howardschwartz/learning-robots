function [rule_set_number, number, condition_old, rule_found] = search_for_rule_set(robot, rules_fired)
% [robot, rule_set_number] = search_for_rule_set(robot)
%  This function searched for the rule set or creates a new rule set
%
%
     rule_found = 0;
     rule_set_number = 0;
     number = 0;
     condition_old = robot.condition;
     count3 = robot.number_of_rules_fired;
%      if (count3 == 1) %this is the first rule fired
%          robot.rules_fired(1).rules_fired = zero(1, ;
%      end
     if (count3 > 0)
         for count31 = 1:count3
             % Has the current rule set been fired yet?
             %
             if(rules_fired == robot.rules_fired(count31).rules_fired)
                  number = robot.rules_fired(count31).number + 1;
                  condition_old = robot.condition;
                  rule_set_number = count31;
                  rule_found = 1;
                  break
             end
         end
     end
end

