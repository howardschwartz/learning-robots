function [phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_rules_fired_setv81(robot, input, input_range, max_rules_fired)
% Determine the rules that fired [phi_norm, not_zero_phi, rule_fire_count, rules_fired] = compute_rules_fired_set(robot, input, input_range)
%   Detailed explanation goes here
%
    no_of_rules = robot.no_of_rules_critic;
    rule = robot.rule_critic;
    phi = zeros(1, no_of_rules);
    not_zero_count = 0;
    rules_fired = zeros(1,max_rules_fired);
    rule_fire_count = 0;
    current_rules_fired = robot.current_rules_fired;
    rule_set_number = robot.rule_set_number;
    if (robot.condition == 1)
      not_zero_phi = robot.capture_not_zero_phi;
    end
    if (robot.condition == 0)
      not_zero_phi = robot.no_capture_not_zero_phi;
    end
    %
    % Lets add code here to search first for the previous rules that fired.
    % I need last number of rules fired
    if (rule_set_number ~= 0)
       last_rules_fired  = robot.rules_fired(rule_set_number).rules_fired; %previous rules fired
       for i=1:max_rules_fired
           j = last_rules_fired(i);
           if( j == 0)
               break;
           end
           phi(j) = fire_strength_for_rule(input, rule(j).mf, input_range);
           if(phi(j) < -0.0001 || phi(j) > 0.0001)
              not_zero_count = not_zero_count + 1;
              not_zero_phi(j) = not_zero_phi(j) + 1;
              rule_fire_count = rule_fire_count + 1;
              rules_fired(rule_fire_count) = j;
          end
       end
    %
    % Have all rules been found?
      if (rule_fire_count ~= max_rules_fired)
        % No, lets search near the rules that have fired.
        for i = 1:rule_fire_count
            j_found = 0; % j has not been found
            j = rules_fired(i) - 1;
            % Is j in last_rules_fired?
            for i1 = 1:max_rules_fired
                if(j == rules_fired(i1))
                    j_found = 1; % j is in the old rule set
                end
            end
            if(j_found == 0) %j is not in the rule set
                % Check if this rule will fire
                if(j > 0)
                  phi(j) = fire_strength_for_rule(input, rule(j).mf, input_range);
                  if(phi(j) < -0.0001 || phi(j) > 0.0001)
                    not_zero_count = not_zero_count + 1;
                    not_zero_phi(j) = not_zero_phi(j) + 1;
                    rule_fire_count = rule_fire_count + 1;
                    rules_fired(rule_fire_count) = j;
                  end
                end
            end
            if (rule_fire_count == max_rules_fired)
                break
            end
        end
        %Now for a higher rule
        for i = 1:rule_fire_count
            j_found = 0; % j has not been found
            j = rules_fired(i) + 1;
            % Is j in last_rules_fired?
            for i1 = 1:max_rules_fired
                if(j == rules_fired(i1))
                    j_found = 1; % j is in the old rule set
                end
            end
            if(j_found == 0) %j is not in the rule set
                % Check if this rule will fire
                if ( j > 2 && j <= no_of_rules)
                phi(j) = fire_strength_for_rule(input, rule(j).mf, input_range);
                  if(phi(j) < -0.0001 || phi(j) > 0.0001)
                     not_zero_count = not_zero_count + 1;
                     not_zero_phi(j) = not_zero_phi(j) + 1;
                     rule_fire_count = rule_fire_count + 1;
                     rules_fired(rule_fire_count) = j;
                  end
                end
            end
            if (rule_fire_count == max_rules_fired)
                break
            end
        end
      end
    end
    %
    %  End new search code
    %
    if (rule_fire_count ~= max_rules_fired)
      for i=1:no_of_rules
          %
          % Is "i" already in rule set, then skip
          %
          rule_found = 0;
          for i1 = 1:rule_fire_count
              if (i == rules_fired(i1))
                  rule_found = 1;
              end
          end
          %
          if (rule_found == 0)
              phi(i) = fire_strength_for_rule(input, rule(i).mf, input_range);
                 if(phi(i) < -0.0001 || phi(i) > 0.0001)
                    not_zero_count = not_zero_count + 1;
                    not_zero_phi(i) = not_zero_phi(i) + 1;
                    rule_fire_count = rule_fire_count + 1;
                    rules_fired(rule_fire_count) = i;
                       if (rule_fire_count == max_rules_fired)
                           break
                       end
                 end
          end
      end
    end
    if (rule_fire_count < max_rules_fired)
        rules_fired1 = rules_fired(1:rule_fire_count);
        rules_fired1 = sort(rules_fired1);
        rules_fired = [rules_fired1, zeros(1,(max_rules_fired - rule_fire_count))];
    end
    if (rule_fire_count == max_rules_fired)
        rules_fired = sort(rules_fired);
    end
    phi_norm = phi;
end

