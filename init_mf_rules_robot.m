function [rule, no_of_rules] = init_mf_rules_robot(mf_data)
% [rules, no_of_rules] = init_mf_rules(). This function initializes the
% membership functions and the rules.
%   Detailed explanation goes here
% This is the test function for creating the memebership functions
% Define the input states first
%
  no_of_inputs = mf_data.no_of_inputs; % define the number of inputs
  for i = 1:no_of_inputs
     input(i).number = mf_data.mf_per_input(i).no_of_mf; % No. MF for input i.
     input(i).range = mf_data.mf_per_input(i).range; % range(i) should be a vector with min and max range
  end
%
% From here everything should be done automatically
%
    for i=1:no_of_inputs
        input(i).mf = define_mf_triangle(input(i).range, input(i).number);
    end
    %x = input
    %input(1).mf
    %
    % create the no_mf_per_input
    %
    for i=1:no_of_inputs
        no_mf_per_input(i) = input(i).number;
    end
    %
    % create the rule index
    %
    [rule_index, no_of_rules] = create_rule_index(no_mf_per_input);
    no_of_rules = no_of_rules - 1;
    %
    % create the rules. rules have as many rows as inputs
    %
    for i=1:no_of_rules
        for j=1:no_of_inputs
          rule(i).mf(j, :) = input(j).mf(rule_index(i).index(j), :);
        end
    end
    %

end

