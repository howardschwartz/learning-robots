function [rule, no_of_rules] = init_mf_rules()
% [rules, no_of_rules] = init_mf_rules(). This function initializes the
% membership functions and the rules.
%   Detailed explanation goes here
% This is the test function for creating the memebership functions
% Define the input states first
%
  no_of_inputs = 2; % define the number of inputs
  input(1).number = 3; % input 1 has two membership functions
  input(2).number = 3; % input 2 has 3 membership functions
 % input(3).number = 3; % input 3 has 2 membership functions
%
  input(1).range = [-10, 10];
  input(2).range = [-10, 10];
 % input(3).range = [0, 2*pi];
%
% From here everything should be done automatically
%
    for i=1:no_of_inputs
        input(i).mf = define_mf_triangle(input(i).range, input(i).number);
    end
    x = input
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
    wl = zeros(1, no_of_rules);
    psil = zeros(1, no_of_rules);
    % create the rules. rules have as many rows as inputs
    % There is something wrong here
    %
    %rule_index(1).index(1)
    %nput(1).mf
    %input(2).mf
    for i=1:no_of_rules
        for j=1:no_of_inputs
          rule(i).mf(j, :) = input(j).mf(rule_index(i).index(j), :);
        end
    end
    %

end

