function rules = define_rules(no_mf_per_input, mf_per_input)
% this function outputs an array of all the possible rules.
% no_mf_per_input is the number of membership functions for each input.
% mf_per_input is the list of all membership functions for each input.
   [n,m] = size(no_mf_per_input);
%
   [rule_index, no_of_rules] = create_rule_index(no_mf_per_input);
%
% Let us put all the rules together
%
   for l=1:no_of_rules
%  Start the for each input loop
       for i=1:m
          rule(l).mf(i,:) = mf_per_input(i).mf(1,:)
       end
   end % end for each rule


end
