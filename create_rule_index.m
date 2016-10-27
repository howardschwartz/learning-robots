function [final_Rule_Base, final_Rule_Index] = create_rule_index(no_mf_per_input)
    % this function rule_index = create_rule_index(no_mf_per_input)
    % creates a rule index that defines the membership function for each input
    % to define the rules. rule_index(i).index = [1 3 2 ... 5}
    %


    % m is the number of inputs
    % n is not important
    [n, m ] = size(no_mf_per_input);

    global no_inputs
    global input_Index
    global rule
    global rule_Index 

    no_inputs = m;
    input_Index = 0;
    rule = ones(1, m);
    rule_Index = 0;
    
    
    [final_Rule_Base, final_Rule_Index] = recursive_rule_generator(no_mf_per_input);
    
    
    % For this new code to integrate into the previous code, I observed
    % that in the file "init_mf_rules.m" you did the following in
    
    % line 38         no_of_rules = no_of_rules - 1; 
    
    % So in order to correct that, I will just simply add a one to the
    % output, so that it gets substracted in the file "init_mf_rules.m"
    final_Rule_Index = final_Rule_Index + 1;
    
    
end% end the function


function [final_Rule_Base, final_Rule_Index] = recursive_rule_generator(no_mf_per_input)
    
    global no_inputs
    global input_Index
    global rule
    global rule_Index
    global rule_Base
   
    input_Index = input_Index + 1;
    
    
    for index = 1:no_mf_per_input(input_Index)     
        
        
        rule(input_Index) = index;
        
        
        if input_Index == no_inputs
            
            rule_Index = rule_Index + 1;
            
            rule_Base(rule_Index).index = rule;
            
        else
            
            recursive_rule_generator(no_mf_per_input);
            
        end 
        
    end
    
    
    input_Index = input_Index - 1;
    
    
    final_Rule_Index = rule_Index;
    final_Rule_Base = rule_Base;
   
end% end the function