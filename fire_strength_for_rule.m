function wl = fire_strength_for_rule(sinput, rule)
% w_l = w_l = fire_strength_for_rule(sinput, rule) where sinput
% is the input to the rule. the input is an array and the rule 
% is a matrix of membership funtions with number of rows equal to
% the number of inputs.
%
   [n,m] = size(sinput); % m is the number of inputs
   [l,n3] = size(rule); % l is the number of membership functions in the rule.
%
   if (l ~= m || n3 ~= 3)
       print('invalid parameters')
       return
   end
%
   wlrule = zeros(m);
   for i=1:m
       wlrule(i) = memgradetriangle(sinput(i), rule(i, :));
   end
% use the product rule
   wl_prod = prod(wlrule);
   wl = wl_prod(1);
%
end


function phi = memgradetriangle(x, mf)
   % Defines the membership grade of state x in MF
    a = mf(1);
    b = mf(2);
    c = mf(3);
    phi = 0; 
    if(x < a)
       phi = 0;
    end
    if(x >= a && x < b)
       phi = (x - a)/(b - a);
    end
    if(x >= b && x < c)
       phi = (c - x)/(c - b);
    end
    if(x >= c)
       phi = 0;
    end
end