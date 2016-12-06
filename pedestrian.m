function [y_prime] = pedestrian(t, y)
% [y_prime] = pedestrian(t, y)
%  Dynamic model of a pedestrian
%
   global u; % Heading angle
   global v; % speed
   y_prime = zeros(2,1);
   y_prime(1) = v*cos(u);
   y_prime(2) = v*sin(u);
end

