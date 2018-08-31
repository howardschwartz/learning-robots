function [capture, up_des, delup] = capture_condition_virtual(robot1, robot2, no_of_v_robots)
%
% This function computes the capture condition of the virtual robots of robot 1 to robot 2.
% first make sure robot 1 is a pursuer and robot 2 is an evader otherwise
% set capture condition to 0 and return.
%
   if (robot1.type == 1 && robot2.type == 2)
      % compute capture condition
      %
      %  Compute initial angle from the virtual pursuer to he true evader
      %
      capture = zeros(1, no_of_v_robots);
      for i = 1:no_of_v_robots
         dx = robot2.x - robot1.virtualx(i);
         dy = robot2.y - robot1.virtualy(i);
         upe = atan2(dy, dx); % LOS from pursuer  to evader.
         uep = atan2(-dy, -dx);
         %sprintf(' The los of pursuer to evader is %f ', upe)
         %sprintf(' The los of evader to pursuer is %f ', uep)
         beta = uep - robot2.heading;
         capture(i) = (robot2.speed/robot1.speed)*sin(beta);
         %sprintf(' The capture value is %f ', capture)
         %sprintf(' The beta value is %f ', beta)
         if ( abs(capture(i)) < 1 && abs(beta) < pi/2) 
           alpha = asin(capture(i));
           %sprintf(' The alpha value is %f ', alpha)
           up_des(i) = upe + alpha;
           %sprintf(' The desired heading value is %f ', up_des)
           delup(i) = up_des(i) - robot1.vheading(i);
           %sprintf(' The error in heading is %f ', delup)
           capture(i) = 1;
           %sprintf(' The capture condition is true ')
         else
           capture(i) = 0;
           %up_des(i) = 0;
           %delup(i) = 0;
           up_des(i) = robot2.heading;
           delup(i) = up_des(i) - robot1.vheading(i);
         end
      end % end the for each virtual robot loop
   else
      capture(i) = 0;
      up_des(i) = 0;
      delup(i) = 0;
   end
end

