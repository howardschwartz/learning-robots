function [capture, alpha, up_des, delup] = capture_condition(robot1, robot2)
%
% This function computes the capture condition of robot 1 to robot 2.
% first make sure robot 1 is anpursuer and robot 2 is an evader otherwise
% set capture condition to 0 and return.
%
   if (robot1.type == 1 && robot2.type == 0)
      % compute capture condition
      %
      %  Compute initial angle from pursuer 1 to evader
      %
      dx = robot2.x - robot1.x;
      dy = robot2.y - robot1.y;
      upe = atan2(dx, dy); % LOS from pursuer  to evader.
      beta = uep - robot2.heading;
      capture = (robot2.speed/robot1.speed)*sin(beta);
      if ( abs(capture) < 1 && abs(beta) < pi/2) 
        alpha = asin(capture);
        up_des = upe + alpha;
        delup = up_des - robot1.heading;
        capture = 1;
        %sprintf(' The capture condition is true the epoch is %d ', j)
      else
        capture = 0;
      end
   else
      capture = 0;
      alpha = 0;
      up_des = 0;
      delup = 0;
   end
end
