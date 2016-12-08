function [capture, alpha, up_des, delup] = capture_condition(robot1, robot2)
%
% This function computes the capture condition of robot 1 to robot 2.
% first make sure robot 1 is a pursuer and robot 2 is an evader otherwise
% set capture condition to 0 and return.
%
   if (robot1.type == 1 && robot2.type == 2)
      % compute capture condition
      %
      %  Compute initial angle from pursuer 1 to evader
      %
      dx = robot2.x - robot1.x;
      dy = robot2.y - robot1.y;
      upe = atan2(dy, dx); % LOS from pursuer  to evader.
      uep = atan2(-dy, -dx);
      sprintf(' The los of pursuer to evader is %f ', upe)
      sprintf(' The los of evader to pursuer is %f ', uep)
      beta = uep - robot2.heading;
      capture = (robot2.speed/robot1.speed)*sin(beta);
      sprintf(' The capture value is %f ', capture)
      sprintf(' The beta value is %f ', beta)
      if ( abs(capture) < 1 && abs(beta) < pi/2) 
        alpha = asin(capture);
        sprintf(' The alpha value is %f ', alpha)
        up_des = upe + alpha;
        sprintf(' The desired heading value is %f ', up_des)
        delup = up_des - robot1.heading;
        capture = 1;
        sprintf(' The capture condition is true ')
      else
        capture = 0;
        alpha = 0;
        up_des = 0;
        delup = 0;
      end
   else
      capture = 0;
      alpha = 0;
      up_des = 0;
      delup = 0;
   end
end
