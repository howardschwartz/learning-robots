function [k1, k2, k3, k4, k5, k6, k7, k8, k9, w1, w2, w3, w4, w5, w6, w7, w8, w9, psil2_success, psil2_success_init, wl2_success, wl2_success_init, sigma_plot, sigma_success_plot, count_success_times, reward2_plot] = agent_sim3pv8(speed, angle)
%speed = 1.;
%angle = pi/3;
%
% This function simulates the differential equation for an agent.
% In this case the agent is a pedestrian similar to the Homcidal Chauffer
%
global up1; % Heading angle pursuer 1
global up2; % Heading angle pursuer 2
global up3; % Heading angle of pursuer 3
global ue;  % heading angle evader
global vp1; % pursuer 1 speed
global vp2; % pursuer 2 speed
global vp3; % pursuer 3 speed
global ve;  % evader speed
%
count = 0;
count2= 0;
count3 = 0;
countm = 0;
count_success_times = 0;
ue = angle;
ve = speed;
% *******************************************************************
% I made only 3 major changes to this file. I made no changes in the 
% the other files. 
%
% The 1st change: I declared the initial values for the learning factors.
% The 2nd change: I modified the code so that the actor and the critic 
%                 can learn simultaneously.
% The 3rd change: I saved all the game plots in a single folder in
%                 the current directory. This is done to increase the ease
%                 of comparing the outcome of several simulation results.
% The 4th change: I added additional code to decay the learning factors
% *******************************************************************
% *******************************************************************
% 1st change
% 
% Since we are going to be updating the learning factors as a function
% of their initial values. We need to declare the initial values 
% seperately so that they will not change after every epoch.
% *******************************************************************
%
%
% I used a small gamma of 0.5 to make the agent focus on maximizing 
% the instantaneous rewards
%
gamma_init = 0.0;
sigma_init = 1;
mu = 0;
alpha_init = 0.1;
beta_init = 0.05;
% Assign the initial values of the learning factors to new variables that will be
% updated after each epoch. The initalize values set above does not change.
sigma = sigma_init;
gamma = gamma_init;
alpha = alpha_init;
beta = beta_init;
uexplore = 0;
% *******************************************************************
% Initialize pursuer and evader positions
%
% pursuer 1
ya(1) = -5; %initialize pursuer 1 position x
ya(2) = 5; %initialize pursuer 1 position y
%
%pursuer 2
ya(3) = 10; % initialize pursuer 2 x position
ya(4) = 10; % Initialize Pursuer 2 y position
%
%pursuer 3
ya(5) = 0; % initialize pursuer 3 x position
ya(6) = -5; % Initialize Pursuer 3 y position
% evader
ya(7) = 0.0; % initialize evader postion x
ya(8) = 0.0; % initialize evader position y
%
%  Compute initial angle from pursuer 1 to evader
%
dy1 = ya(8) - ya(2);
dy2 = ya(7) - ya(1);
up1e = atan2(dy1, dy2); % LOS from pursuer 1 to evader.
%
% Compute the quadrant
%
if (up1e >= 0 && up1e <= pi/2)
    quad1 = 1;
end
%
if (up1e > pi/2 && up1e <= pi)
    quad1 = 2;
end
%
if (up1e < -pi/2 && up1e > -pi)
    quad1 = 3;
end
%
if (up1e < 0 && up1e >= -pi/2)
    quad1 = 4;
end
%
if(up1e < 0)
    up1e  = up1e + 2*pi;
end
uep1 = atan2(-dy1, -dy2); % LOS from evader to pursuer 1.
if(uep1 < 0)
    uep1  = uep1 + 2*pi;
end
%
%  Compute initial angle from pursuer 2 to evader
%
dy1 = ya(8) - ya(4);
dy2 = ya(7) - ya(3);
up2e = atan2(dy1, dy2); % LOS from pursuer 2 to evader.
sprintf(' The up2e is %f ', up2e)
%
% Compute the quadrant
%
if (up2e >= 0 && up2e <= pi/2)
    quad2 = 1;
end
%
if (up2e > pi/2 && up2e <= pi)
    quad2 = 2;
end
%
if (up2e < -pi/2 && up2e > -pi)
    quad2 = 3;
end
%
if (up2e < 0 && up2e >= -pi/2)
    quad2 = 4;
end
sprintf(' The quadrant is %d ', quad2)
%
if( up2e < 0)
    up2e  = up2e + 2*pi;
end
uep2 = atan2(-dy1, -dy2); % LOS from evader to pursuer 2.
if(uep2 < 0)
    uep2  = uep2 + 2*pi;
end
%
%  Compute initial angle from pursuer 3 to evader
%
dy1 = ya(8) - ya(6);
dy2 = ya(7) - ya(5);
up3e = atan2(dy1, dy2); % LOS from pursuer 3 to evader.
%
% Compute the quadrant
%
if (up3e >= 0 && up3e <= pi/2)
    quad3 = 1;
end
%
if (up3e > pi/2 && up3e <= pi)
    quad3 = 2;
end
%
if (up3e < -pi/2 && up3e > -pi)
    quad3 = 3;
end
%
if (up3e < 0 && up3e >= -pi/2)
    quad3 = 4;
end
%
if( up3e < 0)
    up3e  = up3e + 2*pi;
end
uep3 = atan2(-dy1, -dy2); % LOS from evader to pursuer 3.
if(uep3 < 0)
    uep3  = uep3 + 2*pi;
end
%
% define evader direction
%
%ue = 0.5*pi;
beta1 = uep1 - ue;
if(beta1 > pi)
    beta1 = beta1 - 2*pi;
end
beta2 = uep2 - ue;
if(beta2 > pi)
    beta2 = beta2 - 2*pi;
end
beta3 = uep3 - ue;
if(beta3 > pi)
    beta3 = beta3 - 2*pi;
end
% initialize the evader speed and the pursuer speed.
vp1 = 1.0; % pursuer 1 speed
vp2 = 1.0; % pursuer 2 speed
vp3 = 1.0; % pursuer 3 speed
%ve = 1.4;  % evader speed
%
% The fastest Approach is
%
v_max = vp2 + ve;
dist_max = 0.1*v_max;
%
% can the pursuer  1 catch the evader?
%
capturecond1 = (ve/vp1)*sin(abs(beta1));
if(capturecond1 < 1 && abs(beta1) < pi/2)
    alpha1 = asin(capturecond1);
    if ( quad1 == 1 && beta1 < 0)
        up1 = up1e - alpha1;
    end
    if ( quad1 == 1 && beta1 >= 0)
        up1 = alpha1 + up1e;
    end
    if ( quad1 == 2 && beta1 <0)
        up1 = up1e - alpha1;
    end
    if ( quad1 == 2 && beta1 >= 0)
        up1 = up1e+ alpha1;
    end
    if ( quad1 == 3 && beta1 <0)
        up1 = up1e - alpha1;
    end
    if ( quad1 == 3 && beta1 >= 0)
        up1 = up1e + alpha1;
    end
    if ( quad1 == 4 && beta1 <0)
        up1 = up1e - alpha1;
    end
    if ( quad1 == 4 && beta1 >= 0)
        up1 = up1e + alpha1;
    end
    cond1 = 1; % pursuer 1 can capture
else
    sprintf('%s','pursuer 1 cannot capture')
    cond1 = -1; % pursuer 1 cannot capture
    up1 = ue; % follow the evader
end
%
% Can pursuer 2 catch the evader?
%
capturecond2 = (ve/vp2)*sin(abs(beta2));
if(capturecond2 < 1 && abs(beta2) < pi/2)
    alpha2 = asin(capturecond2);
    sprintf(' The alpha2 is %f ', alpha2)
    if ( quad2 == 1 && beta2 < 0)
        up2 = up2e - alpha2;
    end
    if ( quad2 == 1 && beta2 >= 0)
        up2 = alpha2 - up2e;
    end
    if ( quad2 == 2 && beta2 <0)
        up2 = up2e - alpha2;
    end
    if ( quad2 == 2 && beta2 >= 0)
        up2 = up2e+ alpha2;
    end
    if ( quad2 == 3 && beta2 <0)
        up2 = up2e - alpha2;
    end
    if ( quad2 == 3 && beta2 >= 0)
        up2 = up2e + alpha2;
    end
    if ( quad2 == 4 && beta2 <0)
        up2 = up2e - alpha1;
    end
    if ( quad2 == 4 && beta2 >= 0)
        up2 = up2e + alpha2;
    end
    cond2 = 1; % pursuer 2 can capture
else
    sprintf('%s','pursuer 2 cannot capture')
    cond2 = -1; %pursuer 2 cannot capture
    up2= ue; %follow the evader
end
up2_opt = up2;
up2_old = up2;
sprintf(' The optimal control angle is %f ', up2_opt)
%
% Can pursuer 3 catch the evader?
%
capturecond3 = (ve/vp3)*sin(abs(beta3));
if(capturecond3 < 1 && abs(beta3) < pi/2)
    alpha3 = asin(capturecond3);
    if ( quad3 == 1 && beta3 < 0)
        up3 = up3e - alpha3;
    end
    if ( quad3 == 1 && beta3 >= 0)
        up3 = up3e + alpha3;
    end
    if ( quad3 == 2 && beta3 < 0)
        up3 = up3e - alpha3;
    end
    if ( quad3 == 2 && beta3 >= 0)
        up3 = up3e+ alpha3;
    end
    if ( quad3 == 3 && beta3 < 0)
        up3 = up3e - alpha3;
    end
    if ( quad3 == 3 && beta3 >= 0)
        up3 = up3e + alpha3;
    end
    if ( quad3 == 4 && beta3 < 0)
        up3 = up3e - alpha3;
    end
    if ( quad3 == 4 && beta3 >= 0)
        up3 = up3e + alpha3;
    end
    cond3 = 1; % pursuer 3 can capture
else
    sprintf('%s','pursuer 3 cannot capture')
    cond3 = -1; % pursuer 3 cannot capture.
    up3 = ue; % follow the evader
end
%
% Can either pursuer catch the evader
%
if ( capturecond1 >= 1 && capturecond2 >= 1 && capturecond2 >= 1)
    sprintf('%s', 'no one can capture')
    return
end
%
% Initialize the membership functions. 
%
[rule, no_of_rules] = init_mf_rules();
sprintf(' The number of rules is %d ', no_of_rules)
wl1 = zeros(1, no_of_rules);
wl2 = zeros(1, no_of_rules);
wl3 = zeros(1, no_of_rules);
% *******************************************************************
% 2nd change
%
% I commented this code out, so that the critic and the actor can 
% learn simultaneously
% *******************************************************************
% for i=1:no_of_rules
%    wl(i) = 4.492395;
% end
% *******************************************************************
psil1 = zeros(1, no_of_rules);
psil2 = zeros(1, no_of_rules);
psil3 = zeros(1, no_of_rules);
%
phi_norm = zeros(1, no_of_rules);
phi_norm1 = zeros(1, no_of_rules);
phi_norm2 = zeros(1, no_of_rules);
%
psil2_success = psil2;
wl2_success = wl2;
psil2_success_init = psil2;
wl2_success_init = wl2;
psil2_init = psil2;
wl2_init = wl2;
alpha_success = alpha;
beta_success = beta;
sigma_success = sigma;
capture_cond2_old = 0;
%
% start here%
%
game_no = 500;
count_success = 150;
sigma_plot = zeros(1,game_no);
sigma_success_plot = zeros(1, game_no);
for j=1:game_no
    % Initialize pursuer and evader positions
%
% pursuer 1
ya(1) = -5; %initialize pursuer 1 position x
ya(2) = 5; %initialize pursuer 1 position y
%
%pursuer 2
ya(3) = 10; % initialize pursuer 2 x position
ya(4) = 10; % Initialize Pursuer 2 y position
%
%pursuer 3
ya(5) = 0; % initialize pursuer 3 x position
ya(6) = -5; % Initialize Pursuer 3 y position
% evader
ya(7) = 0.0; % initialize evader postion x
ya(8) = 0.0; % initialize evader position y
%
% Position differences
%
dp1ext0 = ya(7) - ya(1);
dp1eyt0 = ya(8) - ya(2);
p1xt0 = ya(1);
p1yt0 = ya(2);
%
dp2ext0 = ya(7) - ya(3);
dp2eyt0 = ya(8) - ya(4);
p2xt0 = ya(3);
p2yt0 = ya(4);
%
dp3ext0 = ya(7) - ya(5);
dp3eyt0 = ya(8) - ya(6);
p3xt0 = ya(5);
p3yt0 = ya(6);
%
% Initialize minimum distances
%
distp2e1_min = sqrt(dp2ext0^2 + dp2eyt0^2);
%
ext0 = ya(7);
eyt0 = ya(8);
count = 0;
capture_fail = 0;
psil2_init = psil2;
wl2_init = wl2;
sigma_plot(j) = sigma;
sigma_success_plot(j) = sigma_success;
   game_on = 1; %start the game
   % *******************************************************************
   % 3rd change
   %
   % Initialize the figure that we make use of to plot the trajectories
   % of the agents.
   % *******************************************************************
   close all % Close all open figures
   gamePlot = figure('visible','off'); % Create new figure but don't display it
   axis([-10 25 -10 25]) % set the axis of the figure
   hold on % ensure continuos plot on the same figure
   grid on % turn on the grid lines
   % *******************************************************************
    while(game_on == 1)
    %
    %  Compute initial angle from each pursuer to the evader
    %  Compute The Manhattan distances
       p1xt0 = ya(1);
       p1yt0 = ya(2);
       %
       p2xt0 = ya(3);
       p2yt0 = ya(4);
       %
       p3xt0 = ya(5);
       p3yt0 = ya(6);
       %
       ext0 = ya(7);
       eyt0 = ya(8);
       %
       dp1ext0 = ya(7) - ya(1); % x - axis distance.
       dp1eyt0 = ya(8) - ya(2); % y - axis distance.
       up1et0 = atan2(dp1eyt0, dp1ext0); % LOS from pursuer 1 to evader.
       %
       dp2ext0 = ya(7) - ya(3); % x - axis distance.
       dp2eyt0 = ya(8) - ya(4); % y - axis distance.
       up2et0 = atan2(dp2eyt0, dp2ext0); % LOS from pursuer 2 to evader.
       %
       dp3ext0 = ya(7) - ya(5); % x - axis distance.
       dp3eyt0 = ya(8) - ya(6); % y - axis distance.
       up3et0 = atan2(dp3eyt0, dp3ext0); % LOS from pursuer 3 to evader. 
    %
    % Now that we have the rules lets deteremine the firing strenth for
    % each rule
    % Compute the euclidian distance
    %
       distp1e0 = sqrt(dp1ext0^2 + dp1eyt0^2);
       distp2e0 = sqrt(dp2ext0^2 + dp2eyt0^2);
       distp3e0 = sqrt(dp3ext0^2 + dp3eyt0^2);
     %
     % The inputs are the manhattan distances.
     %
     % ******************************************************************
      sinput1 = [dp1ext0, dp1eyt0];  % inputs to the controller
      phi = zeros(1, no_of_rules); 
      for i=1:no_of_rules
        phi(i) = fire_strength_for_rule(sinput1, rule(i).mf); % compute the firing strenght
      end
    %
    % Compute the normalized firing strength for each rule.
    %
    phi_sum = sum(phi);    
    phi1_norm = zeros(1, no_of_rules); 
    if (phi_sum ~= 0)
      for i=1:no_of_rules
        phi1_norm(i) = phi(i)/phi_sum;
      end
    end
    %
    % Compute the action for pursuer 1
    %
    u = 0;
    for i=1:no_of_rules
        u = u + phi1_norm(i)*wl1(i);
        %sprintf(' phi_norm(%d) is %f ', i, phi_norm(i))
    end
    up1 = u;
    %
    sinput2 = [dp2ext0, dp2eyt0];  % inputs to the controller
      phi = zeros(1, no_of_rules); 
      for i=1:no_of_rules
        phi(i) = fire_strength_for_rule(sinput2, rule(i).mf); % compute the firing strenght
      end
    %
    % Compute the normalized firing strength for each rule.
    %
    phi_sum = sum(phi);    
    phi2_norm = zeros(1, no_of_rules); 
    if (phi_sum ~= 0)
      for i=1:no_of_rules
        phi2_norm(i) = phi(i)/phi_sum;
      end
    end
    %
    % Compute the action for pursuer 2
    %
    u = 0;
    for i=1:no_of_rules
        u = u + phi2_norm(i)*wl2(i);
        %sprintf(' phi_norm(%d) is %f ', i, phi_norm(i))
    end
    up2 = u;
    %
    sinput3 = [dp3ext0, dp3eyt0];  % inputs to the controller
      phi = zeros(1, no_of_rules); 
      for i=1:no_of_rules
        phi(i) = fire_strength_for_rule(sinput3, rule(i).mf); % compute the firing strenght
      end
    %
    % Compute the normalized firing strength for each rule.
    %
    phi_sum = sum(phi);    
    phi3_norm = zeros(1, no_of_rules); 
    if (phi_sum ~= 0)
      for i=1:no_of_rules
        phi3_norm(i) = phi(i)/phi_sum;
      end
    end
    %
    % Compute the action for pursuer 1
    %
    u = 0;
    for i=1:no_of_rules
        u = u + phi3_norm(i)*wl3(i);
        %sprintf(' phi_norm(%d) is %f ', i, phi_norm(i))
    end
    up3 = u;
    % sprintf('The FLC control angle is %f ', up2) 
    %
    % Add in exploration
    %
    % I chose this method "normrnd(mu,sigma)" instead of "randn(1)" 
    % so that it is possible to decay sigma over time
    %
    noise1 = normrnd(mu,sigma);
    up1 = up1 + noise1;   
    noise2 = normrnd(mu,sigma);
    %sprintf(' The noise2 is %f ', noise2)
    up2 = up2 + noise2;
    noise3 = normrnd(mu,sigma);
    up3 = up3 + noise3;   
    % 
    % ensure that the computed action is between [0, 2pi]
    %
   % if up2 < 0  
   %     up2 = up2 + 2 * pi;
    %elseif up2 > 2 * pi
   %     up2 = up2 - 2 * pi;
    %end
    % *****************************************************************
    %
    % Compute the current value of the state
    %
    valp1 = 0;
    for i=1:no_of_rules
        valp1 = valp1 + phi1_norm(i)*psil1(i);
    end
    %
    valp2 = 0;
    for i=1:no_of_rules
        valp2 = valp2 + phi2_norm(i)*psil2(i);
    end
    %
    valp3 = 0;
    for i=1:no_of_rules
        valp3 = valp3 + phi3_norm(i)*psil3(i);
    end
    %
    %
    % Take the action.
    %
    [T, yout] = ode45(@dagent, [0 0.05 0.1], ya);
    ya = yout(3, :);   
    %
    % Compute the new Manhattan distance
    %
    p1xt1 = ya(1);
    p1yt1 = ya(2);
    dp1ext1 = ya(7) - ya(1); % x - axis distance.
    dp1eyt1 = ya(8) - ya(2); % y - axis distance.
    %
    p2xt1 = ya(3);
    p2yt1 = ya(4);
    
    dp2ext1 = ya(7) - ya(3); % x - axis distance.
    dp2eyt1 = ya(8) - ya(4); % y - axis distance.
    %
    p3xt1 = ya(5);
    p3yt1 = ya(6);
    dp3ext1 = ya(7) - ya(5); % x - axis distance.
    dp3eyt1 = ya(8) - ya(6); % y - axis distance.
    %
    ext1 = ya(7);
    eyt1 = ya(8);
    %
    % Compute the euclidian distance at time 2.
    %
    distp1e1 = sqrt(dp1ext1^2 + dp1eyt1^2);
    distp2e1 = sqrt(dp2ext1^2 + dp2eyt1^2);
    distp3e1 = sqrt(dp3ext1^2 + dp3eyt1^2);
    %
    % Let's computes the closest distance,
    %
    if (distp2e1 < distp2e1_min)
        distp2e1_min = distp2e1;
    end
    %
    %  Compute angle from pursuer to evader
    %
    up1et1 = atan2(dp1eyt1, dp1ext1); % LOS from pursuer 1 to evader.
    uep1t1 = atan2(-dp1eyt1, -dp1ext1); % LOS from evader to pursuer 1.
    dellos1 = up1et1 - up1et0;
    %
    %sprintf(' The dp2eyt1 is %f and dp2ext1 is %f ', dp2eyt1, dp2ext1)
    up2et1 = atan2(dp2eyt1, dp2ext1); % LOS from pursuer 2 to evader.
    uep2t1 = atan2(-dp2eyt1, -dp2ext1); % LOS from evader to pursuer 2.
    dellos2 = up2et1 - up2et0;
    %
    up3et1 = atan2(dp3eyt1, dp3ext1); % LOS from pursuer 3 to evader.
    uep3t1 = atan2(-dp3eyt1, -dp3ext1); % LOS from evade to pursuer 3.
    dellos3 = up3et1 - up3et0;
    %
    % Compute pursuer velocity heading
    %
    vp1xt1 = p1xt1 - p1xt0;
    vp1yt1 = p1yt1 - p1yt0;
    velp1t1 = sqrt(vp1xt1^2 + vp1yt1^2);
    up1t1 = atan2(vp1yt1, vp1xt1); %heading
    %
    vp2xt1 = p2xt1 - p2xt0;
    vp2yt1 = p2yt1 - p2yt0;
    velp2t1 = sqrt(vp2xt1^2 + vp2yt1^2);
    up2t1 = atan2(vp2yt1, vp2xt1); %heading
    %
    vp3xt1 = p3xt1 - p3xt0;
    vp3yt1 = p3yt1 - p3yt0;
    velp3t1 = sqrt(vp3xt1^2 + vp3yt1^2);
    up3t1 = atan2(vp3yt1, vp3xt1); %heading
    %
    % Compute evader velocity
    %
    vext1 = ext1 - ext0;
    veyt1 = eyt1 - eyt0;
    velet1 = sqrt(vext1^2 + veyt1^2);
    uet1 = atan2(veyt1, vext1); %heading
    %
    % Check capture conditions
    %
    betap1t1 = uep1t1 - uet1;
    betap2t1 = uep2t1 - uet1;
    betap3t1 = uep3t1 - uet1;
    %sprintf(' The betap2t1 is %f ', betap2t1)
    %
    capture1 = (velet1/velp1t1)*sin(betap1t1);
    capture2 = (velet1/velp2t1)*sin(betap2t1);
    capture3 = (velet1/velp3t1)*sin(betap3t1);
    %sprintf(' The capture2 is %f ', capture2)
    %
    %up2
    if ( abs(capture2) < 1) 
        alpha2t1 = asin(capture2);
    end
    up2_des = up2et1 + alpha2t1;
    delup2 = up2_des - up2;
    %sprintf(' The alpha2t1 is %f ', alpha2t1)
    %sprintf(' The up2et1 is %f ', up2et1)
    %sprintf(' The up2_des is %f ', up2_des)
    %sprintf(' The up2 is %f ', up2)
    if(abs(capture1) < 1)
        capture_cond1 = 1;
    else
        capture_cond1 = 0;
    end
    if(abs(capture2) < 1 && abs(betap2t1) < pi/2)
        capture_cond2 = 1;
        %sprintf(' The capture condition is true the epoch is %d ', j)
    else
        capture_cond2 = 0;
        capture_fail = 1;
        sprintf(' The capture condition is false the epoch is %d ', j)
         %
         % Did the capture Condition Change?
         %
         if (capture_cond2_old == 1 && sum(psil2_success_init) ~= 0)
         %    
            psil2 = psil2_success_init;
            wl2 = wl2_success_init;
            sigma = sigma_success*1.2;
            %sigma = sigma_init;
            sprintf(' The capture condition has changed no longer can capture epoch %d ', j)
         end
        % psil2 = psil2_success_init;
        % wl2 = wl2_success_init;
         %sigma = sigma*1.2;
         capture_cond2_old = 0;
         if (count_success_times > 0)
            psil2 = psil2_success_init;
            wl2 = wl2_success_init;
            sigma = sigma_success;
         end
    end
    %
    % Did the capture Condition Change?
    %
    if(abs(capture3) < 1)
        capture_cond3 = 1;
    else
        capture_cond3 = 0;
    end
    %
    % Compute the relative velocity
    %
    vp1ext1 = vext1 - vp1xt1;
    vp1eyt1 = veyt1 - vp1yt1;
    velp1et1 = sqrt(vp1ext1^2 + vp1eyt1^2);
    uvp1et1 = atan2(vp1eyt1, vp1ext1);
    %
    vp2ext1 = vext1 - vp2xt1;
    vp2eyt1 = veyt1 - vp2yt1;
    velp2et1 = sqrt(vp2ext1^2 + vp2eyt1^2);
    uvp2et1 = atan2(vp2eyt1, vp2ext1);
    %
    vp3ext1 = vext1 - vp3xt1;
    vp3eyt1 = veyt1 - vp3yt1;
    velp3et1 = sqrt(vp3ext1^2 + vp3eyt1^2);
    uvp3et1 = atan2(vp3eyt1, vp3ext1);
    % Compute the reward
    %
    % Let me create new rules
    %
    %error = up2_opt - up2;
%     sprintf(' The error is %f', error)
    %rewardp2 = 2*exp(-(error^2)) - 1;
    %rewardp1 = (distp1e0 - distp1e1)/dist_max;
    %rewardp3 = (distp3e0 - distp3e1)/dist_max;
    rewardp1 = -velp1et1;
    %rewardp2 = -velp2et1;
   % rewardp2 = (distp2e0 - distp2e1)/dist_max;
    rewardp2 = 2*exp(-(delup2^2)) - 1;
    %rewardp1 = 2*exp(-(dellos1^2/0.05)) - 1;
    %rewardp2 = 2*exp(-(dellos2^2/0.05)) - 1;
    %rewardp3 = 2*exp(-(dellos3^2/0.05)) - 1;
    rewardp3 = -velp3et1;
    reward = rewardp3;
    if (j == game_no)
      count3 = count3 + 1;
      reward_plot(count3) = distp2e1;
      reward1_plot(count3) = rewardp1;
      reward2_plot(count3) = rewardp2;
      reward3_plot(count3) = rewardp3;
      % los_plot(count3) = up2et0;
      los_plot(count3) = distp2e1_min;
      k1(count3) = psil2(1);
      k2(count3) = psil2(2);
      k3(count3) = psil2(3);
      k4(count3) = psil2(4);
      k5(count3) = psil2(5);
      k6(count3) = psil2(6);
      k7(count3) = psil2(7);
      k8(count3) = psil2(8);
      k9(count3) = psil2(9);
      %
      w1(count3) = wl2(1);
      w2(count3) = wl2(2);
      w3(count3) = wl2(3);
      w4(count3) = wl2(4);
      w5(count3) = wl2(5);
      w6(count3) = wl2(6);
      w7(count3) = wl2(7);
      w8(count3) = wl2(8);
      w9(count3) = wl2(9);
      % sprintf('The capture condition is %f ', capture_cond2)
%       sprintf(' The reward is %f ', reward)
%       sprintf(' The reward1 is %f ', reward1)
    end
    %
    % Compute the time delay error in two steps.
    % first compute the new value of the state based on the action taken.
    % Step 1 compute new value of new state.
    %
    %sinput = [dpxt2, dpyt2, up2];
    sinput1 = [dp1ext1, dp1eyt1];
    for i=1:no_of_rules
        phi(i) = fire_strength_for_rule(sinput1, rule(i).mf);
    end
    %
    % Compute the normalized firing strength for each rule.
    %
    phi_sum = sum(phi);
    phi_norm2 = zeros(1, no_of_rules);
    if (phi_sum ~= 0)
       for i=1:no_of_rules
           phi_norm2(i) = phi(i)/phi_sum;
       end
    end
    %
    % Compute the new value of the state
    %
    valp1tplus1 = 0;
    for i=1:no_of_rules
        valp1tplus1 = valp1tplus1 + phi_norm2(i)*psil1(i);
    end
    tdp1 = (rewardp1 + gamma*valp1tplus1) - valp1;
    %
    sinput2 = [dp2ext1, dp2eyt1];
    for i=1:no_of_rules
        phi(i) = fire_strength_for_rule(sinput2, rule(i).mf);
    end
    %
    % Compute the normalized firing strength for each rule.
    %
    phi_sum = sum(phi);
    phi_norm2 = zeros(1, no_of_rules);
    if (phi_sum ~= 0)
       for i=1:no_of_rules
           phi_norm2(i) = phi(i)/phi_sum;
       end
    end
    %
    % Compute the new value of the state
    %
    valp2tplus1 = 0;
    for i=1:no_of_rules
        valp2tplus1 = valp2tplus1 + phi_norm2(i)*psil2(i);
    end
    tdp2 = (rewardp2 + gamma*valp2tplus1) - valp2;
%
    sinput3 = [dp3ext1, dp3eyt1];
    for i=1:no_of_rules
        phi(i) = fire_strength_for_rule(sinput3, rule(i).mf);
    end
    %
    % Compute the normalized firing strength for each rule.
    %
    phi_sum = sum(phi);
    phi_norm2 = zeros(1, no_of_rules);
    if (phi_sum ~= 0)
       for i=1:no_of_rules
           phi_norm2(i) = phi(i)/phi_sum;
       end
    end
    %
    % Compute the new value of the state
    %
    valp3tplus1 = 0;
    for i=1:no_of_rules
        valp3tplus1 = valp3tplus1 + phi_norm2(i)*psil3(i);
    end
    tdp3 = (rewardp3 + gamma*valp3tplus1) - valp3;   
    %
    % Compute the parameter updates
    % 
    for i=1:no_of_rules
        psil1(i) = psil1(i)+ alpha*tdp1*phi1_norm(i);
        %sprintf(' Value psil(%d) is %f', i, psil(i))
    end
    %
    for i=1:no_of_rules        
        deltaw = beta*sign(tdp1*noise1)*phi1_norm(i);
        wl1(i) = wl1(i)+ deltaw;
    end
    %
    for i=1:no_of_rules
        psil2(i) = psil2(i)+ alpha*tdp2*phi2_norm(i);
        %sprintf(' Value psil(%d) is %f', i, psil(i))
    end
    %
    for i=1:no_of_rules        
        deltaw = beta * sign(tdp2 * noise2) * phi2_norm(i);
        wl2(i) = wl2(i)+ deltaw;
    end
    %
    for i=1:no_of_rules
        psil3(i) = psil3(i)+ alpha*tdp3*phi3_norm(i);
        %sprintf(' Value psil(%d) is %f', i, psil(i))
    end
    %
    for i=1:no_of_rules        
        deltaw = beta * sign(tdp3 * noise3) * phi3_norm(i);
        wl3(i) = wl3(i)+ deltaw;
    end
    %
    up2_old = up2;
    %
    dist1 = sqrt( (ya(7) - ya(1))^2 + (ya(8) - ya(2))^2); % Distance to pursuer 1
    dist2 = sqrt( (ya(7) - ya(3))^2 + (ya(8) - ya(4))^2); % Distance to pursuer 2
    dist3 = sqrt( (ya(7) - ya(5))^2 + (ya(8) - ya(6))^2); % Distance to pursuer 3
    if(dist1 < 0.1)
        game_on = 0; %Captured.
    end
    if(dist2 < 0.5)
        game_on = 0; %Captured.
        capture_cond2_old = 1;
        if (count < count_success)
            count_success = count;
            %remember the parameters for success!
            psil2_success = psil2;
            wl2_success = wl2;
            psil2_success_init = psil2_init;
            wl2_success_init = wl2_init;
            alpha_success = alpha;
            beta_success = beta;
            sigma_success = sigma;
            count_success_times = 1;
        end
        if (count == count_success)
            %remember the parameters for success!
            psil2_success = psil2;
            wl2_success = wl2;
            psil2_success_init = psil2_init;
            wl2_success_init = wl2_init;
            alpha_success = alpha;
            beta_success = beta;
            count_success_times = count_success_times + 1;
            sigma_success = (0.95)*sigma_success;
            %psil2 = psil2_success_init;
           % wl2 = wl2_success_init; 
           % alpha = alpha_init;
            %beta = beta_init;
            %sigma = sigma_init;
        end
         if (count > count_success)
            %remember the parameters for success!
            psil2 = psil2_success;
            wl2 = wl2_success; 
            alpha = alpha_success;
            beta = beta_success;
            sigma = sigma_success;
            sprintf(' The count is high reset sigma')
            %psil2 = psil2_success_init;
           % wl2 = wl2_success_init; 
           % alpha = alpha_init;
            %beta = beta_init;
            %sigma = sigma_init;
        end
        sprintf(' The Evader is Captured !!! The count is %d', count)
        sprintf(' The Evader is Captured !!! The best count is %d', count_success)
        sprintf(' The Evader is Captured !!! The times of best count is %d', count_success_times)
        sprintf(' The Evader is Captured !!! The epoch %d', j)
    end
    if(dist3 < 0.1)
        game_on = 0; %Captured.
    end
    if( count > 150) % stop the game
        game_on = 0;
    end
    %
    % If capture condition failed, reset the game to previous game
    % conditions
    if(capture_fail == 1)
    %   psil2 = psil2_success_init;
    %   wl2 = wl2_success_init; 
       game_on = 0; %start again
    %   alpha = alpha_success;
     %  beta = beta_success;
      % sigma = 1.05*sigma_success;
    end
%    game_on = 0;
    count = count + 1;
    xp1(count) = ya(1);
    yp1(count) = ya(2);
    xp2(count) = ya(3);
    yp2(count) = ya(4);
    xp3(count) = ya(5);
    yp3(count) = ya(6);
    xe(count) = ya(7);
    ye(count) = ya(8);
    % ****************************************************************
    % 3rd change (contd.)
    % 
    % I commented this code because real-time visualization of the players
    % trajectories will make the program run slowly.
    % ****************************************************************
    % 
    %     axis([-10, 10, -10, 10]);
    %     
    %     plot(xp1(count), yp1(count), xp2(count), yp2(count), xp3(count), yp3(count), xe(count), ye(count))
    %      pause(1);
    %      hold on; 
    % Get the frame to make a movie.
   % countm = countm + 1;
    %if(countm == 1)
    %   countm = 0;
    %   count2 = count2+1;
    %   m(count2) = getframe;
    %end     
    % ****************************************************************
    % 3rd change (contd.)
    %
    % Update the current figure with the new location of the players
    % The if statement "if mod(iteration_count,10) == 0" will plot the
    % trajectory of the players after every 10 iterations. This is done to
    % improve the visualization of the plot.
    % ****************************************************************
    if  mod(count,10) == 1
        plot(ya(1), ya(2), '*m', ya(3), ya(4), '*r', ya(5), ya(6), '*m', ya(7), ya(8), 'dk', 'MarkerFaceColor', 'k' )
        % uncomment this line to get real time visualization of the
        % players trajectory. (Warning: May slow down your system.)
        % pause(0.0000001);
    end
    % ****************************************************************
   end
   %td_plot(j) = td;
   % *******************************************************************
   % 3rd change (contd.)
   %
   % Create a new folder to save all the game plots
   % *******************************************************************
   if j == 1 % Check if this a new simulation
       date_and_time = datestr(clock,0); % obtain the current system time
       folderName = strcat('Simulation_results_', date_and_time); % define the name of the folder
       folderName = strrep(folderName, ' ', '_');  % replace all ' ' with '_'
       folderName = strrep(folderName, ':', '_');  % replace all ':' with '_'
       folderName = strrep(folderName, '-', '_');  % replace all '-' with '_'
       mkdir(folderName) % create new folder
   end
   % *******************************************************************
   % *******************************************************************
   % 3rd change (contd.)
   %
   % Save the game plots in the new folder
   % *******************************************************************
   if  mod(j,100) == 0
      fileName = sprintf('Epoch_%d.jpg', j); % define the file name
      saveas( gamePlot, [ pwd strcat('/', folderName, '/', fileName, '.png') ]  );  % save the file
   end
   % *******************************************************************
   % *******************************************************************
   % 4th change
   %
   % Decay the learning rate as we prepare for the next epoch
   % *******************************************************************
   %alpha = (0.999^j)* alpha_init;
   %beta = (0.999^j)* beta_init;
   %sigma = (0.999^j)* sigma_init;
   alpha = 0.9999*alpha;
   beta = 0.9999*beta;
   sigma = 0.999*sigma;
   % *******************************************************************
end
% 
% hold off;
% plot(td_plot)
end % function agent_sim complete
% Define the dynamic equations for the agent
%
function [y] = dagent(t, y)
%
global up1; % Heading angle pursuer 1
global up2; % Heading angle pursuer 2
global up3; % Heading angle pursuer 3
global ue;  % heading angle evader
global vp1; % pursuer 1 speed
global vp2; % pursuer 2 speed
global vp3; % pursuer 3 speed
global ve;  % evader speed
%
y(1) = vp1*cos(up1);
y(2) = vp1*sin(up1);
y(3) = vp2*cos(up2);
y(4) = vp2*sin(up2);
y(5) = vp3*cos(up3);
y(6) = vp3*sin(up3);
y(7) = ve*cos(ue);
y(8) = ve*sin(ue);
end


