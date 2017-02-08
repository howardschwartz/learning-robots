function [capture, psi, w, alpha, beta, sigma] = robot_captured(captured, psi, w, count)
% [capture] = robot_captured(captured)
% This functions sets the parametrs for the captured robots
%
   capture = captured;
   if (count < captured.count_success)
       capture.count_success = count;
       %remember the parameters for success!
       capture.psi_success = psi;
       capture.w_success = w;
       capture.psi_success_init = captured.psi_init;
       capture.w_success_init = captured.w_init;
       capture.alpha_success = captured.alpha;
       capture.beta_success = captured.beta;
       capture.sigma_success = captured.sigma;
       capture.count_success_times = 1;
    elseif (count == captured.count_success)
        %remember the parameters for success!
       capture.psi_success = psi;
       capture.w_success = w;
       capture.psi_success_init = captured.psi_init;
       capture.w_success_init = captured.w_init;
       capture.alpha_success = captured.alpha;
       capture.beta_success = captured.beta;
       capture.count_success_times = captured.count_success_times + 1;
       capture.sigma_success = (0.95)*captured.sigma_success;
   end
   %remember the parameters for success!
   psi = capture.psi_success;
   w = capture.w_success;
   alpha = capture.alpha_success;
   beta = capture.beta_success;
   sigma = capture.sigma_success;
end

