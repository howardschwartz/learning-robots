function [psi, w, sigma] = change_capture_condition(capture)
% [psi, w, sigma] = change_capture_condition(capture)
%  This function will determine what to do when a robot can no longer
%  capture an evader.    
   if (capture.count_success_times > 0)
       %psi = capture.psi_success_init;
       psi = capture.psi;
       w = capture.w_success_init;
       sigma = capture.sigma_success;
   else
      psi = capture.psi;
      w = capture.w;
      sigma = capture.sigma;
   end
end

