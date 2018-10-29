function [robot, action] = compute_robot_actionv8(robot, not_zero_count, rule_set_number)
% This function computes the robot action
% The inputs are the robot structure and the input vector
% The output is the heading, but can be anything
%
    %
    % Compute the action for pursuer CAN capture
    %
    u = 0;
    w = robot.w;
    no_capture_w = robot.no_capture_w;
    parameters_capture = robot.rules_fired(rule_set_number).parameters_capture;
    parameters_no_capture = robot.rules_fired(rule_set_number).parameters_no_capture;
    rules_fired = robot.current_rules_fired;
    phi_norm = robot.phi_norm_actor;
    if (robot.condition == 1)
       for i=1:not_zero_count
           i1 = rules_fired(i);
           %u = u + phi_norm(i1)*w(i1);
           u = u + phi_norm(i1)*parameters_capture(i);
       end
       if(u < -pi || u > pi)
            sprintf(' The computed action is out of range %f ', u)
            %reset w
            for i=1:not_zero_count
               i1 = rules_fired(i);
               %robot.w(i1) = 0;
               robot.rules_fired(rule_set_number).parameters_capture(i) = 0;
            end  
        end
    end
    %
    % Compute the value for pursuer CANNOT capture
    %
    if (robot.condition == 0)
        for i=1:not_zero_count
            i1 = rules_fired(i);
            %u = u + phi_norm(i1)*no_capture_w(i1);
            u = u + phi_norm(i1)*parameters_no_capture(i);
        end
        if(u < -pi || u > pi)
            sprintf(' The computed action is out of range %f ', u)
            %reset no_capture_w
            for i=1:not_zero_count
               i1 = rules_fired(i);
               %robot.no_capture_w(i1) = 0;
               robot.rules_fired(rule_set_number).parameters_no_capture(i) = 0;
            end  
        end
    end         
    action = u;
end

