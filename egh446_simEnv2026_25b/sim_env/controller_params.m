function controller_params()
% CONTROLLER_PARAMS Initialise controller tuning parameters
%
% Places controller parameters into the MATLAB base workspace so they
% can be accessed by the Simulink controller blocks.

%% Controller Parameters

% Heading controller proportional gain
K_theta = 1;

% Distance-based speed gain
K_dist = 0.75;

% Maximum forward velocity [m/s]
v_max = 1;                 % was 2: halved to give obstacle avoidance time to react

% Maximum yaw rate [rad/s]
omega_max = (2*pi)/5;

% Min heading speed for above 90 degree turns
heading_speed_min = 0.1;

% RVWP guidance
rvwp_lookahead = 5;        % see note below: try 3 if it cuts corners near walls
capture_radius = 0.8;

% Heading PID controller
%K_i = 0;
%K_d = 0.05;
%N_d = 5;

%% Export parameters to base workspace
assignin('base', 'K_theta', K_theta);
assignin('base', 'K_dist', K_dist);
assignin('base', 'v_max', v_max);
assignin('base', 'omega_max', omega_max);
assignin('base', 'heading_speed_min', heading_speed_min);
assignin('base', 'rvwp_lookahead', rvwp_lookahead);
assignin('base', 'capture_radius', capture_radius);
%assignin('base','K_i',K_i);
%assignin('base','K_d',K_d);
%assignin('base','N_d',N_d);
end