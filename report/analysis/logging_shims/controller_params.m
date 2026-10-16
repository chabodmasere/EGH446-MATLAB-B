function controller_params()
% CONTROLLER_PARAMS Initialise controller tuning parameters
%
% Copy of sim_env/controller_params.m used only by the report's batch runs:
% identical, except that v_max, capture_radius, K_theta and K_dist can be
% overridden through the global REPORT_OVERRIDE for the parameter sweeps.
%
% Places controller parameters into the MATLAB base workspace so they
% can be accessed by the Simulink controller blocks.

%% Controller Parameters

% Heading controller proportional gain
K_theta = 2;               % was 1: doubled after the gain test (report, Controller)

% Distance-based speed gain
K_dist = 0.75;

% Maximum forward velocity [m/s]
v_max = 1;                 % was 2: halved to give obstacle avoidance time to react

% Maximum yaw rate [rad/s]
omega_max = (2*pi)/5;

% Min heading speed for above 90 degree turns
heading_speed_min = 0.1;

% RVWP guidance
rvwp_lookahead = 5;        % Part A look-ahead, NOT used in Part B: guidance follows the path point by point (report, Path following); kept only because the Guidance block still takes the input
capture_radius = 0.4;      % [m] waypoints 1-4 (was 0.8: see report, Speed and capture radius)

% Heading PID controller
%K_i = 0;
%K_d = 0.05;
%N_d = 5;

%% Overrides for the report's parameter sweep (batch runs only)
global REPORT_OVERRIDE
if isstruct(REPORT_OVERRIDE)
    if isfield(REPORT_OVERRIDE, 'v_max'),          v_max = REPORT_OVERRIDE.v_max; end
    if isfield(REPORT_OVERRIDE, 'capture_radius'), capture_radius = REPORT_OVERRIDE.capture_radius; end
    if isfield(REPORT_OVERRIDE, 'K_theta'),        K_theta = REPORT_OVERRIDE.K_theta; end
    if isfield(REPORT_OVERRIDE, 'K_dist'),         K_dist = REPORT_OVERRIDE.K_dist; end
end

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
