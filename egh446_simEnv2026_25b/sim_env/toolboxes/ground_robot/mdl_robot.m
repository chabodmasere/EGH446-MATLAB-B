% Ground Robot parameters
%
% Properties:
%
% X       initial position X (1x1)
% Y       initial position Y (1x1)
% Theta   initial orientation (1x1)
%
% Notes:
% - SI units are used.

disp('mdl_robot executed - generating new mission')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Random Initial Seeding
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Only enable if MATLAB RNG behaviour causes repeated missions
% rng('shuffle');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Environment Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_max = 52;
y_max = 41;
no_wps = 5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial Conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Fixed starting position and heading as per Instructions

robot.X = 2;
robot.Y = 2;
robot.Theta = 0;
% robot.Theta = -pi + 2*pi*rand();   % random heading, for robustness testing only

%% Fixed start for controlled testing

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mission Planner
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate random waypoint locations
wp_list = wp_gen(x_max, y_max, no_wps, [robot.X, robot.Y], logical_map);

% Optimise waypoint visitation order from vehicle start position
[wp_ordered, tour_len] = wp_antColony( ...
    wp_list, [robot.X robot.Y]);

% Route used by RVWP guidance:
% initial vehicle position + ordered waypoints
wp_plot = [robot.X robot.Y;
    wp_ordered];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Diagnostics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('INITIAL VEHICLE STATE:')
fprintf('X = %.1f m, Y = %.1f m, Theta = %.1f deg\n', ...
    robot.X, robot.Y, rad2deg(robot.Theta));

disp('NEW MISSION WAYPOINTS:')
disp(wp_ordered)

disp('NEW WP_PLOT:')
disp(wp_plot)