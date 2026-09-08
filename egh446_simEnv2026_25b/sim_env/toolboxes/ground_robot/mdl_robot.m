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

coord_lo = -100;
coord_hi = 100;
no_wps = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial Conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Random start position and heading

robot.X = randi([coord_lo, coord_hi]);
robot.Y = randi([coord_lo, coord_hi]);

% Random heading from -pi to pi
robot.Theta = -pi + 2*pi*rand();

%% Fixed start for controlled testing
%robot.X = 0;
%robot.Y = 0;
%robot.Theta = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mission Planner
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate random waypoint locations
wp_list = wp_gen(coord_lo, coord_hi, no_wps);

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