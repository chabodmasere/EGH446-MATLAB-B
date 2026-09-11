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

%% Fixed start for controlled testing

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mission Planner
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate random waypoint locations
occ = inflate_map(logical_map, 4);          % 0.25 m radius + 0.15 m margin

wp_list = wp_gen(x_max, y_max, no_wps, [robot.X, robot.Y], occ);
D = waypoint_distances([[robot.X, robot.Y]; wp_list], occ);
[wp_ordered, dist_cum, history] = wp_antColony(wp_list, [robot.X,robot.Y], D);

wp_route = [robot.X robot.Y; wp_ordered];
[path, path_dist] = wp_path(wp_route, occ);
wp_plot = path;
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