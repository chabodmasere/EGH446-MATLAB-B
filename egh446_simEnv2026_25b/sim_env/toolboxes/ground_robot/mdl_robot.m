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
% The planner works on grid points (x, y) = ((col-1)/10, (nRows-row)/10), which
% are the CORNERS of the map cells. A grid point touches a wall if any of the
% four cells meeting at it is occupied, so mark those first; inflating that
% gives true distances to the wall faces.
wall_pts = logical_map;
wall_pts(:, 2:end)       = wall_pts(:, 2:end)       | logical_map(:, 1:end-1);
wall_pts(1:end-1, :)     = wall_pts(1:end-1, :)     | logical_map(2:end, :);
wall_pts(1:end-1, 2:end) = wall_pts(1:end-1, 2:end) | logical_map(2:end, 1:end-1);

% Required clearance is 0.25 m radius + 0.15 m = 0.40 m (4 cells). The extra
% cells allow for path-tracking error and estimator noise.
occ = inflate_map(wall_pts, 6);

% Obstacle positions are used ONLY to keep waypoints off the obstacles
% (brief: waypoints must not overlap obstacles). The path is still planned
% on the empty map: occ contains walls only.
wp_list = wp_gen(x_max, y_max, no_wps, [robot.X, robot.Y], occ, obstacles(:,1:2), 1.0);
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