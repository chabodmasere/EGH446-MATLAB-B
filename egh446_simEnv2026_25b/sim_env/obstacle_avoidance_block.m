function [theta_cmd_safe, v_scale, avoid_active, d_obs] = obstacle_avoidance(theta_cmd, x_hat, detections, ranges, dist_to_wp)
%#codegen
% Reactive avoidance for a STEERED (car-like) vehicle.
%   Obstacles : Object Detector -> remembered in world frame -> repulsion + tangential slide
%   Walls     : Lidar fan       -> short-range repulsion (backup to the planner's inflation)
%   Speed     : slows near obstacles and in sharp turns, but never stops
%
% GOAL-AWARE FIX (stops the robot circling a waypoint):
%   The 12 obstacles are not on the planner's map, so a random waypoint can
%   land right next to one. The old repulsion (felt from 3 m) plus the
%   tangential slide then held the robot outside the capture radius and
%   swept it round and round the obstacle forever. Now, as the robot nears
%   its waypoint, the obstacle "reach" shrinks with dist_to_wp (but never
%   below D_NEAR, which still protects the 0.40 m clearance) and the
%   sideways slide fades out, so the robot can finish its approach.
%
% Inputs
%   theta_cmd  [1]    heading from Guidance (world frame, rad)
%   x_hat      [3]    estimate [x; y; theta]
%   detections [<=3x3, variable size]  rows = [range, bearing (+ = left), label]
%   ranges     [13]   lidar ranges at scanAngles below (NaN = no hit)
%   dist_to_wp [1]    distance to the current waypoint (Guidance output 2)
% Outputs
%   theta_cmd_safe [1]  heading to send to the Controller
%   v_scale        [1]  multiplies v_cmd
%   avoid_active   [1]  1 while an obstacle is influencing the robot
%   d_obs          [1]  distance to nearest remembered obstacle ahead (inf if none)

% ---------- Tuning: obstacles ----------
d_inf   = 3.0;    % [m] start reacting to an obstacle at this range (far from goal)
d_stop  = 0.7;    % [m] range at which obstacle slow-down is strongest
k_rep   = 1.5;    % push away from obstacle
k_tan   = 1.0;    % slide along obstacle edge
v_min   = 0.25;   % obstacle slow-down limit
merge_r = 0.75;   % [m] same-obstacle merge distance
N_MAX   = 12;     % obstacles remembered
b_max   = 0.6*pi; % ignore obstacles more than ~108 deg off the nose (already passed)

% ---------- Tuning: goal-aware shrink ----------
D_NEAR  = 0.7;    % [m] smallest obstacle reach; keeps centre >= ~0.45 m from obstacle

% ---------- Tuning: walls (lidar) ----------
scanAngles = linspace(-pi/2, pi/2, 13);   % MUST match the Lidar Sensor block
d_wall  = 1.0;    % [m] walls closer than this push the robot away
k_wall  = 0.8;    % wall push strength

% ---------- Tuning: speed ----------
v_floor = 0.3;    % never below 30% speed (steered vehicle must move to turn)

% ---------- Memory ----------
persistent mem n_mem side
if isempty(mem)
    mem   = zeros(N_MAX, 2);
    n_mem = 0;
    side  = 0;
end

xr   = x_hat(1);
yr   = x_hat(2);
th_r = x_hat(3);

% ---------- 0) Mission complete: hold still ----------
% Guidance sends dist_to_wp = 0 (and theta_cmd = theta) once the last
% waypoint is captured. Pass the heading straight through and zero the
% speed, so nothing here can make the robot rotate on the spot.
if dist_to_wp <= 0
    theta_cmd_safe = theta_cmd;
    v_scale        = 0;
    avoid_active   = 0;
    d_obs          = inf;
    return
end

% Obstacle reach and slide strength shrink as the waypoint gets close
d_eff  = min(d_inf, max(D_NEAR, dist_to_wp));
kt_eff = k_tan * min(1, dist_to_wp / d_inf);
dw_eff = min(d_wall, max(0.5, dist_to_wp));

% ---------- 1) Store detected obstacles in WORLD coordinates ----------
for i = 1:size(detections, 1)
    r = detections(i,1);
    b = detections(i,2);
    if r <= 0
        continue
    end
    ox = xr + r*cos(th_r + b);
    oy = yr + r*sin(th_r + b);

    matched = false;
    for j = 1:n_mem
        if hypot(mem(j,1) - ox, mem(j,2) - oy) < merge_r
            mem(j,:) = 0.8*mem(j,:) + 0.2*[ox, oy];
            matched = true;
            break
        end
    end
    if ~matched && n_mem < N_MAX
        n_mem = n_mem + 1;
        mem(n_mem,:) = [ox, oy];
    end
end

% ---------- 2) Attraction: guidance direction (body frame) ----------
e  = atan2(sin(theta_cmd - th_r), cos(theta_cmd - th_r));
hx = cos(e);
hy = sin(e);

% ---------- 3) Obstacles: push away + slide along the edge ----------
d_min = inf;
for j = 1:n_mem
    dx = mem(j,1) - xr;
    dy = mem(j,2) - yr;
    r  = hypot(dx, dy);
    if r > d_eff || r < 1e-3
        continue
    end
    b = atan2(dy, dx) - th_r;
    b = atan2(sin(b), cos(b));
    if abs(b) > b_max
        continue
    end

    if side == 0
        if b >= 0
            side = -1;     % obstacle on left -> pass on the right
        else
            side = 1;      % obstacle on right -> pass on the left
        end
    end

    w  = k_rep * (1/r - 1/d_eff);
    hx = hx - w*cos(b) - w*kt_eff*side*sin(b);
    hy = hy - w*sin(b) + w*kt_eff*side*cos(b);
    d_min = min(d_min, r);
end

% ---------- 4) Walls: short-range push from each lidar beam ----------
nb = min(numel(ranges), numel(scanAngles));
for i = 1:nb
    r = ranges(i);
    if isnan(r) || r <= 0 || r > dw_eff
        continue
    end
    a  = scanAngles(i);
    w  = k_wall * (1/r - 1/dw_eff);
    hx = hx - w*cos(a);
    hy = hy - w*sin(a);
end

% ---------- 5) Output heading ----------
e_safe = atan2(hy, hx);
th = th_r + e_safe;
theta_cmd_safe = atan2(sin(th), cos(th));

% ---------- 6) Speed ----------
if isinf(d_min)
    side = 0;
    v_obs = 1;
    avoid_active = 0;
else
    v_obs = min(1, max(v_min, (d_min - d_stop)/(d_inf - d_stop)));
    avoid_active = 1;
end
v_turn  = 0.5 + 0.5*max(0, cos(e_safe));
v_scale = max(v_floor, v_obs * v_turn);
d_obs   = d_min;
end