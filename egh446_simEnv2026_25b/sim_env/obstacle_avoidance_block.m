function [theta_cmd_safe, v_scale, avoid_active, d_obs] = obstacle_avoidance(theta_cmd, x_hat, detections, ranges, dist_to_wp)
%#codegen
% Reactive avoidance for a STEERED (car-like) vehicle.
%   Obstacles : Object Detector -> remembered in world frame -> repulsion + tangential slide
%   Walls     : Lidar fan       -> short-range repulsion (backup to the planner's inflation)
%   Speed     : slows near obstacles and in sharp turns, but never stops
%
% GOAL-AWARE FIX: as the robot nears its waypoint, obstacle reach shrinks
% with dist_to_wp (never below D_NEAR, protecting the 0.40 m clearance)
% and the sideways slide fades out, so it can finish its approach.
% If dist_to_wp is not available (input unwired -> reads 0) the block
% simply behaves like the original avoidance: full reach, full slide.
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

% ---------- Tuning: smoothing ----------
a_f     = 0.02;   % heading low-pass per step (block runs at 100 Hz -> ~0.5 s time constant)

% ---------- Tuning: speed ----------
v_floor = 0.3;    % never below 30% speed (steered vehicle must move to turn)

% ---------- Memory ----------
persistent mem n_mem side th_f
if isempty(mem)
    mem   = zeros(N_MAX, 2);
    n_mem = 0;
    side  = 0;
end

xr   = x_hat(1);
yr   = x_hat(2);
th_r = x_hat(3);

% ---------- 0) Goal-aware reach (only when dist_to_wp is available) ----------
if dist_to_wp > 0
    d_eff  = min(d_inf, max(D_NEAR, dist_to_wp));
    kt_eff = k_tan * min(1, dist_to_wp / d_inf);
    dw_eff = min(d_wall, max(0.5, dist_to_wp));
else
    d_eff  = d_inf;       % no goal info: behave like the original block
    kt_eff = k_tan;
    dw_eff = d_wall;
end

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
% Low-pass the world-frame heading. Estimate noise reaches this point
% through the guidance bearing and, much amplified, through the 1/r
% obstacle and wall terms; averaging it here stops it being passed on
% to the controller as steering jitter.
th = th_r + atan2(hy, hx);
if isempty(th_f)
    th_f = th;
end
th_f = th_f + a_f*atan2(sin(th - th_f), cos(th - th_f));
theta_cmd_safe = atan2(sin(th_f), cos(th_f));
e_safe = atan2(sin(th_f - th_r), cos(th_f - th_r));

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