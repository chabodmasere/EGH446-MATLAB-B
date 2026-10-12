function draw_avoid_view(pose, theta_cmd, theta_safe, avoid_active, d_obs, v, omega)
% DRAW_AVOID_VIEW  Obstacle avoidance overlay for the Robot Visualization figure.
%
%   While avoidance is active:
%     GUIDANCE  dotted grey arrow: heading the path follower asks for
%     STEER     solid red arrow: heading after avoidance, sent to the controller
%     INTENT    dashed red arc: where the robot goes next if v and omega hold
%     TARGET    red ring at 0.40 m around the obstacle being avoided
%   Always:
%     DODGES    red trail over every stretch where avoidance was active
%
%   The target ring is display only: it is matched from the known obstacle
%   list using d_obs. The avoidance itself only uses the onboard detections.
%
%   Save this file in the sim_env folder. It is called from the 'avoid_viz'
%   MATLAB Function block through coder.extrinsic.

ARROW     = 1.5;     % [m] arrow length
HORIZON   = 2.5;     % [m] length of the intent arc
CLEARANCE = 0.40;    % [m] robot radius + safety margin
JUMP      = 2.0;     % [m] a pose jump this large means a new run
MAX_FPS   = 10;     % redraw cap (was 30)
RED       = [0.95 0.15 0.20];
GREY      = [0.75 0.75 0.80];

persistent h st

% Look the axes up only when the cached one has gone (the visualizer makes
% new axes at the start of a run). This block runs at 100 Hz, and a figure
% search on every call was slowing the simulation.
if ~isempty(h) && isgraphics(h.ax)
    ax = h.ax;
else
    fig = findobj('Type', 'figure', 'Tag', 'RobotVisualization');
    if isempty(fig), return; end
    ax = findobj(fig(1), 'Type', 'axes');
    if isempty(ax), return; end
    ax = ax(1);
end

x = pose(1);  y = pose(2);  th = pose(3);

if isempty(h) || ~isgraphics(h.ax) || h.ax ~= ax || ~isgraphics(h.trail) ...
        || hypot(x - st.x, y - st.y) > JUMP
    h  = create_graphics(ax, RED, GREY);
    st = struct('tx', nan(1, 4096), 'ty', nan(1, 4096), 'n', 0, 'was', false, ...
                'obs', base_obstacles(), 'x', x, 'y', y, 't0', tic, 'tLast', -Inf);
end
st.x = x;  st.y = y;
active = avoid_active > 0.5;

% ---------- Dodge trail: record every call, NaN gap between dodges ----------
if active
    if ~st.was && st.n > 0
        st = add_point(st, NaN, NaN);
    end
    st = add_point(st, x, y);
end
st.was = active;

% ---------- Frame-rate cap ----------
t = toc(st.t0);
if t - st.tLast < 1/MAX_FPS
    return
end
st.tLast = t;

set(h.trail, 'XData', st.tx(1:st.n), 'YData', st.ty(1:st.n));

if ~active
    set([h.guid, h.steer, h.arc, h.ring], 'XData', NaN, 'YData', NaN);
    drawnow limitrate
    return
end

% ---------- Arrows ----------
[gx, gy] = arrow_xy(x, y, theta_cmd, ARROW);
[sx, sy] = arrow_xy(x, y, theta_safe, ARROW);
set(h.guid,  'XData', gx, 'YData', gy);
set(h.steer, 'XData', sx, 'YData', sy);

% ---------- Intent arc: constant curvature omega/v ----------
kap = omega/max(v, 0.05);
if abs(kap) < 1e-3
    s  = [0 HORIZON];
    ax_ = x + s*cos(th);
    ay_ = y + s*sin(th);
else
    s  = linspace(0, min(HORIZON, pi/abs(kap)), 40);
    ax_ = x + (sin(th + kap*s) - sin(th))/kap;
    ay_ = y - (cos(th + kap*s) - cos(th))/kap;
end
set(h.arc, 'XData', ax_, 'YData', ay_);

% ---------- Obstacle being avoided ----------
set(h.ring, 'XData', NaN, 'YData', NaN);
if isfinite(d_obs) && ~isempty(st.obs)
    [err, i] = min(abs(hypot(st.obs(:,1) - x, st.obs(:,2) - y) - d_obs));
    if err < 0.5
        set(h.ring, 'XData', st.obs(i,1) + CLEARANCE*h.cu, ...
                    'YData', st.obs(i,2) + CLEARANCE*h.su);
    end
end

drawnow limitrate
end

% =====================================================================
function st = add_point(st, px, py)
if st.n == numel(st.tx)
    st.tx = [st.tx, nan(1, st.n)];
    st.ty = [st.ty, nan(1, st.n)];
end
st.n = st.n + 1;
st.tx(st.n) = px;
st.ty(st.n) = py;
end

% =====================================================================
function [X, Y] = arrow_xy(x, y, a, L)
% Shaft plus a two-stroke head, NaN-separated so it is one line object.
hl = 0.3;
ex = x + L*cos(a);
ey = y + L*sin(a);
X = [x, ex, NaN, ex - hl*cos(a - 0.4), ex, ex - hl*cos(a + 0.4)];
Y = [y, ey, NaN, ey - hl*sin(a - 0.4), ey, ey - hl*sin(a + 0.4)];
end

% =====================================================================
function obs = base_obstacles()
obs = zeros(0, 2);
try
    o = double(evalin('base', 'obstacles'));
    obs = o(:, 1:2);
catch
end
end

% =====================================================================
function h = create_graphics(ax, red, grey)
common = {'HandleVisibility', 'off', 'HitTest', 'off', ...
          'PickableParts', 'none', 'Tag', 'avoidOverlay'};

delete(findobj(ax, 'Tag', 'avoidOverlay'));

h.ax    = ax;
h.trail = line(ax, NaN, NaN, 'LineWidth', 3.5, 'Color', [red, 0.55], common{:});
h.arc   = line(ax, NaN, NaN, 'LineStyle', '--', 'LineWidth', 1.5, 'Color', red, common{:});
h.guid  = line(ax, NaN, NaN, 'LineStyle', ':', 'LineWidth', 1.8, 'Color', grey, common{:});
h.steer = line(ax, NaN, NaN, 'LineWidth', 2.2, 'Color', red, common{:});
h.ring  = line(ax, NaN, NaN, 'LineWidth', 1.5, 'Color', red, common{:});

th   = linspace(0, 2*pi, 48);
h.cu = cos(th);
h.su = sin(th);
end
