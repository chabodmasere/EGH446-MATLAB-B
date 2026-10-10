function draw_lidar_view(pose, ranges, scanAngles, scanPose)
% DRAW_LIDAR_VIEW  Lidar overlay for the Robot Visualization figure.
%
%   draw_lidar_view(pose, ranges, scanAngles, scanPose)
%
%   pose       [x y theta]   robot pose (m, m, rad)
%   ranges     [nB x 1]      lidar ranges (NaN = no return)
%   scanAngles [nB x 1]      beam angles relative to robot heading (rad)
%   scanPose   [x y theta]   pose the Lidar Sensor block measured from
%                            (optional, defaults to pose)
%
%   The robot-centred parts are drawn at POSE, which is what the visualizer
%   shows. The returns are placed from SCANPOSE, because the ranges are
%   only correct relative to the pose they were measured from; placing
%   them from a different pose smears the mapped walls.
%
%   Replaces the fan of dashed beam lines with a picture of what the lidar
%   is actually telling the robot:
%     SCAN FIELD   a translucent wedge covering the free space the scan has
%                  swept; its outer edge is the surface the lidar sees
%     RETURNS      a dot where each beam lands, coloured by range
%                  (red = close, amber = caution, green = clear)
%     MAPPED WALLS every return is remembered, so the walls the robot has
%                  scanned build up behind it as an orange point cloud. The
%                  Lidar Sensor block adds up to 0.45 m to every range, so
%                  the cloud sits in a band just behind each wall face
%     CLEARANCE    a ring at the safety radius and a dotted line to the
%                  nearest return, labelled with its distance; both take the
%                  colour of that range
%
%   Save this file in the sim_env folder. It is called from the 'lidar_viz'
%   MATLAB Function block through coder.extrinsic. Do NOT paste it into a block.

% ---------- Behaviour ----------
R_MAX     = 5;       % [m] lidar max range (match the Lidar Sensor block)
CLEARANCE = 0.40;    % [m] robot radius + safety margin
R_CLOSE   = 0.5;     % [m] range drawn fully red
R_CLEAR   = 2.5;     % [m] range drawn fully green
CELL      = 0.10;    % [m] mapped-wall resolution (one remembered point per cell)
JUMP      = 2.0;     % [m] a pose jump this large means a new run: clear the map
MAX_FPS   = 30;      % redraw cap so the overlay never slows the simulation

% ---------- Look ----------
FIELD_COL   = [0.00 0.62 0.72];
FIELD_ALPHA = 0.13;
MAP_COL     = [1.00 0.45 0.00];
RAMP        = [0.86 0.10 0.10;    % close
               1.00 0.66 0.00;    % caution
               0.10 0.66 0.30];   % clear

persistent h st

% ---------- Find the Robot Visualization axes ----------
fig = findobj('Type', 'figure', 'Tag', 'RobotVisualization');
if isempty(fig), return; end
ax = findobj(fig(1), 'Type', 'axes');
if isempty(ax), return; end
ax = ax(1);

x = pose(1);  y = pose(2);
if nargin < 4
    scanPose = pose;
end

% ---------- (Re)create graphics and memory only when needed ----------
if isempty(h) || ~isgraphics(h.ax) || h.ax ~= ax || ~isgraphics(h.field) ...
        || hypot(x - st.x, y - st.y) > JUMP
    h  = create_graphics(ax, FIELD_COL, FIELD_ALPHA, MAP_COL);
    xl = ax.XLim;  yl = ax.YLim;
    st = struct('x0', xl(1), 'y0', yl(1), ...
                'seen', false(ceil(diff(yl)/CELL) + 1, ceil(diff(xl)/CELL) + 1), ...
                'mx', zeros(1, 4096), 'my', zeros(1, 4096), 'n', 0, ...
                'x', x, 'y', y, 't0', tic, 'tLast', -Inf);
end
st.x = x;  st.y = y;

% ---------- Beam geometry ----------
a   = scanAngles(:)';
r   = ranges(:)';
hit = isfinite(r) & r > 0 & r < R_MAX;
r(~hit) = R_MAX;
bx = scanPose(1) + r.*cos(scanPose(3) + a);
by = scanPose(2) + r.*sin(scanPose(3) + a);

% ---------- Remember each return once per map cell (every call, not just drawn frames) ----------
grew = false;
for i = find(hit)
    c  = floor((bx(i) - st.x0)/CELL) + 1;
    rw = floor((by(i) - st.y0)/CELL) + 1;
    if rw < 1 || c < 1 || rw > size(st.seen, 1) || c > size(st.seen, 2) || st.seen(rw, c)
        continue
    end
    st.seen(rw, c) = true;
    if st.n == numel(st.mx)
        st.mx = [st.mx, zeros(1, st.n)];
        st.my = [st.my, zeros(1, st.n)];
    end
    st.n = st.n + 1;
    st.mx(st.n) = bx(i);
    st.my(st.n) = by(i);
    grew = true;
end
if grew
    set(h.map, 'XData', st.mx(1:st.n), 'YData', st.my(1:st.n));
end

% ---------- Frame-rate cap ----------
t = toc(st.t0);
if t - st.tLast < 1/MAX_FPS
    return
end
st.tLast = t;

% ---------- Scan field and the surface it ends on ----------
set(h.field, 'XData', [x, bx], 'YData', [y, by]);
sx = bx;  sy = by;
sx(~hit) = NaN;  sy(~hit) = NaN;            % only draw the edge where something was seen
set(h.surface, 'XData', sx, 'YData', sy);

% ---------- Returns, coloured by range ----------
if any(hit)
    set(h.returns, 'XData', bx(hit), 'YData', by(hit), ...
        'FaceVertexCData', range_colour(r(hit), R_CLOSE, R_CLEAR, RAMP));
    [rmin, j] = min(r + ~hit*R_MAX);
    col = range_colour(rmin, R_CLOSE, R_CLEAR, RAMP);
    set(h.near, 'XData', [x, bx(j)], 'YData', [y, by(j)], 'Color', col);
    set(h.label, 'Position', [x + 0.9, y + 1.2, 0], 'Color', col, ...
        'String', sprintf('nearest %.2f m', rmin));
else
    col = RAMP(3, :);
    set(h.returns, 'XData', NaN, 'YData', NaN, 'FaceVertexCData', col);
    set(h.near, 'XData', NaN, 'YData', NaN);
    set(h.label, 'String', '');
end
set(h.ring, 'XData', x + CLEARANCE*h.cu, 'YData', y + CLEARANCE*h.su, 'Color', col);

drawnow limitrate
end

% =====================================================================
function c = range_colour(r, rClose, rClear, ramp)
% Red -> amber -> green as the range opens up.
s = min(1, max(0, (r(:) - rClose)/(rClear - rClose)));
c = [interp1([0 0.5 1], ramp(:,1), s), ...
     interp1([0 0.5 1], ramp(:,2), s), ...
     interp1([0 0.5 1], ramp(:,3), s)];
end

% =====================================================================
function h = create_graphics(ax, fieldCol, fieldAlpha, mapCol)
% Builds every overlay object once. Low-level patch/line/text never clear
% the axes, so there is no need to touch the axes' hold state.

common = {'HandleVisibility', 'off', 'HitTest', 'off', ...
          'PickableParts', 'none', 'Tag', 'lidarOverlay'};

delete(findobj(ax, 'Tag', 'lidarOverlay'));   % remove leftovers from old runs

h.ax      = ax;
h.field   = patch(ax, NaN, NaN, fieldCol, 'EdgeColor', 'none', ...
                  'FaceAlpha', fieldAlpha, common{:});
h.map     = line(ax, NaN, NaN, 'LineStyle', 'none', 'Marker', '.', ...
                 'MarkerSize', 7, 'Color', mapCol, common{:});
h.surface = line(ax, NaN, NaN, 'LineWidth', 1.6, 'Color', fieldCol, common{:});
h.near    = line(ax, NaN, NaN, 'LineStyle', ':', 'LineWidth', 1.4, common{:});
h.returns = patch(ax, 'XData', NaN, 'YData', NaN, 'FaceColor', 'none', ...
                  'EdgeColor', 'none', 'Marker', 'o', 'MarkerSize', 5, ...
                  'MarkerFaceColor', 'flat', 'MarkerEdgeColor', [0.15 0.15 0.15], ...
                  'FaceVertexCData', [0 0 0], common{:});
h.ring    = line(ax, NaN, NaN, 'LineWidth', 1.5, common{:});
h.label   = text(ax, NaN, NaN, '', 'FontWeight', 'bold', 'FontSize', 9, ...
                 'BackgroundColor', [1 1 1], 'Margin', 1, common{:});

th   = linspace(0, 2*pi, 48);
h.cu = cos(th);
h.su = sin(th);
end
