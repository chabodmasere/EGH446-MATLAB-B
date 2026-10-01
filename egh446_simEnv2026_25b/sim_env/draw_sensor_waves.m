function draw_sensor_waves(pose, ranges, scanAngles, d_obs, dist_wp)
% DRAW_SENSOR_WAVES  Clean, animated sensor overlay on the Robot Visualization figure.
%
%   - A translucent "scan area" shows the free space the lidar can see
%   - Thin, faint beams with a dot where each beam hits a wall
%   - Pulse arcs ripple outward and stop at the walls
%   - Colour shows what the robot is doing:
%       teal   = cruising
%       red    = obstacle within OBS_NEAR metres
%       purple = waypoint within WP_NEAR metres
%
%   Save this file in the sim_env folder. It is called from the 'sensor_viz'
%   MATLAB Function block through coder.extrinsic. Do NOT paste it into a block.

% ---------- Settings ----------
OBS_NEAR  = 2.0;    % [m] obstacle distance that turns the display red
WP_NEAR   = 2.0;    % [m] waypoint distance that turns the display purple
R_MAX     = 5;      % [m] lidar max range (match the Lidar Sensor block)
N_WAVES   = 3;      % number of pulse arcs
WAVE_STEP = 0.2;    % [m] how far the arcs move per call

COL_CRUISE   = [0.00 0.62 0.62];
COL_OBSTACLE = [0.90 0.12 0.12];
COL_WAYPOINT = [0.58 0.20 0.85];

BEAM_ALPHA  = 0.25; % beam transparency (0 = invisible, 1 = solid)
AREA_ALPHA  = 0.10; % scan-area fill transparency
WAVE_ALPHA  = 0.70; % pulse-arc transparency

persistent ax hArea hBeams hHits hWaves hLabel k

% ---------- Find the Robot Visualization axes ----------
fig = findobj('type', 'figure', 'tag', 'RobotVisualization');
if isempty(fig)
    return
end
axNow = findobj(fig(1), 'type', 'axes');
if isempty(axNow)
    return
end
axNow = axNow(1);

nB = numel(scanAngles);
needNew = isempty(ax) || ~isgraphics(ax) || ax ~= axNow || ...
          isempty(hBeams) || ~all(isgraphics(hBeams)) || numel(hBeams) ~= nB;
if needNew
    ax = axNow;
    hold(ax, 'on');
    hArea = patch(ax, NaN, NaN, COL_CRUISE, 'EdgeColor', 'none', ...
                  'FaceAlpha', AREA_ALPHA, 'HandleVisibility', 'off');
    hBeams = gobjects(nB, 1);
    for i = 1:nB
        hBeams(i) = plot(ax, NaN, NaN, '-', 'LineWidth', 0.6, 'HandleVisibility', 'off');
    end
    hHits = plot(ax, NaN, NaN, 'o', 'MarkerSize', 3, 'LineStyle', 'none', ...
                 'HandleVisibility', 'off');
    hWaves = gobjects(N_WAVES, 1);
    for i = 1:N_WAVES
        hWaves(i) = plot(ax, NaN, NaN, '-', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    end
    hLabel = text(ax, NaN, NaN, '', 'FontWeight', 'bold', 'FontSize', 9);
    k = 0;
end
k = k + 1;

% ---------- State colour: obstacle beats waypoint beats cruise ----------
if d_obs < OBS_NEAR
    col = COL_OBSTACLE;  label = 'OBSTACLE';
elseif dist_wp < WP_NEAR
    col = COL_WAYPOINT;  label = 'WAYPOINT';
else
    col = COL_CRUISE;    label = '';
end

x = pose(1);  y = pose(2);  th = pose(3);
r = ranges(:)';
hit = ~isnan(r);
r(~hit) = R_MAX;

ang = th + scanAngles(:)';
bx = x + r.*cos(ang);
by = y + r.*sin(ang);

% ---------- Scan area (what the lidar can see) ----------
set(hArea, 'XData', [x, bx, x], 'YData', [y, by, y], 'FaceColor', col);

% ---------- Beams (faint) and hit dots ----------
for i = 1:nB
    set(hBeams(i), 'XData', [x, bx(i)], 'YData', [y, by(i)], ...
                   'Color', [col, BEAM_ALPHA]);
end
set(hHits, 'XData', bx(hit), 'YData', by(hit), ...
           'MarkerEdgeColor', col, 'MarkerFaceColor', col);

% ---------- Pulse arcs, clipped at the walls ----------
phi_rel = linspace(scanAngles(1), scanAngles(end), 60);
wall_r  = interp1(scanAngles(:)', r, phi_rel, 'linear');
for i = 1:N_WAVES
    rr = mod(WAVE_STEP*k + (i-1)*R_MAX/N_WAVES, R_MAX);
    px = x + rr*cos(th + phi_rel);
    py = y + rr*sin(th + phi_rel);
    blocked = rr > wall_r;
    px(blocked) = NaN;
    py(blocked) = NaN;
    fade = 1 - rr/R_MAX;                      % arcs fade as they travel out
    set(hWaves(i), 'XData', px, 'YData', py, ...
                   'Color', [col, WAVE_ALPHA*fade]);
end

set(hLabel, 'Position', [x + 0.6, y + 0.8, 0], 'String', label, 'Color', col);
drawnow limitrate
end