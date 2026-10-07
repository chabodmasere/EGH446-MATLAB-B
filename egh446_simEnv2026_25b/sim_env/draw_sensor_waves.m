function draw_sensor_waves(pose, ranges, scanAngles, d_obs, dist_wp)
% DRAW_SENSOR_WAVES  Animated "transmit -> bounce -> echo" lidar overlay.
%
%   draw_sensor_waves(pose, ranges, scanAngles, d_obs, dist_wp)
%
%   pose       [x y theta]   robot pose (m, m, rad)
%   ranges     [nB x 1]      lidar ranges (NaN/Inf = no return)
%   scanAngles [nB x 1]      beam angles relative to robot heading (rad)
%   d_obs      scalar        distance to nearest obstacle (m)
%   dist_wp    scalar        distance to current waypoint (m)
%
%   Each pulse follows a full signal round trip:
%     1. TRANSMIT  a solid arc leaves the robot and expands outward
%     2. IMPACT    where the arc meets an obstacle it stops, and a ripple
%                  bursts out of the hit point
%     3. ECHO      a dashed arc reflects off the obstacle and travels back
%                  along each beam, shaped like the obstacle it hit
%     4. RECEIVE   when the echo from the nearest obstacle reaches the robot,
%                  the clearance ring flashes
%   Beams that hit nothing just fade out at max range and send no echo.
%
%   Colour shows what the robot is doing, easing smoothly between states:
%     teal = cruising, red = obstacle near, purple = waypoint near
%
%   Save this file in the sim_env folder. It is called from the 'sensor_viz'
%   MATLAB Function block through coder.extrinsic. Do NOT paste it into a block.

% ---------- Behaviour ----------
OBS_NEAR   = 2.0;    % [m] obstacle distance that turns the display red
WP_NEAR    = 2.0;    % [m] waypoint distance that turns the display purple
HYST       = 0.3;    % [m] extra distance needed to LEAVE a state (no flicker)
R_MAX      = 5;      % [m] lidar max range (match the Lidar Sensor block)
CLEARANCE  = 0.40;   % [m] robot radius + safety margin

% ---------- Pulse animation ----------
N_PULSES   = 3;      % pulses in flight at once
WAVE_SPEED = 5.0;    % [m/s] signal speed (wall-clock), out AND back
GAP        = 1.5;    % [m] idle distance between round trips
N_ARC      = 120;    % points per arc (smoothness)
RIPPLE_LEN = 0.5;    % [m] how long an impact ripple lasts (in pulse travel)
RIPPLE_R   = 0.35;   % [m] final radius of an impact ripple
GLOW_W     = 0.35;   % [m] width of the receive flash

% ---------- Look ----------
MAX_FPS    = 30;     % redraw cap so the overlay never slows the simulation
EASE       = 0.25;   % colour blend per frame (1 = instant)
COLS = [0.00 0.62 0.62;   % 0 cruise
        0.58 0.20 0.85;   % 1 waypoint
        0.90 0.12 0.12];  % 2 obstacle
AREA_ALPHA   = 0.08;
BEAM_ALPHA   = 0.15;
OUT_ALPHA    = 0.85; % transmit arc
ECHO_ALPHA   = 0.60; % returning echo
RIPPLE_ALPHA = 0.55; % impact ripples

persistent h st

% ---------- Find the Robot Visualization axes ----------
fig = findobj('Type', 'figure', 'Tag', 'RobotVisualization');
if isempty(fig), return; end
ax = findobj(fig(1), 'Type', 'axes');
if isempty(ax), return; end
ax = ax(1);

% ---------- (Re)create graphics only when needed ----------
if isempty(h) || ~isgraphics(h.ax) || h.ax ~= ax || ~isgraphics(h.beams)
    h  = create_graphics(ax, N_PULSES, COLS(1,:), AREA_ALPHA);
    st = struct('mode', 0, 'col', COLS(1,:), 't0', tic, 'tLast', -Inf);
end

% ---------- Frame-rate cap ----------
t = toc(st.t0);
if t - st.tLast < 1/MAX_FPS
    return
end
st.tLast = t;

% ---------- State with hysteresis: obstacle > waypoint > cruise ----------
nearObs = d_obs   < OBS_NEAR + HYST*(st.mode == 2);
nearWp  = dist_wp < WP_NEAR  + HYST*(st.mode == 1);
if nearObs
    st.mode = 2;
elseif nearWp
    st.mode = 1;
else
    st.mode = 0;
end
st.col = st.col + EASE*(COLS(st.mode+1, :) - st.col);
col = st.col;

% ---------- Beam geometry ----------
x = pose(1);  y = pose(2);  th = pose(3);
a   = scanAngles(:)';
r   = ranges(:)';
nB  = numel(r);
hit = isfinite(r) & r < R_MAX;
r(~hit) = R_MAX;

bx = x + r.*cos(th + a);
by = y + r.*sin(th + a);
hitIdx = find(hit);

% ---------- Static layers: scan area, faint beams, hit dots ----------
set(h.area, 'XData', [x, bx], 'YData', [y, by], 'FaceColor', col);

BX = [repmat(x, 1, nB); bx; nan(1, nB)];
BY = [repmat(y, 1, nB); by; nan(1, nB)];
set(h.beams, 'XData', BX(:)', 'YData', BY(:)', 'Color', [col, BEAM_ALPHA]);

set(h.hits, 'XData', bx(hit), 'YData', by(hit), ...
    'MarkerEdgeColor', col, 'MarkerFaceColor', col);

% ---------- Nearest return ----------
if ~isempty(hitIdx)
    [rmin, j] = min(r(hitIdx));
    i = hitIdx(j);
    set(h.near, 'XData', [x, bx(i)], 'YData', [y, by(i)], 'Color', [col, 0.6]);
else
    rmin = Inf;
    set(h.near, 'XData', NaN, 'YData', NaN);
end

% ---------- Pulses: transmit -> impact -> echo -> receive ----------
phi    = linspace(a(1), a(end), N_ARC);
wall   = interp1(a, r, phi, 'linear', R_MAX);              % wall distance per arc point
hitPhi = interp1(a, double(hit), phi, 'nearest', 0) > 0.5; % does this direction echo?
cphi   = cos(th + phi);
sphi   = sin(th + phi);

cycle = 2*R_MAX + GAP;     % one full round trip + pause, in metres of travel
glow  = 0;

for p = 1:N_PULSES
    % s = how far this pulse has travelled since it was transmitted
    s = mod(WAVE_SPEED*t + (p-1)*cycle/N_PULSES, cycle);

    % 1. TRANSMIT: a circle of radius s, cut off wherever it has reached a wall
    ro = s*ones(1, N_ARC);
    ro(s > wall) = NaN;
    aOut = OUT_ALPHA * min(1, s/0.4) * max(0, 1 - s/R_MAX)^0.6;  % fade in, decay out
    set(h.out(p), 'XData', x + ro.*cphi, 'YData', y + ro.*sphi, ...
        'Color', [col, aOut]);

    % 3. ECHO: past the wall the pulse travels back, so its radius is 2*wall - s
    re = 2*wall - s;
    valid = hitPhi & (s > wall) & (re > 0);
    re(~valid) = NaN;
    aEcho = ECHO_ALPHA * max(0.2, 1 - s/(2*R_MAX));             % weaker on return
    set(h.echo(p), 'XData', x + re.*cphi, 'YData', y + re.*sphi, ...
        'Color', [col, aEcho]);

    % 2. IMPACT: a ripple grows out of each hit point just after the pulse arrives
    if ~isempty(hitIdx)
        age = s - r(hitIdx);
        act = age > 0 & age < RIPPLE_LEN;
        if any(act)
            rad = RIPPLE_R * age(act) / RIPPLE_LEN;
            RX  = bx(hitIdx(act)) + h.cr .* rad;   % one small circle per hit, NaN-separated
            RY  = by(hitIdx(act)) + h.sr .* rad;
            set(h.rip(p), 'XData', RX(:)', 'YData', RY(:)', ...
                'Color', [col, RIPPLE_ALPHA]);
        else
            set(h.rip(p), 'XData', NaN, 'YData', NaN);
        end
    else
        set(h.rip(p), 'XData', NaN, 'YData', NaN);
    end

    % 4. RECEIVE: flash when the nearest echo gets home (s = 2*rmin)
    if isfinite(rmin)
        glow = max(glow, exp(-((s - 2*rmin)/GLOW_W)^2));
    end
end

% ---------- Clearance ring doubles as the receiver ----------
set(h.ring, 'XData', x + CLEARANCE*h.cu, 'YData', y + CLEARANCE*h.su, ...
    'Color', [col, 0.45 + 0.55*glow], 'LineWidth', 1.0 + 2.5*glow);

% ---------- Status label ----------
switch st.mode
    case 2,    label = sprintf('OBSTACLE  %.2f m', d_obs);
    case 1,    label = sprintf('WAYPOINT  %.2f m', dist_wp);
    otherwise, label = '';
end
set(h.label, 'Position', [x + 0.6, y + 0.8, 0], 'String', label, 'Color', col);

drawnow limitrate
end

% =====================================================================
function h = create_graphics(ax, nPulses, col, areaAlpha)
% Builds every overlay object once. Low-level patch/line/text never clear
% the axes, so there is no need to touch the axes' hold state.

common = {'HandleVisibility', 'off', 'HitTest', 'off', ...
          'PickableParts', 'none', 'Tag', 'sensorOverlay'};

delete(findobj(ax, 'Tag', 'sensorOverlay'));   % remove leftovers from old runs

h.ax    = ax;
h.area  = patch(ax, NaN, NaN, col, 'EdgeColor', 'none', ...
                'FaceAlpha', areaAlpha, common{:});
h.beams = line(ax, NaN, NaN, 'LineWidth', 0.5, common{:});
h.near  = line(ax, NaN, NaN, 'LineStyle', ':', 'LineWidth', 1.2, common{:});

h.out  = gobjects(nPulses, 1);
h.echo = gobjects(nPulses, 1);
h.rip  = gobjects(nPulses, 1);
for p = 1:nPulses
    h.out(p)  = line(ax, NaN, NaN, 'LineWidth', 1.8, common{:});
    h.echo(p) = line(ax, NaN, NaN, 'LineWidth', 1.2, 'LineStyle', '--', common{:});
    h.rip(p)  = line(ax, NaN, NaN, 'LineWidth', 0.8, common{:});
end

h.hits  = line(ax, NaN, NaN, 'LineStyle', 'none', 'Marker', 'o', ...
               'MarkerSize', 3, common{:});
h.ring  = line(ax, NaN, NaN, 'LineWidth', 1.0, common{:});
h.label = text(ax, NaN, NaN, '', 'FontWeight', 'bold', 'FontSize', 9, common{:});

% Unit circles: smooth one for the clearance ring, small NaN-terminated
% column for impact ripples (the NaN separates one ripple from the next)
th   = linspace(0, 2*pi, 48);
h.cu = cos(th);
h.su = sin(th);
tr   = linspace(0, 2*pi, 20)';
h.cr = [cos(tr); NaN];
h.sr = [sin(tr); NaN];
end