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
%     CLEARANCE    a ring on the robot's outline and a dotted line to the
%                  nearest return, labelled with its distance; both take the
%                  colour of that range
%     ARENA        the figure itself is restyled once per run to look like a
%                  video-game level: dark tiled floor, neon-edged walls, and
%                  the red/green/blue obstacles drawn as cherries, green
%                  apples and blueberries. The 'kong' style is a Donkey
%                  Kong level instead: girder walls, the obstacles as
%                  barrels, the waypoints as Pauline's lost hat, parasol
%                  and purse, and Pauline herself at the last waypoint
%
%   Save this file in the sim_env folder. It is called from the 'lidar_viz'
%   MATLAB Function block through coder.extrinsic. Do NOT paste it into a block.

% ---------- Behaviour ----------
R_MAX     = 5;       % [m] lidar max range (match the Lidar Sensor block)
RING_R    = 0.25;    % [m] ring radius = robot radius, so the ring recolours the
                     % robot's outline. (It used to sit at the 0.40 m clearance
                     % distance, but obstacles are points and their sprites are
                     % drawn larger, so that ring cut across the sprite on a
                     % close but legal pass.)
R_CLOSE   = 0.5;     % [m] range drawn fully red
R_CLEAR   = 2.5;     % [m] range drawn fully green
CELL      = 0.10;    % [m] mapped-wall resolution (one remembered point per cell)
JUMP      = 2.0;     % [m] a pose jump this large means a new run: clear the map
MAX_FPS   = 30;      % redraw cap so the overlay never slows the simulation

% ---------- Look ----------
FIELD_COL   = [0.30 0.92 1.00];    % bright cyan: reads on the dark arena floors
FIELD_ALPHA = 0.30;
MAP_COL     = [1.00 0.45 0.00];
RAMP        = [0.86 0.10 0.10;    % close
               1.00 0.66 0.00;    % caution
               0.10 0.66 0.30];   % clear
ARENA_STYLE = 'kong';    % arena colourway: 'tron' 'arcade' 'grass' 'synth' 'dungeon' 'kong'
                        % or 'off' to leave the visualizer's plain white map alone

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
    if ~strcmp(ARENA_STYLE, 'off')
        style_arena(ax, ARENA_STYLE);
    end
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
set(h.ring, 'XData', x + RING_R*h.cu, 'YData', y + RING_R*h.su, 'Color', col);

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
h.surface = line(ax, NaN, NaN, 'LineWidth', 2.2, 'Color', fieldCol, common{:});
h.near    = line(ax, NaN, NaN, 'LineStyle', ':', 'LineWidth', 1.4, common{:});
h.returns = patch(ax, 'XData', NaN, 'YData', NaN, 'FaceColor', 'none', ...
                  'EdgeColor', 'none', 'Marker', 'o', 'MarkerSize', 5, ...
                  'MarkerFaceColor', 'flat', 'MarkerEdgeColor', [0.15 0.15 0.15], ...
                  'FaceVertexCData', [0 0 0], common{:});
h.ring    = line(ax, NaN, NaN, 'LineWidth', 1.5, common{:});
h.label   = text(ax, NaN, NaN, '', 'FontWeight', 'bold', 'FontSize', 9, ...
                 'BackgroundColor', ax.Color, 'Margin', 1, common{:});

th   = linspace(0, 2*pi, 48);
h.cu = cos(th);
h.su = sin(th);
end

% =====================================================================
function style_arena(ax, style)
% Video-game look for the Robot Visualization axes: dark tiled floor, walls
% with a neon edge and a soft glow, bright line colours. Purely cosmetic;
% runs once per axes (the visualizer makes new axes at the start of a run).

if isappdata(ax, 'arenaStyled'), return; end
setappdata(ax, 'arenaStyled', true);

% BG figure, INK axis text, FLOOR_A/B 1 m checker tiles, SEAM tile seams,
% WALL fill, EDGE wall outline, GLOW light spilling from walls onto the floor,
% TRUSS cross-bracing drawn inside the walls (empty = plain walls)
TRUSS = [];
switch style
    case 'kong'      % Donkey Kong: black level, red steel girders
        BG = [0 0 0];             INK = [0.35 0.90 1.00];
        FLOOR_A = [0 0 0];        FLOOR_B = [0.03 0.02 0.05];
        SEAM = [0.07 0.05 0.12];  WALL = [0.80 0.10 0.26];
        EDGE = [1.00 0.45 0.60];  GLOW = [0.22 0.02 0.07];
        TRUSS = [0.30 0.02 0.10];
    case 'arcade'    % black maze, blue walls, yellow text
        BG = [0 0 0];             INK = [1.00 0.85 0.10];
        FLOOR_A = [0 0 0];        FLOOR_B = [0.03 0.03 0.05];
        SEAM = [0.06 0.06 0.10];  WALL = [0.02 0.03 0.30];
        EDGE = [0.20 0.35 1.00];  GLOW = [0.00 0.05 0.35];
    case 'grass'     % top-down adventure: grass floor, stone walls
        BG = [0.10 0.16 0.10];    INK = [0.93 0.93 0.80];
        FLOOR_A = [0.33 0.62 0.27]; FLOOR_B = [0.37 0.67 0.30];
        SEAM = [0.31 0.58 0.26];  WALL = [0.50 0.48 0.44];
        EDGE = [0.24 0.22 0.20];  GLOW = [-0.12 -0.16 -0.10];   % wall shadow
    case 'synth'     % synthwave: purple floor, hot pink walls
        BG = [0.06 0.02 0.10];    INK = [1.00 0.60 0.90];
        FLOOR_A = [0.09 0.03 0.16]; FLOOR_B = [0.12 0.04 0.20];
        SEAM = [0.22 0.08 0.34];  WALL = [0.20 0.03 0.25];
        EDGE = [1.00 0.20 0.70];  GLOW = [0.55 0.05 0.40];
    case 'dungeon'   % stone floor, torch-lit walls
        BG = [0.05 0.04 0.03];    INK = [0.95 0.80 0.55];
        FLOOR_A = [0.16 0.14 0.12]; FLOOR_B = [0.19 0.17 0.14];
        SEAM = [0.11 0.10 0.08];  WALL = [0.32 0.24 0.17];
        EDGE = [1.00 0.70 0.25];  GLOW = [0.45 0.22 0.02];
    otherwise        % 'tron': dark floor, neon cyan walls
        BG = [0.02 0.03 0.07];    INK = [0.55 0.90 1.00];
        FLOOR_A = [0.035 0.050 0.100]; FLOOR_B = [0.050 0.070 0.135];
        SEAM = [0.070 0.120 0.210]; WALL = [0.100 0.070 0.250];
        EDGE = [0.150 0.950 1.000]; GLOW = [0.000 0.450 0.600];
end

set(ancestor(ax, 'figure'), 'Color', BG);
set(ax, 'Color', BG, 'XColor', INK, 'YColor', INK, 'GridColor', INK, ...
        'GridAlpha', 0.15, 'FontName', 'monospaced', 'Box', 'on', ...
        'LineWidth', 1.2, 'Layer', 'top');
set([ax.Title, ax.XLabel, ax.YLabel], 'Color', INK);

% ---- Repaint the occupancy map image ----
img = findobj(ax, 'Type', 'image');
if ~isempty(img)
    img = img(1);
    C = double(img.CData);
    if ndims(C) == 3
        lum = mean(C, 3);
    else
        lum = C;
        cm  = colormap(ax);
        if mean(cm(1,:)) > mean(cm(end,:))   % low values drawn bright
            lum = -C;
        end
    end
    wall = lum < (min(lum(:)) + max(lum(:)))/2;   % walls are the dark cells
    [nr, nc] = size(wall);

    ppm = 10;                                     % pixels per metre
    if numel(img.XData) > 1 && diff(img.XData([1 end])) > 0
        ppm = max(1, round((nc - 1)/diff(img.XData([1 end]))));
    end
    [cc, rr] = meshgrid(0:nc-1, 0:nr-1);
    checker  = mod(floor(cc/ppm) + floor(rr/ppm), 2);
    seam     = mod(cc, ppm) == 0 | mod(rr, ppm) == 0;
    k        = exp(-(-6:6).^2/18);  k = k/sum(k);
    glow     = conv2(k, k, double(wall), 'same');
    edge     = wall & conv2(double(wall), ones(3), 'same') < 9;

    rgb = zeros(nr, nc, 3);
    for c = 1:3
        ch = FLOOR_A(c) + (FLOOR_B(c) - FLOOR_A(c))*checker;
        ch(seam) = SEAM(c);
        ch = ch + GLOW(c)*glow;
        ch(wall) = WALL(c);
        if ~isempty(TRUSS)      % girder cross-bracing, one X every half metre
            half = max(2, round(ppm/2));
            ch(wall & (mod(cc + rr, half) == 0 | mod(cc - rr, half) == 0)) = TRUSS(c);
        end
        ch(edge) = EDGE(c);
        rgb(:,:,c) = min(1, max(0, ch));
    end
    img.CData = rgb;
end

% ---- Pure blue/green/red lines are hard to read on a coloured floor ----
if strcmp(style, 'grass')
    swap = [0 0 1, 0.05 0.15 0.75;
            0 1 0, 1.00 0.95 0.30;
            1 0 0, 0.85 0.05 0.10];
else
    swap = [0 0 1, 0.35 0.85 1.00;
            0 1 0, 0.25 1.00 0.45;
            1 0 0, 1.00 0.25 0.50];
end
ln = findobj(ax, 'Type', 'line');
for i = 1:numel(ln)
    j = find(all(abs(swap(:,1:3) - ln(i).Color) < 1e-6, 2), 1);
    if ~isempty(j)
        ln(i).Color = swap(j, 4:6);
    end
end

% ---- Obstacles as fruit instead of plain squares (label = colour) ----
obs = [];
try
    obs = double(evalin('base', 'obstacles'));
catch
end
if size(obs, 2) >= 3
    set(findobj(ax, 'Type', 'scatter'), 'Visible', 'off');
    for i = 1:size(obs, 1)
        if strcmp(style, 'kong')
            draw_barrel(ax, obs(i,1), obs(i,2), obs(i,3));
        else
            draw_fruit(ax, obs(i,1), obs(i,2), obs(i,3));
        end
    end
end

% ---- Kong: waypoints as Pauline's lost items, Pauline at the last one ----
if strcmp(style, 'kong')
    wps = [];
    try
        wps = double(evalin('base', 'wp_ordered'));
    catch
    end
    if size(wps, 2) >= 2
        set(findobj(ax, 'Type', 'line', 'Marker', 'x'), 'Visible', 'off');
        for i = 1:size(wps, 1)
            draw_pickup(ax, wps(i,1), wps(i,2), i, size(wps, 1));
        end
    end
end
end

% =====================================================================
function draw_barrel(ax, x, y, label)
% Barrel sprite centred on an obstacle, seen side-on like the arcade game:
% flat top with a lid, bulging sides, metal hoops and wooden staves.
% 1 red, 2 green, 3 blue.
S   = 1.0;                     % [m] sprite scale; cosmetic, not the obstacle size.
                               % The sprite reaches ~0.44 m from the obstacle; the
                               % avoidance block keeps the robot's centre ~0.70 m
                               % away, so the drawn robot (0.25 m radius) passes
                               % clear of it
OUT  = [1.00 0.95 0.80];       % light outline: reads on the black floor
HOOP = [0.16 0.14 0.14];       % dark iron
common = {'HandleVisibility', 'off', 'HitTest', 'off', ...
          'PickableParts', 'none', 'Tag', 'arenaFruit'};
switch label
    case 1,    wood = [0.88 0.24 0.12];  dark = [0.50 0.10 0.05];  lid = [1.00 0.52 0.38];
    case 2,    wood = [0.26 0.72 0.24];  dark = [0.09 0.38 0.10];  lid = [0.58 0.92 0.50];
    otherwise, wood = [0.24 0.46 0.96];  dark = [0.08 0.18 0.55];  lid = [0.58 0.76 1.00];
end

BOT = -0.44;  TOP = 0.36;                       % body, lid sits on top
MID = (BOT + TOP)/2;  HALF = (TOP - BOT)/2;
wid = @(yy) 0.25 + 0.11*(1 - ((yy - MID)/HALF).^2);   % half-width: bulges in the middle
ys  = linspace(BOT, TOP, 17);
slab = @(y0, y1, col, edge) patch(ax, ...
    x + S*[wid(linspace(y0, y1, 7)), -wid(linspace(y1, y0, 7))], ...
    y + S*[linspace(y0, y1, 7), linspace(y1, y0, 7)], ...
    col, 'EdgeColor', edge, 'LineWidth', 0.5, common{:});

% body
patch(ax, x + S*[wid(ys), -wid(fliplr(ys))], y + S*[ys, fliplr(ys)], wood, ...
      'EdgeColor', 'none', common{:});
% staves: follow the bulge of the side
for f = [-0.62 -0.22 0.22 0.62]
    line(ax, x + S*f*wid(ys), y + S*ys, 'Color', dark, 'LineWidth', 0.75, common{:});
end
% highlight down the left staves
patch(ax, x + S*[-0.56*wid(ys), -0.30*wid(fliplr(ys))], y + S*[ys, fliplr(ys)], ...
      [1 1 1], 'EdgeColor', 'none', 'FaceAlpha', 0.22, common{:});
% iron hoops: two round the belly, one at each end
slab(BOT,        BOT + 0.07, HOOP, 'none');
slab(MID - 0.24, MID - 0.15, HOOP, 'none');
slab(MID + 0.15, MID + 0.24, HOOP, 'none');
slab(TOP - 0.07, TOP,        HOOP, 'none');
% lid, seen slightly from above
th = linspace(0, 2*pi, 24);
patch(ax, x + S*wid(TOP)*cos(th), y + S*(TOP + 0.08*sin(th)), lid, ...
      'EdgeColor', HOOP, 'LineWidth', 0.75, common{:});
% outline last so it sits on top of the hoops
th = linspace(pi, 2*pi, 12);
line(ax, x + S*[wid(ys), wid(TOP)*cos(fliplr(th)), -wid(fliplr(ys)), wid(BOT)], ...
         y + S*[ys, TOP - 0.08*sin(fliplr(th)), fliplr(ys), BOT], ...
     'Color', OUT, 'LineWidth', 1.2, common{:});
end

% =====================================================================
function draw_pickup(ax, x, y, k, n)
% Sprite for waypoint K of N. The intermediate waypoints are Pauline's
% lost items (hat, parasol, purse, repeating); the last one is Pauline.
S    = 1.4;
OUT  = [0.10 0.05 0.03];
PINK = [1.00 0.45 0.72];
DARK = [0.75 0.15 0.45];
th   = linspace(0, 2*pi, 28);
cu   = cos(th);  su = sin(th);
common = {'HandleVisibility', 'off', 'HitTest', 'off', ...
          'PickableParts', 'none', 'Tag', 'arenaFruit'};
blob = @(cx, cy, rx, ry, col) patch(ax, x + S*(cx + rx*cu), y + S*(cy + ry*su), ...
            col, 'EdgeColor', OUT, 'LineWidth', 0.5, common{:});
poly = @(px, py, col) patch(ax, x + S*px, y + S*py, col, 'EdgeColor', OUT, ...
            'LineWidth', 0.5, common{:});
pen  = @(px, py, col, w) line(ax, x + S*px, y + S*py, 'Color', col, ...
            'LineWidth', w, common{:});

if k == n                      % Pauline
    blob(0.00, 0.36, 0.24, 0.24, [0.80 0.40 0.10]);                 % hair
    poly([-0.10 0.10 0.34 -0.34], [0.12 0.12 -0.50 -0.50], PINK);   % dress
    pen([-0.10 -0.36], [0.06 0.30], [1.00 0.82 0.68], 2);           % arms up
    pen([ 0.10  0.36], [0.06 0.30], [1.00 0.82 0.68], 2);
    blob(0.00, 0.30, 0.15, 0.15, [1.00 0.82 0.68]);                 % face
    text(ax, x, y + S*0.80, 'HELP!', 'Color', [0.35 0.90 1.00], ...
         'FontWeight', 'bold', 'FontSize', 9, 'FontName', 'monospaced', ...
         'HorizontalAlignment', 'center', common{:});
else
    switch mod(k - 1, 3)
        case 0                 % hat
            blob(0.00, -0.10, 0.42, 0.11, PINK);
            blob(0.00,  0.06, 0.22, 0.20, PINK);
            pen([-0.21 0.21], [-0.03 -0.03], DARK, 2);
        case 1                 % parasol
            a = linspace(0, pi, 16);
            poly(0.42*cos(a), 0.10 + 0.34*sin(a), PINK);
            pen([0 0], [0.10 -0.42], [0.95 0.95 0.95], 1.5);
            pen([0 0.08 0.14], [-0.42 -0.50 -0.42], [0.95 0.95 0.95], 1.5);
            pen([-0.14 0 0.14; 0 0 0]', [0.10 0.10 0.10; 0.42 0.44 0.42]', DARK, 0.5);
        otherwise              % purse
            a = linspace(0, pi, 16);
            pen(0.18*cos(a), 0.10 + 0.26*sin(a), DARK, 2);
            poly([-0.24 0.24 0.32 -0.32], [0.12 0.12 -0.36 -0.36], PINK);
            blob(0.00, 0.02, 0.05, 0.05, [1.00 0.90 0.30]);
    end
    text(ax, x + S*0.45, y + S*0.40, sprintf('%d', k), 'Color', [1 1 1], ...
         'FontWeight', 'bold', 'FontSize', 9, 'FontName', 'monospaced', common{:});
end
end

% =====================================================================
function draw_fruit(ax, x, y, label)
% Small vector sprite centred on an obstacle:
%   1 (red) cherries, 2 (green) apple, 3 (blue) blueberry.
S   = 0.6;                     % [m] sprite scale. Cosmetic, but sized on purpose: the
                               % sprite reaches at most ~0.27 m from the obstacle, so
                               % the drawn robot (0.25 m radius) never overlaps it
                               % when passing at a legal distance
OUT = [0.10 0.05 0.03];        % outline
th  = linspace(0, 2*pi, 28);
cu  = cos(th);  su = sin(th);
common = {'HandleVisibility', 'off', 'HitTest', 'off', ...
          'PickableParts', 'none', 'Tag', 'arenaFruit'};
blob  = @(cx, cy, rx, ry, col) patch(ax, x + S*(cx + rx*cu), y + S*(cy + ry*su), ...
            col, 'EdgeColor', OUT, 'LineWidth', 0.5, common{:});
shine = @(cx, cy, r) patch(ax, x + S*(cx + r*cu), y + S*(cy + r*su), ...
            [1 1 1], 'EdgeColor', 'none', 'FaceAlpha', 0.75, common{:});
stalk = @(px, py, col) line(ax, x + S*px, y + S*py, 'Color', col, ...
            'LineWidth', 1.5, common{:});

switch label
    case 1      % cherries
        stalk([-0.26 0.10 0.28], [-0.05 0.58 -0.10], [0.45 0.65 0.20]);
        blob( 0.30,  0.58, 0.20, 0.08, [0.25 0.65 0.20]);
        blob(-0.26, -0.26, 0.28, 0.28, [0.86 0.06 0.14]);
        blob( 0.28, -0.30, 0.28, 0.28, [0.86 0.06 0.14]);
        shine(-0.35, -0.16, 0.07);
        shine( 0.19, -0.20, 0.07);
    case 2      % green apple
        stalk([0 0.07], [0.30 0.60], [0.55 0.35 0.15]);
        blob( 0.25,  0.52, 0.18, 0.08, [0.20 0.60 0.18]);
        blob( 0.00, -0.08, 0.46, 0.42, [0.50 0.82 0.16]);
        shine(-0.18, 0.06, 0.09);
    otherwise   % blueberry
        blob( 0.00,  0.00, 0.44, 0.44, [0.22 0.32 0.88]);
        k  = 0:10;
        rr = 0.17 - 0.09*mod(k, 2);
        patch(ax, x + S*(rr.*cos(pi/2 + k*pi/5)), ...
                  y + S*(0.20 + rr.*sin(pi/2 + k*pi/5)), ...
              [0.07 0.10 0.40], 'EdgeColor', 'none', common{:});
        shine(-0.20, 0.02, 0.08);
end
end
