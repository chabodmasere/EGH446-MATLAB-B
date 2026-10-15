function varargout = obstacle_replan(cmd, varargin)
%OBSTACLE_REPLAN  Re-plan the path around obstacles seen by the Object Detector.
%
%   obstacle_replan('init', legs, wp_route, clr)      from mdl_robot
%   obstacle_replan('step', x_hat, detections, wp_idx) from the Replanner block
%   t = obstacle_replan('stats')                       computation time [s] of
%                                                      every re-plan this run
%
%   Detections are turned into world positions and remembered. Once an
%   obstacle is confirmed, any remaining leg that passes within TRIGGER of it
%   is re-planned: the obstacle is added to the clearance map, A* finds the
%   new route and PSO smooths it. The current leg is re-planned from where
%   the robot is. The new path goes to Guidance through path_store and is
%   drawn on the Robot Visualization figure.
%
%   Obstacle positions only ever come from the detector. Nothing here reads
%   the obstacle list.

    CONFIRM  = 20;    % detector hits before an obstacle is trusted (block runs at 100 Hz)
    COOLDOWN = 100;   % min calls between re-plans (1 s)
    MERGE    = 0.75;  % [m] detections closer than this are the same obstacle
    MOVE     = 0.3;   % [m] re-plan again if an obstacle estimate moves this far
    TRIGGER  = 1.0;   % [m] re-plan a leg that passes closer than this to an obstacle
    OBS_CLR  = 0.8;   % [m] hard clearance to an obstacle centre: 0.55 required
                      %     (0.40 + 0.15 obstacle radius) + 0.25 for detection
                      %     error and corner cutting
    OBS_WIDE = 1.0;   % [m] A* tries this keep-out first, so it routes round the
                      %     open side of an obstacle instead of squeezing past

    persistent S

    switch cmd
        case 'init'
            [legs, route, clr] = varargin{:};
            S = struct('legs', {legs}, 'route', route, 'clr', clr, ...
                       'obs', zeros(0, 2), 'hits', zeros(0, 1), 'used', zeros(0, 2), ...
                       'calls', 0, 'last', -inf, 'n', 0, 'mod', false(numel(legs), 1), ...
                       'cpu', zeros(0, 1));
            path_store('reset');

        case 'step'
            if isempty(S)
                return
            end
            [x_hat, det, wp_idx] = varargin{:};
            S.calls = S.calls + 1;
            S = remember(S, x_hat, det, MERGE);

            conf = S.obs(S.hits >= CONFIRM, :);
            if isempty(conf) || S.calls - S.last < COOLDOWN
                return
            end
            if ~any(is_new(conf, S.used, MOVE))
                return
            end
            S.used = conf;
            S.last = S.calls;
            t0 = tic;
            [S, changed] = replan(S, x_hat, round(wp_idx), conf, TRIGGER, OBS_CLR, OBS_WIDE);
            if changed
                S.cpu(end+1, 1) = toc(t0);
            end

        case 'stats'
            varargout{1} = zeros(0, 1);
            if ~isempty(S)
                varargout{1} = S.cpu;
            end
    end
end

% =====================================================================
function S = remember(S, x_hat, det, merge)
    for i = 1:size(det, 1)
        r = det(i, 1);
        if r <= 0
            continue
        end
        o = [x_hat(1) + r*cos(x_hat(3) + det(i, 2)), x_hat(2) + r*sin(x_hat(3) + det(i, 2))];
        if ~isempty(S.obs)
            [d, j] = min(hypot(S.obs(:, 1) - o(1), S.obs(:, 2) - o(2)));
            if d < merge
                S.obs(j, :) = 0.9*S.obs(j, :) + 0.1*o;
                S.hits(j) = S.hits(j) + 1;
                continue
            end
        end
        S.obs(end+1, :) = o;
        S.hits(end+1, 1) = 1;
    end
end

% =====================================================================
function tf = is_new(conf, used, move)
    tf = true(size(conf, 1), 1);
    for k = 1:size(conf, 1)
        if ~isempty(used)
            tf(k) = min(hypot(used(:, 1) - conf(k, 1), used(:, 2) - conf(k, 2))) > move;
        end
    end
end

% =====================================================================
function [S, changed] = replan(S, x_hat, wp_idx, conf, trigger, obs_clr, obs_wide)
    N  = numel(S.legs);
    i0 = min(max(wp_idx, 1), N);
    robot = x_hat(1:2)';

    % obstacles go into the clearance field, offset so that clr_eff = 0.6
    % means 0.6 m from a wall or obs_clr from an obstacle centre
    [nR, nC] = size(S.clr);
    gx = (0:nC-1)/10;
    gy = (nR-1:-1:0)'/10;
    dob = inf(nR, nC);
    for k = 1:size(conf, 1)
        dob = min(dob, hypot(gx - conf(k, 1), gy - conf(k, 2)));
    end
    clr_eff  = min(S.clr, dob - (obs_clr - 0.6));
    occ_eff  = clr_eff <= 0.6;
    occ_wide = occ_eff | dob <= obs_wide;

    done = [];
    restart = false;
    for i = i0:N
        leg = S.legs{i};
        if i == i0
            [~, k0] = min(hypot(leg(:, 1) - robot(1), leg(:, 2) - robot(2)));
            ahead = leg(k0:end, :);
            start = robot;
        else
            ahead = leg;
            start = leg(1, :);
        end
        if min_dist(ahead, conf) >= trigger
            continue
        end

        % wide keep-out first, then the hard one if that has no route
        goal = S.route(i+1, :);
        a = [];
        for occ_try = {occ_wide, occ_eff}
            s = snap_free(start, occ_try{1});
            if ~isempty(s) && ~blocked(goal, occ_try{1})
                a = A_star(s, goal, occ_try{1}, false);
            end
            if ~isempty(a)
                break
            end
        end
        if isempty(a)
            continue
        end
        if size(a, 1) < 3
            p = a;
        else
            p = wp_swarm({a}, clr_eff);
        end

        S.legs{i} = p;
        S.mod(i)  = true;
        done(end+1) = i; %#ok<AGROW>
        restart = restart || i == i0;
    end

    changed = ~isempty(done);
    if ~changed
        return
    end

    full = S.legs{1};
    seg  = zeros(N, 1);
    seg(1) = size(full, 1);
    for i = 2:N
        full = [full; S.legs{i}(2:end, :)]; %#ok<AGROW>
        seg(i) = size(full, 1);
    end

    rst = 0;
    if restart
        rst = 1;
        if i0 > 1
            rst = seg(i0-1) + 1;
        end
    end
    path_store('set', full, seg, rst);

    S.n = S.n + 1;
    fprintf('Replan %d: %d obstacle(s) known, re-planned leg(s) %s\n', ...
            S.n, size(conf, 1), mat2str(done));
    draw_replan(S, i0, done, conf, obs_clr);
end

% =====================================================================
function d = min_dist(p, conf)
    d = inf;
    for k = 1:size(conf, 1)
        d = min(d, min(hypot(p(:, 1) - conf(k, 1), p(:, 2) - conf(k, 2))));
    end
end

function tf = blocked(p, occ)
    nR = size(occ, 1);
    tf = occ(nR - round(p(2)*10), round(p(1)*10) + 1);
end

function s = snap_free(p, occ)
% p if it is free, otherwise the nearest free grid point within 1.5 m
    s = [];
    nR = size(occ, 1);
    r0 = nR - round(p(2)*10);
    c0 = round(p(1)*10) + 1;
    if ~occ(r0, c0)
        s = p;
        return
    end
    w  = 15;
    rr = max(1, r0-w):min(nR, r0+w);
    cc = max(1, c0-w):min(size(occ, 2), c0+w);
    [C, R] = meshgrid(cc, rr);
    free = ~occ(rr, cc);
    if ~any(free(:))
        return
    end
    d = hypot(R - r0, C - c0);
    d(~free) = inf;
    [~, k] = min(d(:));
    s = [(C(k) - 1)/10, (nR - R(k))/10];
end

% =====================================================================
function draw_replan(S, i0, done, conf, obs_clr)
% Re-planned legs violet dotted, keep-out rings violet dashed. Only legs
% that differ from the original plan are drawn, from the current leg on.
% The previous drawing is cleared first, so only the current plan shows.
    fig = findobj('Type', 'figure', 'Tag', 'RobotVisualization');
    if isempty(fig), return; end
    ax = findobj(fig(1), 'Type', 'axes');
    if isempty(ax), return; end
    ax = ax(1);

    common = {'HandleVisibility', 'off', 'HitTest', 'off', ...
              'PickableParts', 'none', 'Tag', 'replanOverlay'};
    delete(findall(ax, 'Tag', 'replanOverlay'));   % findall: these handles are hidden

    VIOLET = [0.78 0.45 1.00];
    th = linspace(0, 2*pi, 48);
    for k = 1:size(conf, 1)
        line(ax, conf(k, 1) + obs_clr*cos(th), conf(k, 2) + obs_clr*sin(th), ...
             'LineStyle', '--', 'LineWidth', 1.0, 'Color', VIOLET, common{:});
    end
    for i = i0:numel(S.legs)
        if S.mod(i)
            p = S.legs{i};
            line(ax, p(:, 1), p(:, 2), 'LineStyle', ':', 'LineWidth', 2.4, ...
                 'Color', VIOLET, common{:});
        end
    end
    drawnow limitrate
end
