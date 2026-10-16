function analyse_missions(baseDir, sweepDir, outDir, figDir)
% ANALYSE_MISSIONS  Metrics and figures for the Part B report.
%
%   analyse_missions(baseDir, sweepDir, outDir, figDir)
%
%   baseDir   base_XX.mat files: the 25 seeded missions (batch_missions.m)
%   sweepDir  v15_XX, v20_XX, cap08_XX files: the parameter sweep ('' to skip)
%   outDir    metrics.csv, sweep.csv and summary.txt are written here
%   figDir    report figures are written here
%
%   Ground truth is 'pose_clean', the vehicle pose BEFORE the simulator
%   adds its sensor noise (logged by batch_missions.m from the output of
%   the Four-Wheel Steering Simulation block). Clearance, cross-track error
%   and estimation error are measured from it. Clearance is also given from
%   the Kalman estimate x_hat (what the vehicle believed) and from the noisy
%   'pose_true' signal the sensors see (the minimum of a noisy signal reads
%   low). Clearance is from the vehicle centre to a wall face or to an
%   obstacle's surface (obstacles have a 0.15 m radius, unit FAQ item 12),
%   and is compared with the required 0.40 m.

CLEAR = 0.40;  OBS_R = 0.15;
if ~exist(outDir, 'dir'), mkdir(outDir); end
if ~exist(figDir, 'dir'), mkdir(figDir); end
f0  = dir(fullfile(baseDir, 'base_*.mat'));
m1  = load(fullfile(f0(1).folder, 'map.mat'));     % written once per batch by batch_missions
obs = m1.obstacles;  lm = m1.logical_map;

[T, M] = run_set(f0, obs, lm, OBS_R);
writetable(T, fullfile(outDir, 'metrics.csv'));

fid = fopen(fullfile(outDir, 'summary.txt'), 'w');
pr = @(varargin) fprintf(fid, varargin{:});
N = height(T);
pr('missions                      %d\n', N);
pr('completed                     %d (%.0f %%)\n', sum(T.complete), 100*mean(T.complete));
st = @(name, v) pr('%-34s mean %.3f  median %.3f  min %.3f  max %.3f  std %.3f\n', name, mean(v, 'omitnan'), median(v, 'omitnan'), min(v), max(v), std(v, 'omitnan'));
st('mission time [s]', T.time);
st('final stop error [m]', T.final_err);
st('min obstacle clearance [m]', T.min_obs);
st('min wall clearance [m]', T.min_wall);
st('min obstacle clearance, raw [m]', T.min_obs_raw);
st('min wall clearance, raw [m]', T.min_wall_raw);
st('min obstacle clearance, estimate [m]', T.min_obs_est);
st('min wall clearance, estimate [m]', T.min_wall_est);
st('estimate vs raw pose RMS [m]', T.est_vs_raw);
st('position error RMS (est vs true) [m]', T.est_rms);
st('position error max (est vs true) [m]', T.est_max);
st('heading error RMS (est vs true) [deg]', T.head_rms);
st('raw pose error RMS (raw vs true) [m]', T.raw_rms);
st('wall repulsion active [s]', T.wall_s);
st('re-plan compute time, max [s]', T.cpu_max);
st('re-plan compute time, mean [s]', T.cpu_mean);
st('PSO max curvature [1/m]', T.kappa_max);
st('raw pose noise sigma/axis [m]', T.raw_sigma);
st('capture error wp 1-4 [m]', T.cap_mid);
st('capture error final wp [m]', T.cap_final);
st('closest approach wp 1-4 [m]', T.approach_mid);
st('RMS cross-track, active path [m]', T.rms_xte);
st('max cross-track, active path [m]', T.max_xte);
st('A* length [m]', T.astar_len);
st('planned (PSO) length [m]', T.plan_len);
st('driven length [m]', T.driven_len);
st('driven/planned - 1 [%]', 100*(T.driven_len./T.plan_len - 1));
st('PSO/A* - 1 [%]', 100*(T.plan_len./T.astar_len - 1));
st('mean speed [m/s]', T.driven_len./T.time);
st('re-plans', T.replans);
st('reactive active [s]', T.avoid_s);
pr('missions with reactive layer active      %d\n', sum(T.avoid_s > 0));
pr('missions below %.2f m obstacle clearance %d\n', CLEAR, sum(T.min_obs < CLEAR));
pr('missions below %.2f m wall clearance     %d\n', CLEAR, sum(T.min_wall < CLEAR));
pr('missions below %.2f m by raw pose: obstacle %d, wall %d\n', CLEAR, sum(T.min_obs_raw < CLEAR), sum(T.min_wall_raw < CLEAR));
pr('missions below %.2f m by estimate: obstacle %d, wall %d\n', CLEAR, sum(T.min_obs_est < CLEAR), sum(T.min_wall_est < CLEAR));
pr('missions with wall repulsion active      %d\n', sum(T.wall_s > 0));
pr('PSO legs above curvature limit 1.0 1/m   %d of %d\n', sum(T.kappa_over), sum(T.pso_legs));
pr('ACO order optimal (brute force)          %d of %d\n', sum(T.aco_opt), N);
pr('PSO legs falling back to A*              %d of %d\n', sum(T.pso_fallback), sum(T.pso_legs));
st('nearest-neighbour tour above optimum [%]', T.nn_gap);
pr('nearest-neighbour optimal in             %d of %d\n', sum(T.nn_gap < 1e-6), N);
st('ACO tour above optimum [%]', T.aco_gap);

% ---------------- parameter sweep ----------------
if ~isempty(sweepDir)
    tags = {'v15', 'v20', 'cap08'};
    names = {'baseline (1.0 m/s, 0.4 m)', 'v_max 1.5 m/s', 'v_max 2.0 m/s', 'capture radius 0.8 m'};
    seeds = [];
    S = cell(1, 4);
    for q = 1:3
        fq = dir(fullfile(sweepDir, [tags{q} '_*.mat']));
        if isempty(fq), continue, end
        S{q+1} = run_set(fq, obs, lm, OBS_R);
        seeds = union(seeds, S{q+1}.seed');
    end
    S{1} = T(ismember(T.seed, seeds), :);
    wv = {'n', 'completed', 'time_mean', 'min_wall_worst', 'min_obs_worst', 'cap_mid_mean', 'rms_xte_mean', 'below_clear', 'approach_mean', 'min_wall_mean', 'final_err_mean'};
    A = nan(4, numel(wv));
    for q = 1:4
        s = S{q};
        if isempty(s) || height(s) == 0, continue, end
        A(q, :) = [height(s), sum(s.complete), mean(s.time), min(s.min_wall), min(s.min_obs), ...
                   mean(s.cap_mid, 'omitnan'), mean(s.rms_xte), sum(s.min_wall < CLEAR | s.min_obs < CLEAR), ...
                   mean(s.approach_mid), mean(s.min_wall), mean(s.final_err)];
    end
    W = [table(string(names(:)), 'VariableNames', {'config'}), array2table(A, 'VariableNames', wv)];
    writetable(W, fullfile(outDir, 'sweep.csv'));
    pr('\nSWEEP over seeds %s\n', mat2str(seeds));
    for q = 1:4
        pr('%-28s n %d  completed %d  time %.1f  worst wall %.3f  worst obs %.3f  cap(1-4) %.3f  rms xte %.3f  below clearance %d\n', ...
           W.config(q), W.n(q), W.completed(q), W.time_mean(q), W.min_wall_worst(q), W.min_obs_worst(q), W.cap_mid_mean(q), W.rms_xte_mean(q), W.below_clear(q));
        pr('%-28s closest approach %.3f  mean min wall %.3f  final err %.3f\n', '', W.approach_mean(q), W.min_wall_mean(q), W.final_err_mean(q));
    end
end

% showcase mission: reactive layer active, RMS cross-track nearest the median
cand = find(T.avoid_s > 0);
if isempty(cand), cand = (1:height(T))'; end
[~, j] = min(abs(T.rms_xte(cand) - median(T.rms_xte)));
show = cand(j);
r = M{show};
pr('\nshowcase mission: seed %d (time %.1f s, re-plans %d, reactive %.2f s, rms xte %.3f, min obs %.3f, min wall %.3f)\n', ...
   r.seed, r.tend, numel(r.history), T.avoid_s(show), T.rms_xte(show), T.min_obs(show), T.min_wall(show));
pr('re-plan times [s]: %s\n', mat2str(round([r.history.step]/100, 1)));
fclose(fid);
type(fullfile(outDir, 'summary.txt'));
disp(T(:, {'seed', 'time', 'final_err', 'min_obs', 'min_wall', 'min_obs_raw', 'min_wall_raw', 'approach_mid', 'cap_final', 'rms_xte', 'max_xte', 'replans', 'avoid_s'}));

% ---------------- figures ----------------
C = struct('plan', [0.90 0.55 0.00], 'astar', [0.40 0.40 0.45], 'drv', [0.00 0.40 0.75], ...
           'rep', [0.55 0.20 0.85], 'lim', [0.80 0.10 0.10], ...
           'obs', [0.85 0.15 0.15; 0.15 0.60 0.20; 0.15 0.35 0.85]);
th = linspace(0, 2*pi, 60);
WP = {'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.85 0.1 0.45], 'MarkerEdgeColor', 'k'};

% fig_plan: the plan before the mission, with the inflated bands
f = new_fig(430, 390);  ax = axes(f);  draw_map(ax, lm, true);
h1 = plot(ax, r.astar_path(:,1), r.astar_path(:,2), ':', 'Color', C.astar, 'LineWidth', 1.8);
h2 = plot(ax, r.wp_plot(:,1), r.wp_plot(:,2), '-', 'Color', C.plan, 'LineWidth', 1.6);
h3 = plot(ax, r.wp_ordered(:,1), r.wp_ordered(:,2), WP{:});
h4 = plot(ax, r.p(1,1), r.p(1,2), 's', 'MarkerSize', 10, 'MarkerFaceColor', [0.1 0.5 0.9], 'MarkerEdgeColor', 'k');
h5 = patch(ax, NaN, NaN, [1.00 0.80 0.80], 'EdgeColor', 'none');
label_wps(ax, r.wp_ordered);
legend(ax, [h4 h3 h1 h2 h5], {'Start', 'Waypoint', 'A* route', 'Smoothed path', 'Within 0.60 m of a wall'}, 'Location', 'southoutside', 'NumColumns', 3, 'FontSize', 8);
save_fig(f, figDir, 'fig_plan');

% fig_mission: the whole mission as driven
f = new_fig(430, 390);  ax = axes(f);  draw_map(ax, lm, false);
ho = draw_obstacles(ax, obs, C.obs, CLEAR + OBS_R, th, OBS_R);
% same line styles as fig_replan
h2 = plot(ax, r.wp_plot(:,1), r.wp_plot(:,2), '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.2);
h5 = plot_replans(ax, r, [0.95 0.50 0.00], 2.6, '-');
h6 = plot(ax, r.p(:,1), r.p(:,2), '-', 'Color', C.drv, 'LineWidth', 1.6);
plot(ax, r.wp_ordered(:,1), r.wp_ordered(:,2), WP{:});
label_wps(ax, r.wp_ordered);
legend(ax, [h2 h5 h6 ho], {'Planned path', 'Re-planned path', 'Driven path (true pose)', 'Obstacle, 0.40 m clearance (from surface)'}, 'Location', 'southoutside', 'NumColumns', 2, 'FontSize', 8);
save_fig(f, figDir, 'fig_mission');

% fig_replan: the clearest re-plan over all missions. Candidates are
% re-plans whose previous path ran within 0.3 m of an obstacle centre (so
% through the obstacle) and whose new path clears the 0.8 m keep-out; the
% one the vehicle then followed most closely, along the whole new path, is
% shown.
best = struct('track', inf);
for k = 1:numel(M)
    rk = M{k};  stepk = round(rk.ptt*100);  prev = rk.wp_plot;
    for q = 1:numel(rk.history)
        P = rk.history(q).P;
        k0 = find(stepk >= rk.history(q).step, 1);
        if isempty(k0), prev = P; continue, end
        rp0 = rk.p(k0, :);
        [~, i0] = min(hypot(P(:,1) - rp0(1), P(:,2) - rp0(2)));
        Q = P(i0:end, :);
        jq = find(hypot(diff(Q(:,1)), diff(Q(:,2))) > 0.5, 1);
        if ~isempty(jq), Q = Q(1:jq, :); end
        [~, ke] = min(hypot(rk.p(k0:end,1) - Q(end,1), rk.p(k0:end,2) - Q(end,2)));
        seg = rk.p(k0:k0+ke-1, :);
        for o = 1:size(obs, 1)
            dPrev = min(hypot(prev(:,1) - obs(o,1), prev(:,2) - obs(o,2)));
            dNew  = min(hypot(Q(:,1) - obs(o,1), Q(:,2) - obs(o,2)));
            if dPrev < 0.3 && dNew > 0.8 && size(seg, 1) > 10
                tr = max(path_distance(seg, Q));
                if tr < best.track
                    best = struct('track', tr, 'd', dPrev, 'k', k, 'q', q, 'o', o, 'k0', k0, 'prev', prev, 'P', P, 'Pn', Q);
                end
            end
        end
        prev = P;
    end
end
rb = M{best.k};  ob = obs(best.o, 1:2);  rp = rb.p(best.k0, :);  Pn = best.Pn;
% view: the new path, the obstacle and the re-plan point with a 1 m margin
bx = [min([Pn(:,1); ob(1); rp(1)]) - 1, max([Pn(:,1); ob(1); rp(1)]) + 1];
by = [min([Pn(:,2); ob(2); rp(2)]) - 1, max([Pn(:,2); ob(2); rp(2)]) + 1];
cx = mean(bx);  cy = mean(by);  hw = max(diff(bx), diff(by))/2;
ORANGE = [0.95 0.50 0.00];
f = new_fig(400, 420);  ax = axes(f);  draw_map(ax, lm, false);
ho = draw_obstacles(ax, obs, C.obs, CLEAR + OBS_R, th, OBS_R);
for k = 1:size(obs, 1)
    hk = plot(ax, obs(k,1) + 0.8*cos(th), obs(k,2) + 0.8*sin(th), '--', 'Color', C.rep, 'LineWidth', 1.0);
end
% path before the re-plan thin grey dashed, new path solid orange, driven
% path solid blue on top
% every path the vehicle had: raw A* route and smoothed plan before the
% re-plan, the re-planned path as a wide see-through band, and the driven
% path as a thin line on top, so overlapping paths stay visible
h1 = plot(ax, rb.astar_path(:,1), rb.astar_path(:,2), ':', 'Color', [0.35 0.35 0.40], 'LineWidth', 1.1);
h2 = plot(ax, best.prev(:,1), best.prev(:,2), '--', 'Color', [0.60 0.60 0.60], 'LineWidth', 1.4);
h5 = plot(ax, Pn(:,1), Pn(:,2), '-', 'Color', [ORANGE 0.45], 'LineWidth', 6.0);
h6 = plot(ax, rb.p(:,1), rb.p(:,2), '-', 'Color', [0.00 0.30 0.70], 'LineWidth', 1.3);
plot(ax, rp(1), rp(2), 'd', 'MarkerSize', 8, 'MarkerFaceColor', ORANGE, 'MarkerEdgeColor', 'k');
inview = abs(rb.wp_ordered(:,1) - cx) < hw - 0.3 & abs(rb.wp_ordered(:,2) - cy) < hw - 0.3;
plot(ax, rb.wp_ordered(:,1), rb.wp_ordered(:,2), WP{:});
for kw = find(inview)'                                     % label inside the view, above-left of the marker
    text(ax, rb.wp_ordered(kw,1) - 0.30, rb.wp_ordered(kw,2) + 0.35, sprintf('%d', kw), ...
         'FontWeight', 'bold', 'FontSize', 9, 'HorizontalAlignment', 'right', 'Clipping', 'on');
end
xlim(ax, cx + [-hw hw]);  ylim(ax, cy + [-hw hw]);
legend(ax, [h1 h2 h5 h6 ho hk], {'Raw A* route', 'Plan before re-plan', 'Re-planned path', ...
       'Driven path', '0.40 m clearance', '0.8 m keep-out'}, 'Location', 'southoutside', 'NumColumns', 2, 'FontSize', 8);
save_fig(f, figDir, 'fig_replan');
fidx = fopen(fullfile(outDir, 'summary.txt'), 'a');
fprintf(fidx, ['fig_replan: mission seed %d, re-plan %d, obstacle %d at (%.1f, %.1f); path before passed %.2f m ' ...
               'from its centre, new path %.2f m, vehicle %.2f m away at the re-plan, worst tracking %.2f m\n'], ...
        rb.seed, best.q, best.o, ob(1), ob(2), best.d, ...
        min(hypot(best.P(:,1) - ob(1), best.P(:,2) - ob(2))), hypot(ob(1) - rp(1), ob(2) - rp(2)), best.track);
fclose(fidx);

% fig_timeline: clearance and cross-track error against time
f = new_fig(430, 380);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
plot(ax, r.ptt, r.dObs, '-', 'Color', C.drv, 'LineWidth', 1.1);
plot(ax, r.ptt, r.dWall, '-', 'Color', C.plan, 'LineWidth', 1.1);
yline(ax, CLEAR, '--', 'Color', C.lim, 'LineWidth', 1.6);
ylabel(ax, 'Clearance (m)');  ylim(ax, [0 2]);  xlim(ax, [0 r.ptt(end)]);
legend(ax, {'Obstacle', 'Wall', 'Required 0.40 m'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
set(ax, 'FontSize', 9);
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
plot(ax, r.ptt, r.xte, '-', 'Color', C.drv, 'LineWidth', 0.9);
for q = 1:numel(r.history)
    xline(ax, r.history(q).step/100, ':', 'Color', C.rep, 'LineWidth', 1.8);
end
ylabel(ax, 'Cross-track error (m)');  xlabel(ax, 'Time (s)');  xlim(ax, [0 r.ptt(end)]);
set(ax, 'FontSize', 9);
save_fig(f, figDir, 'fig_timeline');

% fig_batch: every mission
f = new_fig(800, 300);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
b = bar(ax, T.seed, [T.min_obs, T.min_wall], 'grouped');
b(1).FaceColor = C.drv;  b(2).FaceColor = C.plan;
yline(ax, CLEAR, '--', 'Color', C.lim, 'LineWidth', 1.8);
ylabel(ax, 'Minimum clearance (m)');  xlabel(ax, 'Mission');
legend(ax, {'Obstacle', 'Wall', 'Required 0.40 m'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 9);
set(ax, 'FontSize', 9, 'XTick', 1:2:N);
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
yyaxis(ax, 'left');   bar(ax, T.seed, T.time, 'FaceColor', [0.60 0.60 0.65]);  ylabel(ax, 'Mission time (s)');
yyaxis(ax, 'right');  plot(ax, T.seed, T.replans, 'o', 'Color', C.rep, 'MarkerFaceColor', C.rep, 'MarkerSize', 5);  ylabel(ax, 'Re-plans');  ylim(ax, [0 7]);  yticks(ax, 0:7);
ax.YAxis(1).Color = [0.3 0.3 0.3];  ax.YAxis(2).Color = C.rep;
xlabel(ax, 'Mission');  set(ax, 'FontSize', 9, 'XTick', 1:2:N);
save_fig(f, figDir, 'fig_batch');
end

% =====================================================================
function [T, M] = run_set(files, obs, lm, OBS_R)
N = numel(files);
vars = {'seed', 'time', 'final_err', 'complete', 'astar_len', 'plan_len', 'driven_len', ...
        'min_obs', 'min_wall', 'min_obs_raw', 'min_wall_raw', 'est_vs_raw', 'raw_sigma', ...
        'cap_mid', 'cap_final', 'approach_mid', 'rms_xte', 'max_xte', 'replans', 'avoid_s', ...
        'aco_opt', 'pso_fallback', 'pso_legs', 'nn_gap', 'aco_gap', ...
        'min_obs_est', 'min_wall_est', 'est_rms', 'est_max', 'head_rms', 'raw_rms', ...
        'wall_s', 'cpu_max', 'cpu_mean', 'kappa_max', 'kappa_over'};
T = array2table(nan(N, numel(vars)), 'VariableNames', vars);
M = cell(N, 1);
for k = 1:N
    r = load(fullfile(files(k).folder, files(k).name));
    raw = r.pt(:, 1:2);                               % noisy simulator pose, 20 Hz
    pe = interp1(r.t, r.xh(:, 1:3), r.ptt, 'linear', 'extrap');  % estimate at the same instants
    tr = interp1(r.pct, r.pc(:, 1:3), r.ptt, 'linear', 'extrap'); % noise-free pose
    p = tr(:, 1:2);                                   % truth drives every metric below
    r.p = p;  r.pe = pe;  n = size(p, 1);
    ee = hypot(pe(:,1) - p(:,1), pe(:,2) - p(:,2));
    eh = rad2deg(atan2(sin(pe(:,3) - tr(:,3)), cos(pe(:,3) - tr(:,3))));
    raw_rms = sqrt(mean(sum((raw - p).^2, 2)));
    dObsH  = min(hypot(pe(:,1) - obs(:,1)', pe(:,2) - obs(:,2)'), [], 2) - OBS_R;
    dWallH = wall_distance(pe(:,1:2), lm);
    % wall repulsion: a lidar beam inside the reach min(1.2, max(0.5, d_wp))
    dw = interp1(r.dwpt, r.dwp(:), r.rgt, 'previous', 'extrap');
    reach = 1.2*ones(size(dw));  g = dw > 0;  reach(g) = min(1.2, max(0.5, dw(g)));
    rg = r.ranges;  rg(isnan(rg) | rg <= 0) = inf;
    wall_s = sum(any(rg < reach, 2)) * median(diff(r.rgt));
    cpu = r.replan_cpu(:);  if isempty(cpu), cpu = NaN; end
    kap = [r.pso_info.kappa_max];
    s1 = p([1:20:n-1, n], :);                         % 1 Hz
    driven = sum(hypot(diff(s1(:,1)), diff(s1(:,2))));
    r.dObs  = min(hypot(p(:,1) - obs(:,1)', p(:,2) - obs(:,2)'), [], 2) - OBS_R;
    r.dWall = wall_distance(p, lm);
    dObsE  = min(hypot(raw(:,1) - obs(:,1)', raw(:,2) - obs(:,2)'), [], 2) - OBS_R;
    dWallE = wall_distance(raw, lm);
    est_rms = sqrt(mean(sum((pe(:,1:2) - raw).^2, 2)));   % estimate against raw pose
    % White noise of standard deviation s per axis gives a second difference
    % of RMS sqrt(12)*s over the two axes; the motion itself contributes little.
    jit = sqrt(mean(sum(diff(raw, 2).^2, 2)))/sqrt(12);
    % cross-track error against the path that was active at each instant
    step = round(r.ptt*100);
    xte = path_distance(p, r.wp_plot);
    for q = 1:numel(r.history)
        if q < numel(r.history), last = r.history(q+1).step; else, last = inf; end
        idx = step >= r.history(q).step & step < last;
        if any(idx), xte(idx) = path_distance(p(idx, :), r.history(q).P); end
    end
    r.xte = xte;
    wp = r.wp_ordered;  nw = size(wp, 1);
    fe = hypot(p(end,1) - wp(end,1), p(end,2) - wp(end,2));
    ce = r.cap_err(:);
    cap_mid = NaN;  cap_final = NaN;
    if numel(ce) >= nw, cap_mid = mean(ce(1:nw-1));  cap_final = ce(nw);
    elseif ~isempty(ce), cap_mid = mean(ce(1:min(end, nw-1))); end
    appr = min(hypot(p(:,1) - wp(:,1)', p(:,2) - wp(:,2)'), [], 1);   % closest true approach
    av = sum(isfinite(r.d_obs(:)))*0.01;
    % brute-force optimum of the ordering
    P = perms(2:nw+1);  best = inf;
    for q = 1:size(P, 1)
        tour = [1, P(q, :)];
        best = min(best, sum(r.D(sub2ind(size(r.D), tour(1:end-1), tour(2:end)))));
    end
    % nearest-neighbour tour on the same distance matrix
    left = 2:nw+1;  cur = 1;  nn = 0;
    while ~isempty(left)
        [dmin, j] = min(r.D(cur, left));  nn = nn + dmin;  cur = left(j);  left(j) = [];
    end
    fb = [r.pso_info.fallback];
    T{k, :} = [r.seed, r.tend, fe, double(numel(ce) >= nw && r.tend < 599), r.astar_dist, r.path_dist, driven, ...
               min(r.dObs), min(r.dWall), min(dObsE), min(dWallE), est_rms, jit, cap_mid, cap_final, mean(appr(1:nw-1)), ...
               rms(xte), max(xte), numel(r.history), av, double(r.dist_cum <= best + 1e-6), sum(fb), numel(fb), 100*(nn/best - 1), 100*(r.dist_cum/best - 1), ...
               min(dObsH), min(dWallH), sqrt(mean(ee.^2)), max(ee), sqrt(mean(eh.^2)), raw_rms, ...
               wall_s, max(cpu), mean(cpu), max(kap), sum(kap > 1.0 + 1e-3)];
    M{k} = r;
end
end

function h = plot_replans(ax, r, col, lw, ls)
% The parts of each published path that differ from the path before it,
% and a diamond where the vehicle was when that re-plan happened.
if nargin < 5, ls = ':'; end
h = plot(ax, NaN, NaN, ls, 'Color', col, 'LineWidth', lw);
prev = r.wp_plot;
for q = 1:numel(r.history)
    P = r.history(q).P;
    ch = path_distance(P, prev) > 0.08;
    Q = P;  Q(~ch, :) = NaN;
    jump = [false; hypot(diff(P(:,1)), diff(P(:,2))) > 0.5];
    Q(jump, :) = NaN;
    plot(ax, Q(:,1), Q(:,2), ls, 'Color', col, 'LineWidth', lw);
    k = min(max(round(r.history(q).step/5), 1), size(r.p, 1));       % 100 Hz step to 20 Hz sample
    plot(ax, r.p(k,1), r.p(k,2), 'd', 'MarkerSize', 7, 'MarkerFaceColor', col, 'MarkerEdgeColor', 'k');
    prev = P;
end
end

function f = new_fig(w, h)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 w h]);
end

function save_fig(f, outDir, name)
exportgraphics(f, fullfile(outDir, [name '.png']), 'Resolution', 260);
close(f);
end

function draw_map(ax, lm, bands)
[nr, nc] = size(lm);
g = 1 - 0.75*double(lm);
rgb = repmat(g, 1, 1, 3);
if bands
    d6 = dilate(lm, 6) & ~lm;
    tint = cat(3, 1.00*ones(nr, nc), 0.80*ones(nr, nc), 0.80*ones(nr, nc));
    m3 = repmat(d6, 1, 1, 3);
    rgb(m3) = tint(m3);
end
image(ax, 'XData', [0.05 nc/10 - 0.05], 'YData', [nr/10 - 0.05 0.05], 'CData', uint8(255*rgb));
hold(ax, 'on');  axis(ax, 'equal');  box(ax, 'on');
xlim(ax, [0 nc/10]);  ylim(ax, [0 nr/10]);
set(ax, 'YDir', 'normal', 'FontSize', 9, 'Layer', 'top');
xlabel(ax, 'x (m)');  ylabel(ax, 'y (m)');
end

function out = dilate(lm, r)
[n, m] = size(lm);  out = lm;
for dr = -r:r
    for dc = -r:r
        if dr^2 + dc^2 > r^2, continue, end
        sh = false(n, m);
        rs = max(1, 1+dr):min(n, n+dr);  cs = max(1, 1+dc):min(m, m+dc);
        sh(rs, cs) = lm(rs-dr, cs-dc);
        out = out | sh;
    end
end
end

function h = draw_obstacles(ax, obs, cols, ring, th, rad)
% Each obstacle as a disc of its real radius, with the clearance ring
% (vehicle centre must stay outside it).
for k = 1:size(obs, 1)
    c = cols(min(max(round(obs(k,3)), 1), 3), :);
    patch(ax, obs(k,1) + rad*cos(th), obs(k,2) + rad*sin(th), c, 'EdgeColor', 'k', 'LineWidth', 0.5);
    h = plot(ax, obs(k,1) + ring*cos(th), obs(k,2) + ring*sin(th), '-', 'Color', [0.80 0.10 0.10], 'LineWidth', 1.1);
end
end

function label_wps(ax, wp)
for k = 1:size(wp, 1)
    text(ax, wp(k,1) + 0.6, wp(k,2) + 0.8, sprintf('%d', k), 'FontWeight', 'bold', 'FontSize', 9, 'Clipping', 'on');
end
end

function d = path_distance(xy, pth)
% Distance from each point to the nearest point on the polyline PTH.
% Joins longer than 0.5 m are not part of the path (see plot_replans).
a  = pth(1:end-1,:);  b = pth(2:end,:);
keep = hypot(b(:,1) - a(:,1), b(:,2) - a(:,2)) <= 0.5;
a = a(keep, :);  b = b(keep, :);
ab = b - a;  L2 = max(sum(ab.^2, 2), eps);
d  = zeros(size(xy, 1), 1);
for i = 1:size(xy, 1)
    s = min(1, max(0, ((xy(i,1) - a(:,1)).*ab(:,1) + (xy(i,2) - a(:,2)).*ab(:,2))./L2));
    d(i) = sqrt(min((a(:,1) + s.*ab(:,1) - xy(i,1)).^2 + (a(:,2) + s.*ab(:,2) - xy(i,2)).^2));
end
end

function d = wall_distance(xy, lm)
% Distance from each point to the nearest wall face (as in report_plots.m).
[nr, ~] = size(lm);
pad  = true(size(lm) + 2);  pad(2:end-1, 2:end-1) = lm;
edge = lm & ~(pad(1:end-2, 2:end-1) & pad(3:end, 2:end-1) & pad(2:end-1, 1:end-2) & pad(2:end-1, 3:end));
[r, c] = find(edge);
cx = (c - 0.5)/10;  cy = (nr - r + 0.5)/10;
d  = zeros(size(xy, 1), 1);
for i = 1:size(xy, 1)
    d(i) = min(hypot(max(abs(cx - xy(i,1)) - 0.05, 0), max(abs(cy - xy(i,2)) - 0.05, 0)));
end
end
