function report_plots(nBatch)
% REPORT_PLOTS  Report figures from the last simulation run.
%
%   report_plots          figures for the mission that was just simulated
%   report_plots(N)       the same, then N more random missions are simulated
%                         and their statistics plotted (takes about a minute
%                         per mission)
%
%   Run a mission first (startupGround, then Run in Simulink), then call this
%   from the sim_env folder. It reads the logged signals and the planner's
%   results from the base workspace, so nothing has to be passed in.
%
%   Every figure is saved to sim_env/report_figures as a vector PDF and a
%   300 dpi PNG:
%     01 trajectory            map, planned path, driven path, waypoints
%     02 obstacle_pass         zoom on the closest obstacle pass
%     03 clearance_map         inflated walls the planner avoids, with the path
%     04 aco_convergence       ant colony tour length against iteration
%     05 waypoint_order        generated order against optimised order
%     06 estimate_vs_measured  x, y, heading: measurement and Kalman estimate
%     07 estimate_zoom         a 10 s window of the same
%     08 innovation            innovations with their +/-2 sigma bounds
%     09 innovation_hist       normalised innovations against a unit normal
%     10 covariance            diagonal of the error covariance P
%     11 kalman_gain           main Kalman gains
%     12 cross_track           distance from the planned path, with its RMS
%     13 capture_error         distance to each waypoint at capture
%     14 obstacle_clearance    distance to the nearest obstacle
%     15 wall_clearance        distance to the nearest wall face
%     16 speed                 measured speed
%     17 batch_*               statistics over N missions (only with N > 0)
%
%   Clearances and errors are measured from the Kalman estimate x_hat. The
%   logged pose is the noisy measurement, and the minimum of a noisy signal
%   reads lower than the true clearance. This quick-look script does not log
%   the noise-free pose; the report's figures and statistics are measured
%   from it (Four-Wheel Steering Simulation output) by
%   report/analysis/batch_missions.m and analyse_missions.m.

if nargin < 1, nBatch = 0; end

% ---------- Settings ----------
CLEAR     = 0.40;                 % [m] required clearance: 0.25 m radius + 0.15 m
FINAL_CAP = 0.10;                 % [m] capture radius of the last waypoint (Guidance block)
% Measurement noise variances, as in the State Estimation block: x, y, theta, v
R         = diag([0.1, 0.1, 2*(pi/180)^2, 0.01]);
MODEL     = 'sl_groundvehicleDynamics';
OUT_DIR   = fullfile(fileparts(mfilename('fullpath')), 'report_figures');
COL       = struct('est', [0.00 0.45 0.74], 'meas', [0.70 0.70 0.70], ...
                   'plan', [0.85 0.33 0.10], 'lim', [0.80 0.10 0.10], ...
                   'obs', [0.85 0.10 0.10; 0.10 0.60 0.20; 0.10 0.30 0.85]);

if ~exist(OUT_DIR, 'dir'), mkdir(OUT_DIR); end

% ---------- Data from the last run ----------
need = {'xhat_log', 'z_log', 'innov_log', 'K_log', 'P_log', 'd_obs_log', ...
        'logical_map', 'occ', 'obstacles', 'wp_list', 'wp_ordered', 'wp_plot', ...
        'D', 'dist_cum', 'history', 'path_dist', 'capture_radius', 'v_max', 'robot'};
for i = 1:numel(need)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', need{i}))
        error('report_plots:noData', ...
              'Variable "%s" is not in the workspace. Run a mission first.', need{i});
    end
end
ws = @(name) evalin('base', name);

xhat_log = ws('xhat_log');     z_log = ws('z_log');
t     = xhat_log.Time(:);
xh    = squeeze(xhat_log.Data)';                 % n-by-3 estimate [x y theta]
z     = z_log.Data;                              % n-by-4 measurement [x y theta v]
innov = squeeze(ws('innov_log').Data)';         % n-by-4
K     = ws('K_log').Data;                       % 7-by-4-by-n
P     = ws('P_log').Data;                       % 7-by-7-by-n
d_obs = ws('d_obs_log');
lm    = ws('logical_map');     occ = ws('occ');
obs   = ws('obstacles');
wpl   = ws('wp_list');         wps = ws('wp_ordered');
pth   = ws('wp_plot');         D   = ws('D');
hist_ = ws('history');
cap   = ws('capture_radius');  v_max = ws('v_max');
rob   = ws('robot');           start = [rob.X, rob.Y];
n     = numel(t);
% Driven length from the estimate sampled once a second: summing every
% 10 ms step would add up the estimate's jitter as if it were distance.
xs1    = xh([1:100:n-1, n], 1:2);
driven = sum(hypot(diff(xs1(:,1)), diff(xs1(:,2))));

% ---------- 01 Trajectory ----------
f = new_fig(900, 720);  ax = axes(f);
draw_map(ax, lm);  draw_obstacles(ax, obs, COL.obs);
plot(ax, pth(:,1), pth(:,2), '--', 'Color', COL.plan, 'LineWidth', 1.2, 'DisplayName', 'Planned path');
plot(ax, xh(:,1), xh(:,2), '-', 'Color', COL.est, 'LineWidth', 1.4, 'DisplayName', 'Driven path (estimate)');
plot(ax, start(1), start(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Start');
plot(ax, wps(:,1), wps(:,2), 'kp', 'MarkerFaceColor', [1 0.85 0.1], 'MarkerSize', 13, 'DisplayName', 'Waypoints');
for k = 1:size(wps, 1)
    text(ax, wps(k,1) + 0.5, wps(k,2) + 0.6, sprintf('%d', k), 'FontWeight', 'bold');
end
legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal');
title(ax, sprintf('Mission: %.1f s, planned %.1f m, driven %.1f m', ...
      t(end), ws('path_dist'), driven));
save_fig(f, OUT_DIR, '01_trajectory');

% ---------- 02 Closest obstacle pass ----------
dObsAll = hypot(xh(:,1) - obs(:,1)', xh(:,2) - obs(:,2)');    % n-by-nObs
[dNear, iNear] = min(dObsAll, [], 2);
[dMinObs, iMin] = min(dNear);
o = obs(iNear(iMin), :);
f = new_fig(620, 600);  ax = axes(f);
draw_map(ax, lm);  draw_obstacles(ax, obs, COL.obs);
th = linspace(0, 2*pi, 80);
plot(ax, o(1) + CLEAR*cos(th), o(2) + CLEAR*sin(th), ':', 'Color', COL.lim, ...
     'LineWidth', 1.5, 'DisplayName', sprintf('%.2f m clearance', CLEAR));
plot(ax, pth(:,1), pth(:,2), '--', 'Color', COL.plan, 'LineWidth', 1.2, 'DisplayName', 'Planned path');
plot(ax, xh(:,1), xh(:,2), '-', 'Color', COL.est, 'LineWidth', 1.6, 'DisplayName', 'Driven path (estimate)');
plot(ax, [o(1), xh(iMin,1)], [o(2), xh(iMin,2)], 'k-', 'LineWidth', 1, 'HandleVisibility', 'off');
xlim(ax, o(1) + [-3 3]);  ylim(ax, o(2) + [-3 3]);
legend(ax, 'Location', 'southoutside', 'NumColumns', 2);
title(ax, sprintf('Closest obstacle pass: %.2f m at t = %.1f s', dMinObs, t(iMin)));
save_fig(f, OUT_DIR, '02_obstacle_pass');

% ---------- 03 Clearance map used by the planner ----------
f = new_fig(900, 720);  ax = axes(f);
draw_map(ax, lm + (occ & ~lm)*0.35);            % walls dark, inflated band light
plot(ax, pth(:,1), pth(:,2), '-', 'Color', COL.plan, 'LineWidth', 1.4, 'DisplayName', 'A* path');
plot(ax, wps(:,1), wps(:,2), 'kp', 'MarkerFaceColor', [1 0.85 0.1], 'MarkerSize', 12, 'DisplayName', 'Waypoints');
legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal');
title(ax, 'Walls (dark) and the inflated band the planner keeps out of (light)');
save_fig(f, OUT_DIR, '03_clearance_map');

% ---------- 04 Ant colony convergence ----------
f = new_fig(700, 420);  ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, hist_(:,2), '-', 'Color', COL.meas, 'DisplayName', 'Mean of the iteration');
plot(ax, hist_(:,1), '-', 'Color', COL.est, 'LineWidth', 1.6, 'DisplayName', 'Best so far');
xlabel(ax, 'Iteration');  ylabel(ax, 'Tour length (m)');  legend(ax, 'Location', 'northeast');
title(ax, sprintf('Ant colony optimisation: best tour %.1f m', hist_(end,1)));
save_fig(f, OUT_DIR, '04_aco_convergence');

% ---------- 05 Waypoint order: as generated against optimised ----------
nW  = size(wpl, 1);
[~, ord] = ismember(wps, wpl, 'rows');           % optimised order as indices into wp_list
tourLen  = @(idx) sum(D(sub2ind(size(D), [1, idx(1:end-1)' + 1], idx' + 1)));
dGen = tourLen((1:nW)');  dOpt = tourLen(ord);
f = new_fig(1100, 480);
tours = {[start; wpl], [start; wps]};
names = {sprintf('As generated: %.1f m', dGen), sprintf('Optimised: %.1f m', dOpt)};
for k = 1:2
    ax = subplot(1, 2, k, 'Parent', f);
    draw_map(ax, lm);
    plot(ax, tours{k}(:,1), tours{k}(:,2), '-o', 'Color', COL.est, 'LineWidth', 1.4, ...
         'MarkerFaceColor', COL.est);
    plot(ax, start(1), start(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    for j = 1:nW
        text(ax, tours{k}(j+1,1) + 0.6, tours{k}(j+1,2) + 0.8, sprintf('%d', j), 'FontWeight', 'bold');
    end
    title(ax, names{k});
end
sgtitle(f, 'Visiting order (lines show order only; lengths are A* driving distances)');
save_fig(f, OUT_DIR, '05_waypoint_order');

% ---------- 06 / 07 Estimate against measurement ----------
lab = {'x (m)', 'y (m)', 'Heading (deg)'};
sc  = [1, 1, 180/pi];
win = {[t(1), t(end)], min(t(end), t(end)/2 + [0, 10])};
fn  = {'06_estimate_vs_measured', '07_estimate_zoom'};
new = ~isnan(innov(:,1));                        % steps where a new pose sample arrived
for w = 1:2
    f = new_fig(900, 700);
    for k = 1:3
        ax = subplot(3, 1, k, 'Parent', f);  hold(ax, 'on');  grid(ax, 'on');
        plot(ax, t(new), z(new,k)*sc(k), '.', 'Color', COL.meas, 'MarkerSize', 5, 'DisplayName', 'Measured');
        plot(ax, t, xh(:,k)*sc(k), '-', 'Color', COL.est, 'LineWidth', 1.3, 'DisplayName', 'Kalman estimate');
        ylabel(ax, lab{k});  xlim(ax, win{w});
        if k == 1, legend(ax, 'Location', 'best'); end
    end
    xlabel(ax, 'Time (s)');
    save_fig(f, OUT_DIR, fn{w});
end

% ---------- 08 / 09 Innovation and its +/-2 sigma bounds ----------
% The innovation covariance S is recovered from the logged gain: on a pose
% step H*K = I - R/S, so S = (I - H*K)\R; on a speed-only step the same
% holds for the scalar speed channel.
sig = nan(n, 4);
for i = 1:n
    if ~isnan(innov(i,1))
        S = (eye(4) - K(1:4,:,i)) \ R;
        sig(i,:) = sqrt(abs(diag(S)))';
    else
        sig(i,4) = sqrt(R(4,4)/(1 - K(4,4,i)));
    end
end
lab = {'x (m)', 'y (m)', 'Heading (deg)', 'Speed (m/s)'};
sc  = [1, 1, 180/pi, 1];
f = new_fig(900, 820);
for k = 1:4
    ax = subplot(4, 1, k, 'Parent', f);  hold(ax, 'on');  grid(ax, 'on');
    ok = ~isnan(innov(:,k));
    if k < 4
        plot(ax, t(ok), innov(ok,k)*sc(k), '.', 'Color', COL.est, 'MarkerSize', 4);
    else                                         % every step has a speed sample: draw a line
        plot(ax, t(ok), innov(ok,k)*sc(k), '-', 'Color', COL.est, 'LineWidth', 0.4);
    end
    xlim(ax, [t(1), t(end)]);
    plot(ax, t(ok), 2*sig(ok,k)*sc(k), '-', 'Color', COL.lim, 'LineWidth', 1);
    plot(ax, t(ok), -2*sig(ok,k)*sc(k), '-', 'Color', COL.lim, 'LineWidth', 1);
    inside = mean(abs(innov(ok,k)) <= 2*sig(ok,k))*100;
    ylabel(ax, lab{k});
    title(ax, sprintf('%.1f%% inside \\pm2\\sigma (95%% expected)', inside), 'FontWeight', 'normal');
end
xlabel(ax, 'Time (s)');
sgtitle(f, 'Kalman filter innovations');
save_fig(f, OUT_DIR, '08_innovation');

f = new_fig(900, 600);
xs = linspace(-4, 4, 200);
for k = 1:4
    ax = subplot(2, 2, k, 'Parent', f);  hold(ax, 'on');  grid(ax, 'on');
    ok = ~isnan(innov(:,k));
    nu = innov(ok,k)./sig(ok,k);
    histogram(ax, nu, 'BinLimits', [-4 4], 'NumBins', 40, 'Normalization', 'pdf', ...
              'FaceColor', COL.est, 'EdgeColor', 'none');
    plot(ax, xs, exp(-xs.^2/2)/sqrt(2*pi), '-', 'Color', COL.lim, 'LineWidth', 1.4);
    xlabel(ax, ['Normalised innovation: ', strtok(lab{k}, ' ')]);
    title(ax, sprintf('mean %.2f, std %.2f', mean(nu), std(nu)), 'FontWeight', 'normal');
end
sgtitle(f, 'Normalised innovations against the unit normal (ideal: mean 0, std 1)');
save_fig(f, OUT_DIR, '09_innovation_hist');

% ---------- 10 Covariance ----------
f = new_fig(800, 460);  ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
pd = zeros(n, 4);
for k = 1:4, pd(:,k) = squeeze(P(k,k,:)); end
plot(ax, t, sqrt(pd(:,1)), 'LineWidth', 1.2, 'DisplayName', '\sigma_x (m)');
plot(ax, t, sqrt(pd(:,2)), '--', 'LineWidth', 1.2, 'DisplayName', '\sigma_y (m)');
plot(ax, t, sqrt(pd(:,3))*180/pi, 'LineWidth', 1.2, 'DisplayName', '\sigma_\theta (deg)');
plot(ax, t, sqrt(pd(:,4)), 'LineWidth', 1.2, 'DisplayName', '\sigma_v (m/s)');
set(ax, 'YScale', 'log');  xlabel(ax, 'Time (s)');  ylabel(ax, 'Estimate standard deviation');
xlim(ax, [0, min(t(end), 20)]);  legend(ax, 'Location', 'northeast');
title(ax, 'Error covariance settling (first 20 s; the sawtooth is the 10 Hz pose update)');
save_fig(f, OUT_DIR, '10_covariance');

% ---------- 11 Kalman gain ----------
f = new_fig(800, 460);  ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
ok = ~isnan(innov(:,1));                          % steps with a pose update
plot(ax, t(ok), squeeze(K(1,1,ok)), 'LineWidth', 1.2, 'DisplayName', 'x from x');
plot(ax, t(ok), squeeze(K(2,2,ok)), '--', 'LineWidth', 1.2, 'DisplayName', 'y from y');
plot(ax, t(ok), squeeze(K(3,3,ok)), 'LineWidth', 1.2, 'DisplayName', '\theta from \theta');
plot(ax, t, squeeze(K(4,4,:)), 'LineWidth', 1.2, 'DisplayName', 'v from v');
xlabel(ax, 'Time (s)');  ylabel(ax, 'Kalman gain');  xlim(ax, [0, min(t(end), 20)]);
legend(ax, 'Location', 'northeast');  title(ax, 'Kalman gains settling (first 20 s)');
save_fig(f, OUT_DIR, '11_kalman_gain');

% ---------- 12 Cross-track error ----------
xte = path_distance(xh(:,1:2), pth);
f = new_fig(900, 420);  ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, t, xte, '-', 'Color', COL.est, 'LineWidth', 1.1);
yline(ax, rms(xte), '--', sprintf('RMS %.3f m', rms(xte)), 'Color', COL.lim, 'LineWidth', 1.2);
xlabel(ax, 'Time (s)');  ylabel(ax, 'Distance from planned path (m)');
title(ax, 'Cross-track error (peaks are obstacle avoidance leaving the path on purpose)');
save_fig(f, OUT_DIR, '12_cross_track');

% ---------- 13 Waypoint capture error ----------
capErr = nan(nW, 1);  capT = nan(nW, 1);  i0 = 1;
for k = 1:nW
    c = cap;
    if k == nW, c = min(cap, FINAL_CAP); end
    dk = hypot(xh(i0:end,1) - wps(k,1), xh(i0:end,2) - wps(k,2));
    j  = find(dk < c, 1);
    if isempty(j), break, end
    capErr(k) = dk(j);  capT(k) = t(i0 + j - 1);  i0 = i0 + j - 1;
end
closest = min(hypot(xh(:,1) - wps(:,1)', xh(:,2) - wps(:,2)'), [], 1)';
f = new_fig(700, 420);  ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
b = bar(ax, 1:nW, [capErr, closest], 0.8);
b(1).FaceColor = COL.est;   b(1).DisplayName = 'At capture';
b(2).FaceColor = COL.plan;  b(2).DisplayName = 'Closest approach';
legend(ax, b, 'Location', 'northeast');
plot(ax, [0.5, nW - 0.5], [cap cap], '--', 'Color', COL.lim, 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot(ax, [nW - 0.5, nW + 0.5], min(cap, FINAL_CAP)*[1 1], '--', 'Color', COL.lim, 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(ax, 0.6, cap, ' capture radius', 'VerticalAlignment', 'bottom', 'Color', COL.lim);
xlabel(ax, 'Waypoint (visiting order)');  ylabel(ax, 'Distance at capture (m)');
xticks(ax, 1:nW);  ylim(ax, [0, cap*1.35]);
title(ax, sprintf('Waypoint capture: mean error at capture %.2f m, mean closest approach %.2f m', ...
      mean(capErr, 'omitnan'), mean(closest)));
save_fig(f, OUT_DIR, '13_capture_error');

% ---------- 14 Obstacle clearance ----------
d_obs(~isfinite(d_obs)) = NaN;
f = new_fig(900, 420);  ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, t, dNear, '-', 'Color', COL.est, 'LineWidth', 1.2, 'DisplayName', 'Nearest obstacle (from estimate)');
if numel(d_obs) == n
    plot(ax, t, d_obs, '.', 'Color', COL.plan, 'MarkerSize', 5, 'DisplayName', 'Avoidance block''s d\_obs');
end
yline(ax, CLEAR, '--', sprintf('%.2f m required', CLEAR), 'Color', COL.lim, 'LineWidth', 1.2, ...
      'HandleVisibility', 'off');
ylim(ax, [0, 5]);  xlabel(ax, 'Time (s)');  ylabel(ax, 'Distance to obstacle centre (m)');
legend(ax, 'Location', 'northeast');
title(ax, sprintf('Obstacle clearance: minimum %.2f m', dMinObs));
save_fig(f, OUT_DIR, '14_obstacle_clearance');

% ---------- 15 Wall clearance ----------
dWall = wall_distance(xh(:,1:2), lm);
f = new_fig(900, 420);  ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, t, dWall, '-', 'Color', COL.est, 'LineWidth', 1.2);
yline(ax, CLEAR, '--', sprintf('%.2f m required', CLEAR), 'Color', COL.lim, 'LineWidth', 1.2);
ylim(ax, [0, 3]);  xlabel(ax, 'Time (s)');  ylabel(ax, 'Distance to nearest wall face (m)');
title(ax, sprintf('Wall clearance: minimum %.2f m', min(dWall)));
save_fig(f, OUT_DIR, '15_wall_clearance');

% ---------- 16 Speed ----------
f = new_fig(900, 400);  ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, t, z(:,4), '-', 'Color', COL.meas, 'DisplayName', 'Measured speed');
plot(ax, t, movmean(z(:,4), 101), '-', 'Color', COL.est, 'LineWidth', 1.4, 'DisplayName', '1 s moving average');
yline(ax, v_max, '--', 'v_{max}', 'Color', COL.lim, 'LineWidth', 1.2, 'HandleVisibility', 'off');
for k = 1:nW
    if ~isnan(capT(k)), xline(ax, capT(k), ':', sprintf('WP %d', k), 'HandleVisibility', 'off'); end
end
xlabel(ax, 'Time (s)');  ylabel(ax, 'Speed (m/s)');  legend(ax, 'Location', 'southeast');
title(ax, 'Speed over the mission');
save_fig(f, OUT_DIR, '16_speed');

fprintf('\nThis mission:\n');
fprintf('  mission time            %.1f s\n', t(end));
fprintf('  planned / driven length %.1f / %.1f m\n', ws('path_dist'), driven);
fprintf('  RMS cross-track error   %.3f m\n', rms(xte));
fprintf('  mean capture error      %.3f m\n', mean(capErr, 'omitnan'));
fprintf('  min obstacle clearance  %.3f m\n', dMinObs);
fprintf('  min wall clearance      %.3f m\n', min(dWall));

% ---------- 17 Statistics over many missions ----------
if nBatch > 0
    res = nan(nBatch, 5);          % time, min obstacle, min wall, final error, RMS cross-track
    load_system(MODEL);
    for m = 1:nBatch
        fprintf('Batch mission %d of %d ...\n', m, nBatch);
        out = sim(MODEL, 'ReturnWorkspaceOutputs', 'on');
        bx  = squeeze(out.xhat_log.Data)';
        bw  = ws('wp_ordered');
        res(m,:) = [out.xhat_log.Time(end), ...
                    min(min(hypot(bx(:,1) - obs(:,1)', bx(:,2) - obs(:,2)'), [], 2)), ...
                    min(wall_distance(bx(1:2:end,1:2), lm)), ...
                    hypot(bx(end,1) - bw(end,1), bx(end,2) - bw(end,2)), ...
                    rms(path_distance(bx(1:5:end,1:2), ws('wp_plot')))];
    end
    save(fullfile(OUT_DIR, 'batch_results.mat'), 'res');

    f = new_fig(1000, 420);
    lab  = {'Minimum obstacle clearance (m)', 'Minimum wall clearance (m)'};
    for k = 1:2
        ax = subplot(1, 2, k, 'Parent', f);  hold(ax, 'on');  grid(ax, 'on');
        histogram(ax, res(:,k+1), 12, 'FaceColor', COL.est);
        xline(ax, CLEAR, '--', sprintf('%.2f m required', CLEAR), 'Color', COL.lim, 'LineWidth', 1.2);
        xlabel(ax, lab{k});  ylabel(ax, 'Missions');
        title(ax, sprintf('min %.2f m, %d of %d below %.2f m', min(res(:,k+1)), ...
              sum(res(:,k+1) < CLEAR), nBatch, CLEAR), 'FontWeight', 'normal');
    end
    sgtitle(f, sprintf('Clearance over %d random missions', nBatch));
    save_fig(f, OUT_DIR, '17_batch_clearance');

    f = new_fig(1100, 380);
    lab = {'Mission time (s)', 'Final stop error (m)', 'RMS cross-track error (m)'};
    col = [1 4 5];
    for k = 1:3
        ax = subplot(1, 3, k, 'Parent', f);  hold(ax, 'on');  grid(ax, 'on');
        histogram(ax, res(:,col(k)), 12, 'FaceColor', COL.est);
        xlabel(ax, lab{k});  ylabel(ax, 'Missions');
        title(ax, sprintf('mean %.2f, max %.2f', mean(res(:,col(k))), max(res(:,col(k)))), ...
              'FontWeight', 'normal');
    end
    sgtitle(f, sprintf('Performance over %d random missions', nBatch));
    save_fig(f, OUT_DIR, '17_batch_performance');
end

fprintf('\nFigures saved in %s\n', OUT_DIR);
end

% =====================================================================
function f = new_fig(w, h)
f = figure('Color', 'w', 'Position', [80 80 w h]);
end

% =====================================================================
function save_fig(f, outDir, name)
% Vector PDF for the report and a PNG for anything that needs an image.
set(findall(f, '-property', 'FontSize'), 'FontSize', 11);
exportgraphics(f, fullfile(outDir, [name '.pdf']), 'ContentType', 'vector');
exportgraphics(f, fullfile(outDir, [name '.png']), 'Resolution', 300);
end

% =====================================================================
function draw_map(ax, lm)
% Occupancy map in world coordinates. Cell (row, col) is the square
% x in [(col-1)/10, col/10], y in [(nRows-row)/10, (nRows-row+1)/10].
[nr, nc] = size(lm);
imagesc(ax, [0.05, nc/10 - 0.05], [0.05, nr/10 - 0.05], flipud(double(lm)));
colormap(ax, flipud(gray(64)));  clim(ax, [0 1.3]);
set(ax, 'YDir', 'normal');  axis(ax, 'equal');
xlim(ax, [0, nc/10]);  ylim(ax, [0, nr/10]);
hold(ax, 'on');  box(ax, 'on');
xlabel(ax, 'x (m)');  ylabel(ax, 'y (m)');
end

% =====================================================================
function draw_obstacles(ax, obs, cols)
% Obstacles as filled circles in their label colour: 1 red, 2 green, 3 blue.
for k = 1:3
    m = obs(:,3) == k;
    if any(m)
        h = plot(ax, obs(m,1), obs(m,2), 'o', 'MarkerSize', 7, 'MarkerFaceColor', cols(k,:), ...
                 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
        if k == 1, set(h, 'HandleVisibility', 'on', 'DisplayName', 'Obstacles'); end
    end
end
end

% =====================================================================
function d = path_distance(xy, pth)
% Distance from each point to the nearest point on the polyline PTH.
a  = pth(1:end-1,:);  b = pth(2:end,:);
ab = b - a;  L2 = max(sum(ab.^2, 2), eps);
d  = zeros(size(xy, 1), 1);
for i = 1:size(xy, 1)
    s = min(1, max(0, ((xy(i,1) - a(:,1)).*ab(:,1) + (xy(i,2) - a(:,2)).*ab(:,2))./L2));
    d(i) = sqrt(min((a(:,1) + s.*ab(:,1) - xy(i,1)).^2 + (a(:,2) + s.*ab(:,2) - xy(i,2)).^2));
end
end

% =====================================================================
function d = wall_distance(xy, lm)
% Distance from each point to the nearest wall face. Only wall cells with a
% free neighbour can be nearest, so interior cells are dropped first.
[nr, ~] = size(lm);
pad  = true(size(lm) + 2);  pad(2:end-1, 2:end-1) = lm;
edge = lm & ~(pad(1:end-2, 2:end-1) & pad(3:end, 2:end-1) & pad(2:end-1, 1:end-2) & pad(2:end-1, 3:end));
[r, c] = find(edge);
cx = (c - 0.5)/10;  cy = (nr - r + 0.5)/10;      % cell centres
d  = zeros(size(xy, 1), 1);
for i = 1:size(xy, 1)
    d(i) = min(hypot(max(abs(cx - xy(i,1)) - 0.05, 0), max(abs(cy - xy(i,2)) - 0.05, 0)));
end
end
