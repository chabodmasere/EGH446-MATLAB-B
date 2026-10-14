function T = quick_eval(dirs, tags)
% QUICK_EVAL  Compact comparison of mission sets against the noise-free pose.
%
%   T = quick_eval({dir1, dir2, ...}, {tag1, tag2, ...})
%
%   One row per set: completion, time, worst and mean minimum clearance from
%   the TRUE (noise-free) pose, estimation error, and re-plan statistics.
OBS_R = 0.15;  CLEAR = 0.40;
rows = {};
for s = 1:numel(tags)
    f = dir(fullfile(dirs{s}, [tags{s} '_*.mat']));
    mp = load(fullfile(dirs{s}, 'map.mat'));  obs = mp.obstacles;  lm = mp.logical_map;
    R = nan(numel(f), 10);
    for k = 1:numel(f)
        r = load(fullfile(f(k).folder, f(k).name));
        pc = r.pc(:, 1:2);
        dO = min(hypot(pc(:,1) - obs(:,1)', pc(:,2) - obs(:,2)'), [], 2) - OBS_R;
        dW = wall_distance(pc, lm);
        xh = interp1(r.t, r.xh, r.pct, 'linear', 'extrap');
        e  = hypot(xh(:,1) - pc(:,1), xh(:,2) - pc(:,2));
        eth = rad2deg(abs(atan2(sin(xh(:,3) - r.pc(:,3)), cos(xh(:,3) - r.pc(:,3)))));
        nw = size(r.wp_ordered, 1);
        done = numel(r.cap_err) >= nw && r.tend < 599;
        R(k, :) = [done, r.tend, min(dO), min(dW), sqrt(mean(e.^2)), max(e), sqrt(mean(eth.^2)), ...
                   numel(r.history), max([r.replan_cpu; 0]), max([r.pso_info.kappa_max])];
    end
    rows(end+1, :) = {tags{s}, numel(f), sum(R(:,1)), mean(R(:,2)), min(R(:,3)), mean(R(:,3)), ...
                      min(R(:,4)), mean(R(:,4)), sum(R(:,3) < CLEAR | R(:,4) < CLEAR), ...
                      mean(R(:,5)), max(R(:,6)), mean(R(:,7)), mean(R(:,8)), max(R(:,9)), max(R(:,10))}; %#ok<AGROW>
end
T = cell2table(rows, 'VariableNames', {'set', 'n', 'done', 'time', 'obs_worst', 'obs_mean', ...
    'wall_worst', 'wall_mean', 'below', 'est_rms', 'est_max', 'head_rms_deg', 'replans', 'cpu_max', 'kappa_max'});
disp(T);
end

function d = wall_distance(xy, lm)
% Distance from each point to the nearest wall face.
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
