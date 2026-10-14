function extra_checks(simEnv, baseDir, outFile)
% EXTRA_CHECKS  Supporting numbers for the report: A* against Dijkstra,
% smoothed-path curvature, and where the closest wall pass happened.
addpath(genpath(fullfile(simEnv, 'toolboxes')));
S = load(fullfile(simEnv, 'complexMap_air_ground.mat'));  lm = S.logical_map;
wall_pts = lm;
wall_pts(:, 2:end)       = wall_pts(:, 2:end)       | lm(:, 1:end-1);
wall_pts(1:end-1, :)     = wall_pts(1:end-1, :)     | lm(2:end, :);
wall_pts(1:end-1, 2:end) = wall_pts(1:end-1, 2:end) | lm(2:end, 1:end-1);
occ = inflate_map(wall_pts, 6);
fid = fopen(outFile, 'w');
nA = [];  nD = [];  tA = [];  tD = [];  dl = [];
for seed = 1:8
    r = load(fullfile(baseDir, sprintf('base_%02d.mat', seed)));
    route = [r.xh(1,1:2)*0 + [2 2]; r.wp_ordered];
    for i = 1:size(route, 1) - 1
        t0 = tic; [~, d1, n1] = A_star_count(route(i,:), route(i+1,:), occ, false, 1); t1 = toc(t0);
        t0 = tic; [~, d0, n0] = A_star_count(route(i,:), route(i+1,:), occ, false, 0); t2 = toc(t0);
        nA(end+1) = n1; nD(end+1) = n0; tA(end+1) = t1; tD(end+1) = t2; dl(end+1) = abs(d1 - d0); %#ok<AGROW>
    end
end
fprintf(fid, 'A* vs Dijkstra over %d legs (missions 1-8)\n', numel(nA));
fprintf(fid, '  points expanded: A* mean %.0f, Dijkstra mean %.0f, ratio %.2f\n', mean(nA), mean(nD), mean(nD)/mean(nA));
fprintf(fid, '  time per leg:    A* mean %.3f s, Dijkstra mean %.3f s\n', mean(tA), mean(tD));
fprintf(fid, '  largest path length difference %.4f m\n', max(dl));
kap = [];  cm = [];
f = dir(fullfile(baseDir, 'base_*.mat'));
for k = 1:numel(f)
    r = load(fullfile(f(k).folder, f(k).name));
    kap = [kap, [r.pso_info.kappa_max]];  cm = [cm, [r.pso_info.clr_min]]; %#ok<AGROW>
end
fprintf(fid, 'PSO legs: %d, peak curvature mean %.2f, max %.2f 1/m, legs over 1.0: %d; min wall clearance of smoothed legs: mean %.2f, min %.2f m\n', numel(kap), mean(kap), max(kap), sum(kap > 1.0), mean(cm), min(cm));
% closest wall pass (mission 11)
r = load(fullfile(baseDir, 'base_11.mat'));
p = interp1(r.t, r.xh(:,1:2), r.ptt, 'linear', 'extrap');
[nr, ~] = size(lm);
pad = true(size(lm) + 2);  pad(2:end-1, 2:end-1) = lm;
edge = lm & ~(pad(1:end-2, 2:end-1) & pad(3:end, 2:end-1) & pad(2:end-1, 1:end-2) & pad(2:end-1, 3:end));
[rr, cc] = find(edge);  cx = (cc - 0.5)/10;  cy = (nr - rr + 0.5)/10;
wd = @(q) min(hypot(max(abs(cx - q(1)) - 0.05, 0), max(abs(cy - q(2)) - 0.05, 0)));
d = arrayfun(@(i) wd(p(i,:)), (1:size(p,1))');
[dm, im] = min(d);
P = r.wp_plot;  step = round(r.ptt(im)*100);
for q = 1:numel(r.history), if r.history(q).step <= step, P = r.history(q).P; end, end
[xt, ip] = min(hypot(P(:,1) - p(im,1), P(:,2) - p(im,2)));
dw = min(hypot(r.wp_ordered(:,1) - p(im,1), r.wp_ordered(:,2) - p(im,2)));
fprintf(fid, 'mission 11 closest wall pass: %.3f m at t = %.1f s, position (%.2f, %.2f); active path there is %.2f m from the wall; vehicle %.2f m off the path; nearest waypoint %.2f m away; replans before: %d\n', ...
        dm, r.ptt(im), p(im,1), p(im,2), wd(P(ip,:)), xt, dw, sum([r.history.step] <= step));
fclose(fid);  type(outFile);
end
