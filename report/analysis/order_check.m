% ORDER_CHECK  Would ordering the waypoints by straight-line distance give a
% longer drive? For each logged mission, finds the best tour under the
% straight-line cost and under the A* driving-distance cost (all 120
% orders of five waypoints), and reports how much longer the straight-line
% tour is to drive. Used for the 13 of 25 missions / 8.6% / 32% figures in
% the report (Waypoint Ordering).
%
%   order_check(dirn)   dirn holds the base_XX.mat files from batch_missions

function order_check(dirn)
f = dir(fullfile(dirn, 'base_*.mat'));  ex = [];  diffn = 0;
P = perms(2:6);
for k = 1:numel(f)
    r = load(fullfile(f(k).folder, f(k).name));
    pts = [2 2; r.wp_list];
    E = hypot(pts(:,1) - pts(:,1)', pts(:,2) - pts(:,2)');
    bestD = inf; bestE = inf; tE = [];
    for q = 1:size(P,1)
        t = [1 P(q,:)]; id = sub2ind([6 6], t(1:end-1), t(2:end));
        cD = sum(r.D(id)); cE = sum(E(id));
        if cD < bestD, bestD = cD; end
        if cE < bestE, bestE = cE; tE = t; end
    end
    idE = sub2ind([6 6], tE(1:end-1), tE(2:end));
    ex(end+1) = 100*(sum(r.D(idE))/bestD - 1);
end
fprintf('straight-line order longer to drive in %d of %d missions; mean %.1f%%, max %.1f%%\n', sum(ex > 1e-6), numel(ex), mean(ex), max(ex));
end
