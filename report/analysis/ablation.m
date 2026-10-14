function ablation(baseDir, sweepDir, outFile)
% ABLATION  Missions 1-8 with the re-planner switched off (reactive layer
% only), against the same missions with both layers.
m1 = load(fullfile(baseDir, 'map.mat'));  obs = m1.obstacles;  lm = m1.logical_map;
[nr, ~] = size(lm);
pad = true(size(lm) + 2);  pad(2:end-1, 2:end-1) = lm;
edge = lm & ~(pad(1:end-2, 2:end-1) & pad(3:end, 2:end-1) & pad(2:end-1, 1:end-2) & pad(2:end-1, 3:end));
[rr, cc] = find(edge);  cx = (cc - 0.5)/10;  cy = (nr - rr + 0.5)/10;
fid = fopen(outFile, 'w');
sets = {'base', baseDir; 'norep', sweepDir};
for s = 1:2
    R = nan(8, 5);
    for seed = 1:8
        f = fullfile(sets{s, 2}, sprintf('%s_%02d.mat', sets{s, 1}, seed));
        if ~exist(f, 'file'), continue, end
        r = load(f);
        if isfield(r, 'pc')
            p = r.pc(:, 1:2);                      % noise-free pose (ground truth)
        else
            p = interp1(r.t, r.xh(:,1:2), r.ptt, 'linear', 'extrap');
        end
        dO = min(hypot(p(:,1) - obs(:,1)', p(:,2) - obs(:,2)'), [], 2) - 0.15;
        dW = arrayfun(@(i) min(hypot(max(abs(cx - p(i,1)) - 0.05, 0), max(abs(cy - p(i,2)) - 0.05, 0))), (1:size(p,1))');
        R(seed, :) = [r.tend, min(dO), min(dW), sum(isfinite(r.d_obs(:)))*0.01, numel(r.cap_err)];
    end
    fprintf(fid, '%s: time %.1f s, min obstacle clearance mean %.3f worst %.3f, worst wall %.3f, below 0.40: obstacle %d wall %d of %d, reactive active mean %.2f s (max %.2f), all 5 captured in %d\n', ...
        sets{s, 1}, mean(R(:,1), 'omitnan'), mean(R(:,2), 'omitnan'), min(R(:,2)), min(R(:,3)), sum(R(:,2) < 0.40), sum(R(:,3) < 0.40), sum(~isnan(R(:,1))), mean(R(:,4), 'omitnan'), max(R(:,4)), sum(R(:,5) >= 5));
    fprintf(fid, '   per mission min obstacle: %s\n', mat2str(round(R(:,2)', 2)));
end
fclose(fid);  type(outFile);
end
