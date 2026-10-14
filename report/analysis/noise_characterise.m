function noise_characterise(files, outFile)
% NOISE_CHARACTERISE  Statistics of the simulator's pose-measurement noise.
%
%   noise_characterise(files, outFile)
%
%   FILES are mission logs from batch_missions. The noise is the measured
%   pose ('pose_true', what the sensors and the estimator see) minus the
%   noise-free pose ('pose_clean', the Four-Wheel Steering Simulation output
%   before noise is added), interpolated to the measurement times. Reports
%   the mean, standard deviation, axis cross-correlation, autocorrelation,
%   a normality test and the rate at which the measured pose refreshes.
%   The speed noise is the measured forward speed (component 1 of the
%   vehicle's speed output, the estimator's fourth input) minus the true
%   speed from differencing the noise-free pose.
E = [];  chg = [];  dtm = [];  V = [];
for k = 1:numel(files)
    r = load(files{k});
    tr = interp1(r.pct, r.pc(:, 1:3), r.ptt, 'linear', 'extrap');
    e  = r.pt(:, 1:3) - tr;
    e(:, 3) = atan2(sin(e(:, 3)), cos(e(:, 3)));
    E  = [E; e]; %#ok<AGROW>
    d  = any(diff(r.pose_meas) ~= 0, 2);       % estimator input, 100 Hz
    chg = [chg; d]; %#ok<AGROW>
    dtm = [dtm; median(diff(r.ptt))]; %#ok<AGROW>
    if isfield(r, 'spd')
        dt = diff(r.pct);
        v  = hypot(diff(r.pc(:, 1)), diff(r.pc(:, 2)))./dt;
        vt = interp1(r.pct(1:end-1) + dt/2, v, r.spdt, 'linear', nan);
        ev = r.spd(:, 1) - vt;
        V  = [V; ev(~isnan(ev))]; %#ok<AGROW>
    end
end
fid = fopen(outFile, 'w');
pr = @(varargin) fprintf(fid, varargin{:});
pr('missions %d, samples %d, measured-pose sample time %.3f s\n', numel(files), size(E, 1), median(dtm));
pr('mean   x %.4f m  y %.4f m  theta %.3f deg\n', mean(E(:,1)), mean(E(:,2)), rad2deg(mean(E(:,3))));
pr('std    x %.4f m  y %.4f m  theta %.3f deg\n', std(E(:,1)), std(E(:,2)), rad2deg(std(E(:,3))));
pr('planar RMS %.4f m\n', sqrt(mean(E(:,1).^2 + E(:,2).^2)));
c = corrcoef(E(:,1), E(:,2));  pr('corr(x, y) %.3f\n', c(1, 2));
for lag = 1:3
    a = arrayfun(@(i) corr_lag(E(:, i), lag), 1:3);
    pr('autocorrelation lag %d (%.2f s): x %.3f  y %.3f  theta %.3f\n', lag, lag*median(dtm), a);
end
for i = 1:3
    z = (E(:, i) - mean(E(:, i)))/std(E(:, i));
    [~, p] = kstest(z);
    pr('axis %d: KS normality p %.3f, kurtosis %.2f\n', i, p, kurtosis(E(:, i)));
end
if ~isempty(V)
    pr('speed  mean %.4f m/s  std %.4f m/s  lag-1 autocorrelation (0.01 s) %.3f  kurtosis %.2f  (%d samples)\n', ...
       mean(V), std(V), corr_lag(V, 1), kurtosis(V), numel(V));
end
pr('estimator input refreshes on %.1f %% of 100 Hz steps (%.1f Hz)\n', 100*mean(chg), 100*mean(chg));
fclose(fid);  type(outFile);
end

function a = corr_lag(x, k)
x = x - mean(x);
a = sum(x(1:end-k).*x(1+k:end))/sum(x.^2);
end
