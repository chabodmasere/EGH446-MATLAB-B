function G = kf_tune(files, outFile)
% KF_TUNE  Replay the estimator offline on logged measurements and score it
% against the noise-free pose, over a grid of noise settings.
%
%   G = kf_tune(files, outFile)
%
%   FILES are mission logs from batch_missions (they must contain pose_meas,
%   spd, pc). The filter below is the estimator block's code with R and Q
%   exposed. Run on fixed data, every setting sees the same measurements, so
%   differences are due to the filter alone. Also reports the normalised
%   innovation squared (NIS) of the pose update: a consistent filter has a
%   mean NIS equal to the measurement dimension, 3.
sxy = [0.10 0.14 0.20 0.316 0.45];
qv  = [0.05 0.2 0.5 2];
qw  = [1 5 20];
D = cell(numel(files), 1);
for k = 1:numel(files)
    r = load(files{k});
    spd = r.spd;
    if size(spd, 2) > 1, spd = spd(:, 1); end      % the filter's speed input is component 1 of the vehicle speed output
    z = [r.pose_meas, spd];
    if size(z, 1) ~= numel(r.t), error('pose_meas and xhat_log differ in length'); end
    D{k} = struct('z', z, 't', r.t, 'pc', r.pc, 'pct', r.pct, 'xh', r.xh);
end
G = [];
for a = sxy
    for b = qv
        for c = qw
            e = [];  eh = [];  nis = [];
            for k = 1:numel(D)
                [xh, ni] = lkf(D{k}.z, 0.01, a, deg2rad(sqrt(2)), b, c);
                p  = interp1(D{k}.t, xh, D{k}.pct, 'linear', 'extrap');
                e  = [e;  hypot(p(:,1) - D{k}.pc(:,1), p(:,2) - D{k}.pc(:,2))]; %#ok<AGROW>
                eh = [eh; atan2(sin(p(:,3) - D{k}.pc(:,3)), cos(p(:,3) - D{k}.pc(:,3)))]; %#ok<AGROW>
                nis = [nis; ni]; %#ok<AGROW>
            end
            G = [G; a, b, c, sqrt(mean(e.^2)), prctile(e, 99), rad2deg(sqrt(mean(eh.^2))), mean(nis)]; %#ok<AGROW>
        end
    end
end
G = array2table(G, 'VariableNames', {'sig_xy', 'q_vxy', 'q_omega', 'pos_rms', 'pos_p99', 'head_rms_deg', 'nis_mean'});
G = sortrows(G, 'pos_rms');
% the block as logged, and the raw measurement, for reference
e0 = [];  er = [];
for k = 1:numel(D)
    p  = interp1(D{k}.t, D{k}.xh, D{k}.pct, 'linear', 'extrap');
    e0 = [e0; hypot(p(:,1) - D{k}.pc(:,1), p(:,2) - D{k}.pc(:,2))]; %#ok<AGROW>
    q  = interp1(D{k}.t, D{k}.z(:,1:2), D{k}.pct, 'previous', 'extrap');
    er = [er; hypot(q(:,1) - D{k}.pc(:,1), q(:,2) - D{k}.pc(:,2))]; %#ok<AGROW>
end
fid = fopen(outFile, 'w');
fprintf(fid, 'missions %d; logged estimator: pos RMS %.4f m, p99 %.4f m; raw pose: RMS %.4f m\n', numel(D), sqrt(mean(e0.^2)), prctile(e0, 99), sqrt(mean(er.^2)));
fprintf(fid, '%s\n', strjoin(G.Properties.VariableNames, ', '));
for i = 1:height(G)
    fprintf(fid, '%.3f, %.2f, %.0f, %.4f, %.4f, %.3f, %.2f\n', G{i, :});
end
fclose(fid);  type(outFile);
end

function [X, nis] = lkf(Z, dt, sig_xy, sig_theta, q_vxy, q_omega)
% Same filter as the 'State Estimation and Filtering' block.
sig_v = 0.1;  q_theta = 0.005;  q_v = 0.5;
R = diag([sig_xy^2, sig_xy^2, sig_theta^2, sig_v^2]);
Q = zeros(7);
Q(4,4) = q_v*dt;
Q(3,3) = q_omega*dt^3/3 + q_theta*dt;  Q(3,5) = q_omega*dt^2/2;  Q(5,3) = Q(3,5);  Q(5,5) = q_omega*dt;
Q(1,1) = q_vxy*dt^3/3;  Q(1,6) = q_vxy*dt^2/2;  Q(6,1) = Q(1,6);  Q(6,6) = q_vxy*dt;
Q(2,2) = q_vxy*dt^3/3;  Q(2,7) = q_vxy*dt^2/2;  Q(7,2) = Q(2,7);  Q(7,7) = q_vxy*dt;
A = eye(7);  A(1,6) = dt;  A(2,7) = dt;  A(3,5) = dt;
H = [eye(4), zeros(4,3)];
n = size(Z, 1);  X = zeros(n, 3);  nis = zeros(0, 1);
z = Z(1,:)';
x = [z(1); z(2); z(3); z(4); 0; z(4)*cos(z(3)); z(4)*sin(z(3))];
P = diag([sig_xy^2, sig_xy^2, sig_theta^2, sig_v^2, 1, 1, 1]);
zp = z;
for i = 1:n
    z = Z(i,:)';
    newp = i == 1 || any(z(1:3) ~= zp(1:3));
    zp = z;
    x = A*x;  x(3) = wrap(x(3));  P = A*P*A' + Q;
    if newp
        in = z - H*x;  in(3) = wrap(in(3));
        S = H*P*H' + R;  S = (S + S')/2;
        K = P*H'/S;
        if i > 1
            nis(end+1, 1) = in(1:3)'/S(1:3,1:3)*in(1:3); %#ok<AGROW>
        end
    else
        h = H(4,:);  S = h*P*h' + R(4,4);  K = P*h'/S;  in = z(4) - h*x;  H_ = h;
    end
    if newp, H_ = H; Rk = R; else, Rk = R(4,4); end
    x = x + K*in;
    I = eye(7) - K*H_;  P = I*P*I' + K*Rk*K';  P = (P + P')/2;
    x(3) = wrap(x(3));
    X(i,:) = x(1:3)';
end
end

function a = wrap(a)
a = atan2(sin(a), cos(a));
end
