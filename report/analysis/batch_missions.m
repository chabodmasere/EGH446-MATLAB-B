function batch_missions(simEnv, shimDir, outDir, cacheDir, seeds, tag, override)
% BATCH_MISSIONS  Run seeded missions and save the logs the report needs.
%
%   batch_missions(simEnv, shimDir, outDir, cacheDir, seeds, tag, override)
%
%   simEnv    the sim_env folder
%   shimDir   report/analysis/logging_shims (logging copies of path_store and
%             controller_params; MATLAB runs from there so they take priority)
%   seeds     rng seeds, one mission each
%   tag       prefix of the saved files
%   override  struct with optional fields v_max, capture_radius ([] for none),
%             kf_sig_xy, kf_sig_theta_deg, kf_q_vxy, kf_q_omega (estimator
%             noise), avoid_file (avoidance block code); all this run only
%
%   Besides the model's own logs, three signals are logged for evaluation
%   only (they feed nothing in the loop):
%     pose_clean  the vehicle pose BEFORE the simulator adds its sensor
%                 noise (output of Four-Wheel Steering Simulation), used as
%                 ground truth for clearance and estimation error
%     ranges      the lidar ranges, to measure wall-repulsion activity
%     dist_to_wp  Guidance output 2, which sets the repulsion reach
global REPORT_OVERRIDE
REPORT_OVERRIDE = override;
set(0, 'DefaultFigureVisible', 'off');     % no windows popping up during the batch
Simulink.fileGenControl('set', 'CacheFolder', cacheDir, 'CodeGenFolder', cacheDir);
addpath(fileparts(mfilename('fullpath')));
addpath(simEnv);  addpath(genpath(fullfile(simEnv, 'toolboxes')));
cd(shimDir);
assert(startsWith(which('path_store'), shimDir) && startsWith(which('controller_params'), shimDir));
mdl = 'sl_groundvehicleDynamics';
load_system(mdl);
log_port([mdl '/Vehicle1/Four-Wheel Steering Simulation'], 1, 'pose_clean');
log_port([mdl '/Lidar Sensor'], 1, 'ranges');
log_port([mdl '/Guidance'], 2, 'dist_to_wp');
log_port([mdl '/Vehicle1'], 2, 'speed_meas');
if isstruct(override) && any(isfield(override, {'kf_sig_xy', 'kf_sig_theta_deg', 'kf_q_vxy', 'kf_q_omega'}))
    set_kf_noise(mdl, override);
end
if isstruct(override) && isfield(override, 'avoid_file')
    % candidate avoidance code, this session only (the model is not saved)
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', '-regexp', 'Script', 'v_floor');
    assert(numel(ch) == 1, 'avoidance block not found');
    ch.Script = fileread(override.avoid_file);
end
for seed = seeds
    if exist(fullfile(outDir, sprintf('%s_%02d.mat', tag, seed)), 'file'), continue, end   % already done
    try
        rng(seed);
        t0 = tic; out = sim(mdl, 'StopTime', '600'); wall = toc(t0);
        r = struct('seed', seed, 'wall', wall, 'tend', out.tout(end), 'override', override);
        xl = out.xhat_log; r.t = xl.Time(:); r.xh = squeeze(xl.Data)';
        d = out.d_obs_log; if isa(d, 'timeseries'), d = d.Data; end, r.d_obs = squeeze(d);
        ls = out.logsout;
        g = @(n) ls.getElement(n).Values;
        pt = g('pose_true'); r.pt = pt.Data; r.ptt = pt.Time(:);
        pc = g('pose_clean'); r.pc = squeeze(pc.Data); r.pct = pc.Time(:);
        if size(r.pc, 1) == 3 && size(r.pc, 2) ~= 3, r.pc = r.pc'; end
        rg = g('ranges'); r.ranges = squeeze(rg.Data); r.rgt = rg.Time(:);
        if size(r.ranges, 1) == 13 && size(r.ranges, 2) ~= 13, r.ranges = r.ranges'; end
        dw = g('dist_to_wp'); r.dwp = squeeze(dw.Data); r.dwpt = dw.Time(:);
        sp = g('speed_meas'); r.spd = squeeze(sp.Data); r.spdt = sp.Time(:);
        ce = g('capture_error'); ev = g('capture_event'); k = find(ev.Data > 0.5);
        r.cap_err = ce.Data(k); r.cap_t = ev.Time(k);
        r.wp_idx = squeeze(g('wp_idx_out').Data); r.v_cmd = squeeze(g('v_cmd').Data);
        pl = out.pose_log; r.pose_meas = squeeze(pl.Data);
        b = @(n) evalin('base', n);
        r.wp_ordered = b('wp_ordered'); r.wp_list = b('wp_list'); r.D = b('D');
        r.astar_path = b('astar_path'); r.wp_plot = b('wp_plot');
        r.astar_dist = b('astar_dist'); r.path_dist = b('path_dist');
        r.pso_info = rmfield(b('pso_info'), 'history');
        r.aco_history = b('history'); r.dist_cum = b('dist_cum');
        r.v_max = b('v_max'); r.capture_radius = b('capture_radius');
        r.replans = path_store('version');
        r.history = path_store('history');
        r.replan_cpu = obstacle_replan('stats');
        if seed == 1 || ~exist(fullfile(outDir, 'map.mat'), 'file')
            obstacles = b('obstacles'); logical_map = b('logical_map');
            save(fullfile(outDir, 'map.mat'), 'obstacles', 'logical_map');
        end
        save(fullfile(outDir, sprintf('%s_%02d.mat', tag, seed)), '-struct', 'r');
        fprintf('MISSION %s %2d: sim %.1f s, wall %.0f s, replans %d, captures %d\n', tag, seed, r.tend, wall, numel(r.history), numel(r.cap_err));
    catch e
        fprintf('MISSION %s %2d: FAILED %s\n', tag, seed, strrep(getReport(e, 'basic', 'hyperlinks', 'off'), newline, ' '));
    end
end
fprintf('BATCH_DONE %s\n', tag);
end

function log_port(blk, port, name)
ph = get_param(blk, 'PortHandles');
set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

function set_kf_noise(mdl, o)
% Replace the estimator's measurement-noise lines for this session only
% (the model is not saved).
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/State Estimation and Filtering/MATLAB Function']);
s = ch.Script;
if isfield(o, 'kf_sig_xy')
    s = regexprep(s, 'sig_xy\s*=\s*[^;]*;', sprintf('sig_xy    = %.6g;', o.kf_sig_xy), 'once');
end
if isfield(o, 'kf_sig_theta_deg')
    s = regexprep(s, 'sig_theta\s*=\s*[^;]*;', sprintf('sig_theta = %.6g * pi/180;', o.kf_sig_theta_deg), 'once');
end
if isfield(o, 'kf_q_vxy')
    s = regexprep(s, 'q_vx\s*=\s*[^;]*;', sprintf('q_vx    = %.6g;', o.kf_q_vxy), 'once');
    s = regexprep(s, 'q_vy\s*=\s*[^;]*;', sprintf('q_vy    = %.6g;', o.kf_q_vxy), 'once');
end
if isfield(o, 'kf_q_omega')
    s = regexprep(s, 'q_omega\s*=\s*[^;]*;', sprintf('q_omega = %.6g;', o.kf_q_omega), 'once');
end
ch.Script = s;
end
