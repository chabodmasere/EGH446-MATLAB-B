% ADD_REPLANNER  One-off model edit for online obstacle re-planning.
% Run from sim_env with guidance_block.txt in the same folder. It:
%   1. replaces the Guidance block code (old code saved to guidance_block_old.txt)
%   2. cuts the avoidance block's obstacle reach from 2.0 m to 0.8 m, so it
%      only acts as a backup when the robot gets too close to an obstacle
%      (old code saved to avoidance_block_old.txt)
%   3. adds a 'Replanner' MATLAB Function block and wires it
% Nothing is saved: check the model, run it, then save it yourself.

mdl = 'sl_groundvehicleDynamics';
open_system(mdl);
cfg = @(p) get_param([mdl '/' p], 'MATLABFunctionConfiguration');

%% 1. Guidance code
g = cfg('Guidance/MATLAB Function');
write_text('guidance_block_old.txt', g.FunctionScript);
g.FunctionScript = fileread('guidance_block.txt');
disp('1. Guidance code replaced.');

%% 2. Avoidance obstacle reach
a = cfg('MATLAB Function');
old = 'd_inf   = 2.00;   % [m] start reacting to an obstacle at this range (far from goal)';
new = 'd_inf   = 0.80;   % [m] start reacting to an obstacle (backup: the Replanner steers around them)';
if contains(a.FunctionScript, old)
    write_text('avoidance_block_old.txt', a.FunctionScript);
    a.FunctionScript = strrep(a.FunctionScript, old, new);
    disp('2. Avoidance obstacle reach set to 0.8 m.');
else
    disp('2. SKIPPED: d_inf line not found. Set d_inf = 0.80 by hand in the avoidance block.');
end

%% 3. Replanner block
blk = [mdl '/Replanner'];
if getSimulinkBlockHandle(blk) > 0
    error('A Replanner block already exists. Delete it and re-run.');
end
p = get_param([mdl '/MATLAB Function'], 'Position');
add_block('simulink/User-Defined Functions/MATLAB Function', blk, ...
          'Position', [p(1), p(4) + 80, p(3), p(4) + 80 + 90]);
r = cfg('Replanner');
r.FunctionScript = sprintf([ ...
    'function replan(x_hat, detections, wp_idx)\n' ...
    '%%#codegen\n' ...
    '%% Re-plans the path around detected obstacles (see obstacle_replan.m).\n' ...
    'coder.extrinsic(''obstacle_replan'');\n' ...
    'obstacle_replan(''step'', x_hat, detections, wp_idx);\n' ...
    'end\n']);

add_line(mdl, 'State Estimation and Filtering/1', 'Replanner/1', 'autorouting', 'smart');
add_line(mdl, 'Object Detector/1',                'Replanner/2', 'autorouting', 'smart');
add_line(mdl, 'Guidance/4',                       'Replanner/3', 'autorouting', 'smart');
disp('3. Replanner block added and wired: x_hat, detections, wp_idx_out.');

disp('Done. Check the wiring, run the sim, then save the model.');

function write_text(f, s)
    fid = fopen(f, 'w');
    fwrite(fid, s);
    fclose(fid);
end
