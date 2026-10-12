% ADD_AVOID_VIZ  One-off: adds the 'avoid_viz' block to the model and wires it.
% Run once from sim_env with the model open, check the wiring, then save.

mdl = 'sl_groundvehicleDynamics';
open_system(mdl);
blk = [mdl '/avoid_viz'];

if getSimulinkBlockHandle(blk) > 0
    error('avoid_viz already exists. Delete it first to re-add.');
end

p = get_param([mdl '/lidar_viz'], 'Position');
add_block('simulink/User-Defined Functions/MATLAB Function', blk, ...
          'Position', [p(1), p(4) + 60, p(3), p(4) + 60 + 140]);

rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = sprintf([ ...
    'function avoid_viz(pose, theta_cmd, theta_safe, avoid_active, d_obs, v, omega)\n' ...
    '%% Draws the obstacle avoidance overlay (see draw_avoid_view.m).\n' ...
    'coder.extrinsic(''draw_avoid_view'');\n' ...
    'draw_avoid_view(pose, theta_cmd, theta_safe, avoid_active, d_obs, v, omega);\n' ...
    'end\n']);

% sources: x_hat at 20 Hz, Guidance heading, avoidance block outputs,
% applied speed (v_cmd * v_scale) and the controller's yaw rate
src = {'x_hat to Visualizer (20 Hz)/1', 'Guidance/1', 'MATLAB Function/1', ...
       'MATLAB Function/3', 'MATLAB Function/4', 'Product/1', 'Controller/2'};
for k = 1:numel(src)
    add_line(mdl, src{k}, sprintf('avoid_viz/%d', k), 'autorouting', 'smart');
end

disp('avoid_viz added. Check the wiring, run once, then save the model.');
