function varargout = obstacle_replan(cmd, varargin)
%OBSTACLE_REPLAN  Stub used only by the report's ablation run: the online
%   re-planner is switched off, so the reactive layer is the only obstacle
%   avoidance. 'init' still resets the shared path store; 'step' does nothing.
    switch cmd
        case 'init'
            path_store('reset');
        case 'stats'
            varargout{1} = zeros(0, 1);
    end
end
