function varargout = path_store(cmd, varargin)
%PATH_STORE  Shared copy of the planned path, so the replanner can hand
%   Guidance a new path mid-run.
%
%   path_store('reset')                       clear (mdl_robot, every run)
%   path_store('set', path, wp_seg, restart)  publish a new path
%   v = path_store('version')                 bumps on every 'set'
%   [P, n, wp_seg, restart] = path_store('get', maxp)
%
%   WP_SEG(i) is the path index of waypoint i. RESTART is the path index
%   Guidance should resume from, or 0 to keep its current progress.
%   P is padded to MAXP rows because Guidance needs a fixed size.

    persistent P seg rst ver
    if isempty(ver)
        ver = 0;
    end

    switch cmd
        case 'reset'
            P = [];  seg = [];  rst = 0;  ver = 0;

        case 'set'
            P   = varargin{1};
            seg = varargin{2}(:);
            rst = varargin{3};
            ver = ver + 1;

        case 'version'
            varargout{1} = ver;

        case 'get'
            maxp = varargin{1};
            n = min(size(P, 1), maxp);
            out = zeros(maxp, 2);
            out(1:n, :) = P(1:n, :);
            varargout = {out, n, seg, rst};
    end
end
