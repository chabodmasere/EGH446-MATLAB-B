function varargout = path_store(cmd, varargin)
%PATH_STORE  Logging copy of toolboxes/ground_robot/path_store.m, used only
%   by the report's batch runs. Behaviour is identical; it also keeps every
%   path the Replanner publishes, with the simulation step it arrived on.
%
%   H = path_store('history')   struct array: step, P, restart
%   Guidance asks for 'version' once per 10 ms step, so the number of
%   'version' calls since 'reset' is the step count.

    persistent P seg rst ver H cnt
    if isempty(ver)
        ver = 0;  cnt = 0;  H = struct('step', {}, 'P', {}, 'restart', {});
    end

    switch cmd
        case 'reset'
            P = [];  seg = [];  rst = 0;  ver = 0;  cnt = 0;
            H = struct('step', {}, 'P', {}, 'restart', {});

        case 'set'
            P   = varargin{1};
            seg = varargin{2}(:);
            rst = varargin{3};
            ver = ver + 1;
            H(end+1) = struct('step', cnt, 'P', P, 'restart', rst);

        case 'version'
            cnt = cnt + 1;
            varargout{1} = ver;

        case 'get'
            maxp = varargin{1};
            n = min(size(P, 1), maxp);
            out = zeros(maxp, 2);
            out(1:n, :) = P(1:n, :);
            varargout = {out, n, seg, rst};

        case 'history'
            varargout{1} = H;
    end
end
