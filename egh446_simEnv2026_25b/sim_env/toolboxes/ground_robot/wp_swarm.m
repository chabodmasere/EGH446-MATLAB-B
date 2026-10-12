function [path, info, out] = wp_swarm(legs, clr, opt)
%WP_SWARM  Smooth each A* leg with particle swarm optimisation.
%
%   [path, info] = wp_swarm(legs, clr)
%   [path, info] = wp_swarm(legs, clr, opt)
%   [path, info, out] = wp_swarm(...)   OUT holds each smoothed leg separately
%
%   LEGS is a cell array of A* paths, one per pair of consecutive ordered
%   waypoints (from wp_path). CLR is the clearance field from clearance_field.
%   Each leg becomes a cubic B-spline whose control points start on the A*
%   path and are shifted sideways by the swarm. A* fixes the topology, PSO
%   only reshapes the curve within it.
%
%   A leg whose best curve breaks the hard clearance falls back to its raw
%   A* path, so the output is never less safe than the A* route.

    def = pso_defaults();
    if nargin < 3 || isempty(opt), opt = struct(); end
    fn = fieldnames(def);
    for ii = 1:numel(fn)
        if ~isfield(opt, fn{ii}), opt.(fn{ii}) = def.(fn{ii}); end
    end

    path = legs{1}(1,:);
    out  = cell(size(legs));
    for i = 1:numel(legs)
        [p, info(i)] = pso_leg(legs{i}, clr, opt); %#ok<AGROW>
        out{i} = p;
        path = [path; p(2:end,:)]; %#ok<AGROW>
    end
end

function [p, info] = pso_leg(leg, clr, opt)
    S = leg(1,:);
    E = leg(end,:);

    s = [0; cumsum(hypot(diff(leg(:,1)), diff(leg(:,2))))];
    L = s(end);
    K = max(2, round(L/opt.spacing));

    % seed control points: evenly spaced along the A* path
    sk   = L*(1:K)'/(K+1);
    seed = [interp1(s, leg(:,1), sk), interp1(s, leg(:,2), sk)];

    % lateral direction at each seed, from the path tangent over +-0.5 m
    t  = [interp1(s, leg(:,1), min(sk+0.5, L)) - interp1(s, leg(:,1), max(sk-0.5, 0)), ...
          interp1(s, leg(:,2), min(sk+0.5, L)) - interp1(s, leg(:,2), max(sk-0.5, 0))];
    t  = t./hypot(t(:,1), t(:,2));
    nrm = [-t(:,2), t(:,1)];

    % optimisation: coarse samples. Output and final check: fine samples.
    m_opt = 12;
    m_out = max(4, ceil(opt.spacing/0.1));
    C  = curve_model(S, E, seed, nrm, m_opt);
    Cf = curve_model(S, E, seed, nrm, m_out);

    % swarm: each particle is K lateral offsets [m]
    nP   = opt.particles;
    band = opt.band;
    vmax = 0.5*band;

    X = 0.5*band*(2*rand(K, nP) - 1);
    X(:,1) = 0;                                   % one particle starts on the A* path
    V = zeros(K, nP);

    J = cost(C, X, clr, opt);
    pbest = X;   pJ = J;
    [gJ, gi] = min(J);
    g = X(:,gi);
    history = zeros(opt.iterations, 1);

    for it = 1:opt.iterations
        w = opt.w_start - (opt.w_start - opt.w_end)*(it-1)/max(1, opt.iterations-1);
        V = w*V + opt.c1*rand(K, nP).*(pbest - X) + opt.c2*rand(K, nP).*(g - X);
        V = min(max(V, -vmax), vmax);
        X = min(max(X + V, -band), band);

        J = cost(C, X, clr, opt);
        better = J < pJ;
        pbest(:, better) = X(:, better);
        pJ(better) = J(better);
        [jmin, gi] = min(pJ);
        if jmin < gJ
            gJ = jmin;
            g  = pbest(:,gi);
        end
        history(it) = gJ;
    end

    [~, x, y, kap, ds, c] = cost(Cf, g, clr, opt);
    p = [x, y; E];
    fallback = min(c) <= opt.c_hard;
    if fallback
        p = leg;
    end

    info = struct('K', K, 'J', gJ, 'history', history, ...
                  'len_astar', L, 'len_pso', sum(ds), ...
                  'clr_min', min(c), 'clr_min_astar', min(lookup(clr, leg(:,1), leg(:,2))), ...
                  'kappa_max', max(abs(kap)), 'fallback', fallback);
end

function C = curve_model(S, E, seed, nrm, m)
% Curve samples are linear in the offsets: r = base + G*offsets.
% Control polygon [2S-C1, S, C1..CK, E, 2E-CK]: the reflected end points make
% the spline pass through S and E with zero curvature there.
    K = size(seed, 1);
    N = K + 4;

    T = zeros(N, K+2);                % control polygon from [S; C; E]
    T(1,1) = 2;  T(1,2) = -1;
    T(2:K+3, :) = eye(K+2);
    T(N,K+2) = 2;  T(N,K+1) = -1;

    M = [-1  3 -3 1;
          3 -6  3 0;
         -3  0  3 0;
          1  4  1 0]/6;
    u  = (0:m-1)'/m;
    B0 = [u.^3, u.^2, u, ones(m,1)]*M;
    B1 = [3*u.^2, 2*u, ones(m,1), zeros(m,1)]*M;
    B2 = [6*u, 2*ones(m,1), zeros(m,2)]*M;

    nS = (N-3)*m;
    A0 = zeros(nS, N);  A1 = A0;  A2 = A0;
    for j = 1:N-3
        rows = (j-1)*m + (1:m);
        A0(rows, j:j+3) = B0;
        A1(rows, j:j+3) = B1;
        A2(rows, j:j+3) = B2;
    end
    A0 = A0*T;  A1 = A1*T;  A2 = A2*T;

    P = [S; seed; E];
    Ac = {A0, A1, A2};
    for k = 1:3
        C.bx{k} = Ac{k}*P(:,1);
        C.by{k} = Ac{k}*P(:,2);
        C.Gx{k} = Ac{k}(:, 2:K+1).*nrm(:,1)';
        C.Gy{k} = Ac{k}(:, 2:K+1).*nrm(:,2)';
    end
    C.du = 1/m;
end

function [J, x, y, kap, ds, c] = cost(C, X, clr, opt)
    x   = C.bx{1} + C.Gx{1}*X;   y   = C.by{1} + C.Gy{1}*X;
    dx  = C.bx{2} + C.Gx{2}*X;   dy  = C.by{2} + C.Gy{2}*X;
    ddx = C.bx{3} + C.Gx{3}*X;   ddy = C.by{3} + C.Gy{3}*X;

    sp  = max(hypot(dx, dy), 1e-9);
    ds  = sp*C.du;
    kap = (dx.*ddy - dy.*ddx)./sp.^3;
    c   = lookup(clr, x, y);

    J = sum(ds) ...
      + opt.k_curv*sum(kap.^2.*ds) ...
      + opt.k_kmax*sum(max(0, abs(kap) - opt.kappa_max).^2.*ds) ...
      + opt.k_soft*sum(max(0, opt.c_soft - c).^2.*ds) ...
      + opt.k_hard*sum(max(0, opt.c_hard - c).*ds);
end

function c = lookup(clr, x, y)
% clr is indexed on grid points (x, y) = ((col-1)/10, (nRows-row)/10)
    nRows = size(clr, 1);
    c = interp2(clr, x*10 + 1, nRows - y*10, 'linear', 0);
end
