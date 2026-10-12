function d = pso_defaults()
%PSO_DEFAULTS  PSO path smoothing parameters. Single source of truth.
%
%   Swarm
%     particles, iterations   swarm size and iteration budget per leg
%     w_start, w_end          inertia weight, decreased linearly over the run
%     c1, c2                  cognitive and social coefficients
%   Encoding
%     spacing  [m]            one B-spline control point per this much A* path
%     band     [m]            max lateral shift of a control point off the A* path
%   Cost
%     c_hard   [m]            clearance that must hold (0.6 = the A* inflation)
%     c_soft   [m]            clearance the path is pushed towards for tracking margin
%     kappa_max [1/m]         tightest curvature allowed, 1/r_min
%     k_*                     weights on curvature, curvature limit, soft and hard clearance
%
%   Turning circle: the front-steer model has no steering lock, so the limit
%   is the controller's, r = v/omega. At v_max and omega_max that is 0.8 m.
%   r_min = 1.0 m leaves omega headroom to correct tracking error mid-turn.

    v_max     = 1;
    omega_max = 2*pi/5;
    r_min     = 1.0;                    % [m], vs v_max/omega_max = 0.8 m

    d = struct('particles', 30, 'iterations', 150, ...
               'w_start', 0.9, 'w_end', 0.4, 'c1', 1.5, 'c2', 1.5, ...
               'spacing', 1.0, 'band', 1.0, ...
               'c_hard', 0.6, 'c_soft', 0.9, 'kappa_max', 1/r_min, ...
               'k_curv', 1, 'k_kmax', 100, 'k_soft', 20, 'k_hard', 1e4);
end
