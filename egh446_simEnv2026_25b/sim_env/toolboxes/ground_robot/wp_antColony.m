function [wp_ordered, dist_cum, history] = wp_antColony(wp_list, x0, D, opt)
%WP_ANTCOLONY  Order waypoints by ant colony optimisation.
%
%   [wp_ordered, dist_cum]          = wp_antColony(wp_list, x0, D)
%   [wp_ordered, dist_cum, history] = wp_antColony(wp_list, x0, D, opt)
%
%   D is an (N+1)-by-(N+1) matrix of traversable distances, node 1 being the
%   depot x0 and node w+1 being waypoint w. Build it with:
%       D = wp_distances([x0; wp_list], logical_map);
%
%   OPT optionally overrides any tuneable parameter; omitted fields fall back
%   to the deployed defaults below. HISTORY is iterations-by-2: best-so-far
%   and iteration-mean tour length.

    N = size(wp_list, 1);

    def = aco_defaults();

    if nargin < 4 || isempty(opt), opt = struct(); end
    fn = fieldnames(def);
    for ii = 1:numel(fn)
        if ~isfield(opt, fn{ii}), opt.(fn{ii}) = def.(fn{ii}); end
    end

    iterations          = opt.iterations;
    ants                = opt.ants;
    evaporation_coef    = opt.evaporation_coef;
    pheromone_retention = 1 - evaporation_coef;
    Q                   = opt.Q;
    alpha               = opt.alpha;
    beta                = opt.beta;
    pheromone_max       = opt.pheromone_max;
    pheromone_min       = pheromone_max/(2*N);
    starting_pheramones = pheromone_max;

    best_dist = inf;
    best_path = zeros(N,2);
    history   = zeros(iterations, 2);

    path_pheramones = starting_pheramones * ones(N+1);   % "pheromone" levels

    for i = 1:iterations % once finished highest pheromones is path
        ants_path = zeros(N*ants,2);
        ants_idx  = zeros(N*ants,1);   % waypoint index visited at each step
        ant_dists = zeros(ants,1);     % tour length of each ant this iteration

        for j = 1:ants % run set of ants to add pheromones
            unvisted_wps = true(N,1);
            xc = x0;
            ant_dist = 0;

            for k = 1:N % iterate through path
                % index of the current node in path_pheramones and D.
                % row/col 1 is the depot x0, row/col w+1 is waypoint w.
                % at k == 1 the ant is still at x0; after that the ant is
                % sitting on the waypoint it chose last step, so idx carries
                % over and no lookup into wp_list is needed.
                if k == 1
                    index_xc = 1;
                else
                    index_xc = idx + 1;
                end

                probabilities = zeros(N,1);
                for m = 1:N % iterate through waypoints to work out where to go
                    if ~unvisted_wps(m)
                        continue
                    end
                    % eps floor: coincident points give d_k = 0, so (1/d_k)^beta
                    % is Inf and the normalisation below yields NaN.
                    d_k = max(D(index_xc, m+1), eps);     % Nij, proximity to wp m
                    tau = path_pheramones(index_xc, m+1); % Tij, trail on this edge
                    probabilities(m) = tau^alpha * (1/d_k)^beta;
                end

                weight_total  = sum(probabilities);
                probabilities = probabilities / weight_total;

                dec = rand();
                idx = find(cumsum(probabilities) > dec, 1);
                if isempty(idx)
                    % cumsum can land a hair under 1 through float error, and
                    % with ~1e5 draws per call that does eventually happen.
                    idx = find(unvisted_wps, 1, 'last');
                end

                ant_dist = ant_dist + D(index_xc, idx+1);
                xc = wp_list(idx, :);
                ants_path((j-1)*N+k,:) = xc;
                ants_idx((j-1)*N+k)    = idx;
                unvisted_wps(idx)      = false;
            end

            ant_dists(j) = ant_dist;

            if ant_dist < best_dist
                best_dist = ant_dist;
                best_path = ants_path((j-1)*N+1 : j*N, :);
            end
        end

        path_pheramones = pheromone_retention * path_pheramones; % (1-p)*Tij(t)

        for j = 1:N*ants
            if mod(j,N) == 1
                index_c = 1;                 % leg out of the depot
            else
                index_c = ants_idx(j-1) + 1;
            end
            index_n = ants_idx(j) + 1;

            ant_no  = ceil(j/N);
            deposit = Q / ant_dists(ant_no);

            path_pheramones(index_c,index_n) = path_pheramones(index_c,index_n) + deposit;
            path_pheramones(index_n,index_c) = path_pheramones(index_n,index_c) + deposit;
        end

        path_pheramones = min(max(path_pheramones, pheromone_min), pheromone_max);

        history(i,:) = [best_dist, mean(ant_dists)];
    end

    wp_ordered = best_path;
    dist_cum   = best_dist;
end