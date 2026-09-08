function [d, name] = aco_defaults(set_name)
%ACO_DEFAULTS  Tuned ACO parameter sets. Single source of truth.
%
%   [D, NAME] = ACO_DEFAULTS()          returns the ACTIVE set
%   [D, NAME] = ACO_DEFAULTS('N10')     returns a named set
%
%   wp_antColony reads its defaults from here and run_all logs them, so the
%   parameters in the results can never disagree with the ones in the model.
%   Switch configurations by editing ACTIVE below.
%
%   Both tuned sets came from unconstrained Bayesian optimisation over all
%   seven parameters, 100 evaluations, objective mean optimality gap against
%   exact solutions, scored on held-out instances. The two differ in the
%   direction that matters: at N = 20 the search reaches for a stronger
%   pheromone weighting (alpha 1.19 against 0.31), slower evaporation
%   (rho 0.23 against 0.62) and a far lower clamp floor (tau_max 2.5 against
%   92). At N = 10 the 8280-construction budget is ample and the trail is
%   nearly irrelevant; at N = 20 it is load-bearing.

    ACTIVE = 'N10';

    if nargin < 1 || isempty(set_name)
        set_name = ACTIVE;
    end
    name = set_name;

    switch set_name

        case 'N20'
            % Tuned at N = 20. Held out: 66.6% optimal, 0.2749% mean gap
            % (original hand-picked set: 53.8%, 0.5457%).
            % Cost 10094 tour constructions.
            d = struct('iterations', 206, 'ants', 49, ...
                       'evaporation_coef', 0.233250, 'Q', 75.7743, ...
                       'alpha', 1.19490, 'beta', 3.05030, ...
                       'pheromone_max', 2.48130);

        case 'N10'
            % Tuned at N = 10. Held out: 97.7% optimal, 0.024% mean gap over
            % 5000 trials (original hand-picked set: 92.3%, 0.096%).
            % Cost 8280 tour constructions.
            d = struct('iterations', 230, 'ants', 36, ...
                       'evaporation_coef', 0.615022, 'Q', 188.5673, ...
                       'alpha', 0.308978, 'beta', 2.66729, ...
                       'pheromone_max', 91.9699);

        case 'original'
            % Hand-selected starting point, kept for comparison.
            % 92.3% optimal, 0.096% mean gap at N = 10.
            d = struct('iterations', 150, 'ants', 20, ...
                       'evaporation_coef', 0.15, 'Q', 20, ...
                       'alpha', 1, 'beta', 3, 'pheromone_max', 10);

        otherwise
            error('aco_defaults:unknown', 'Unknown parameter set "%s".', set_name);
    end
end