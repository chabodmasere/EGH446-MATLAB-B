function wp_list = wp_gen(coord_lo, coord_hi, N, x0)
%WP_GEN  N distinct waypoints on the integer grid in [coord_lo, coord_hi].
%
%   WP_LIST = WP_GEN(COORD_LO, COORD_HI, N, X0)
%
%   Draws without replacement and excludes the start pose X0. Both matter:
%   the ACO heuristic term is (1/d)^beta, so a zero-length leg makes it Inf,
%   normalisation turns that into NaN, and the tour construction silently
%   falls through to its fallback branch instead of erroring.

    if nargin < 4 || isempty(x0)
        x0 = [0 0];
    end

    cells = (coord_hi - coord_lo + 1)^2;
    if N > cells - 1
        error('wp_gen:tooMany', ...
              'Cannot draw %d distinct waypoints from %d cells.', N, cells);
    end

    wp_list = zeros(N, 2);
    n = 0;

    while n < N
        c = [randi([coord_lo, coord_hi]), randi([coord_lo, coord_hi])];

        if isequal(c, x0) || (n > 0 && any(all(wp_list(1:n,:) == c, 2)))
            continue
        end

        n = n + 1;
        wp_list(n,:) = c;
    end
end