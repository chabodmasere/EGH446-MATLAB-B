function clr = clearance_field(wall_pts, r)
%CLEARANCE_FIELD  Distance [m] from every grid point to the nearest wall point.
%   Exact Euclidean distance out to r cells, capped at r beyond that.
%   Same shift scheme as inflate_map, so clr <= 0.6 is exactly inflate_map(wall_pts, 6).

    [n, m] = size(wall_pts);
    clr = r*ones(n, m);

    for dr = -r:r
        for dc = -r:r
            d = hypot(dr, dc);
            if d > r
                continue
            end
            shifted = false(n, m);
            rs = max(1, 1+dr):min(n, n+dr);
            cs = max(1, 1+dc):min(m, m+dc);
            shifted(rs, cs) = wall_pts(rs-dr, cs-dc);
            clr(shifted) = min(clr(shifted), d);
        end
    end

    clr = clr/10;
end
