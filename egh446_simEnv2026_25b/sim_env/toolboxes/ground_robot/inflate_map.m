function inflated = inflate_map(logical_map, r)
    %INFLATE_MAP  Grow occupied cells by r cells in all directions.
    
    [n, m] = size(logical_map);
    inflated = logical_map;
    
    for dr = -r:r
        for dc = -r:r
            if dr^2 + dc^2 > r^2
                continue
            end
            shifted = false(n, m);
            rs = max(1, 1+dr):min(n, n+dr);
            cs = max(1, 1+dc):min(m, m+dc);
            shifted(rs, cs) = logical_map(rs-dr, cs-dc);
            inflated = inflated | shifted;
        end
    end
end