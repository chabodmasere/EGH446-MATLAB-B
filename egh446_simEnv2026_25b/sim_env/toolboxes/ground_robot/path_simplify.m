function simple = path_simplify(path, occ)
%PATH_SIMPLIFY  Reduce a path to the fewest points with clear sight lines.
%   OCC is a fine-resolution occupancy map, already inflated by the
%   required clearance.

    n = size(path,1);
    simple = path(1,:);
    i = 1;

    while i < n
        j = n;
        while j > i+1 && ~clear_line(path(i,:), path(j,:), occ)
            j = j - 1;
        end
        simple = [simple; path(j,:)];
        i = j;
    end
end

function ok = clear_line(a, b, occ)
    nRows = size(occ,1);
    steps = max(ceil(norm(b-a)*40), 2);
    ok = true;

    for t = linspace(0, 1, steps)
        p = a + t*(b - a);
        r = nRows - round(p(2)*10);
        c = round(p(1)*10) + 1;
        if r < 1 || r > nRows || c < 1 || c > size(occ,2) || occ(r,c)
            ok = false;
            return
        end
    end
end