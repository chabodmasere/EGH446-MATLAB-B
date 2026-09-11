function [D, paths] = waypoint_distances(wps, logical_map)
    n = size(wps, 1);
    D = zeros(n);
    paths = cell(n);

    for i = 1:n-1
        for j = i+1:n
            [p, d] = A_star(wps(i,:), wps(j,:), logical_map, false);
            D(i,j) = d;
            D(j,i) = d;
            paths{i,j} = p;
            paths{j,i} = flipud(p);
        end
    end
end